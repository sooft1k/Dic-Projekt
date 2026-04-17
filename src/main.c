#define F_CPU 16000000UL 

#include <avr/io.h>       
#include <avr/interrupt.h>// ISR, sei()
#include <util/delay.h>   // 
#include <util/atomic.h>  // ATOMIC_BLOCK — Interrupt-sicherer Zugriff
#include <stdlib.h>       // 
#include "motors.h"       
#include "sensors.h"      
#include "bt.h"           

#define OBSTACLE_DISTANCE_CM    20  // Hindernis-Schwellwert in cm
#define REMOTE_START_BYTE       0xFF// Startbyte eines Steuerpakets
#define REMOTE_TIMEOUT_MS       100 // Max. Wartezeit auf Byte in ms
#define REMOTE_THRESHOLD        100 // Mindestwert für Bewegungsbefehl (-200 bis +200)

typedef enum { MODE_AUTONOMOUS, MODE_MANUEL } RobotMode; // Zwei Betriebsmodi
typedef enum { AUTO_FORWARD, AUTO_REVERSE, AUTO_TURN, AUTO_PAUSE } AutoState; // 4 Zustände autonomer Modus

volatile RobotMode current_mode = MODE_AUTONOMOUS; // Aktueller Modus — volatile wegen ISR-Zugriff
static AutoState auto_state = AUTO_FORWARD;        // Aktueller Zustand State-Machine
volatile uint16_t state_timer_ms = 0;              // Timer-Zähler — wird von ISR jede ms dekrementiert

void timer0_init(void) {
    TCCR0A = (1 << WGM01);              // CTC-Modus — zählt bis OCR0A dann zurück auf 0
    TCCR0B = (1 << CS01) | (1 << CS00);// Prescaler 64 → 250.000 Ticks/Sek bei 16MHz
    OCR0A  = 249;                        // 250.000 / 250 = 1.000 Interrupts/Sek = 1ms Takt
    TIMSK0 = (1 << OCIE0A);             // Compare-Interrupt aktivieren
}

ISR(TIMER0_COMPA_vect) {               // Wird automatisch jede 1ms aufgerufen
    if (state_timer_ms > 0) state_timer_ms--; // Timer herunterzählen
}

void toggle_mode(void) {
    if (current_mode == MODE_AUTONOMOUS) { // Wechsel zu manuell
        current_mode = MODE_MANUEL;
        motor_set_speed(180);              // Geschwindigkeit für manuellen Modus
    } else {                               // Wechsel zu autonom
        current_mode = MODE_AUTONOMOUS;
        auto_state   = AUTO_FORWARD;       // State-Machine zurücksetzen
        motor_set_speed(180);              // Geschwindigkeit für autonomen Modus
        motor_stop();                      // Motoren stoppen beim Wechsel
    }
}

void check_mode_switch(void) {
    if (!bt_data_available()) return;      // Kein Byte verfügbar — nichts tun
    uint8_t b = bt_receive();             // Byte lesen
    if (b == 0xFE) toggle_mode();         // 0xFE = Moduswechsel-Befehl vom Controller
}

static uint8_t remote_receive_timeout(uint8_t *timeout) {
    uint16_t ms = 0;
    while (!bt_data_available()) {         // Warten bis Byte verfügbar
        _delay_ms(1);
        if (++ms >= REMOTE_TIMEOUT_MS) {   // Nach 100ms — Timeout
            *timeout = 1;
            return 0;
        }
    }
    return bt_receive();                   // Byte zurückgeben
}

void process_remote(void) {
    uint8_t timeout = 0;
    uint16_t sync = 0;

    uint8_t first = remote_receive_timeout(&timeout);
    if (timeout) { motor_stop(); return; } // Timeout — Motor stoppen

    if (first == 0xFE) {                   // Moduswechsel mitten im manuellen Modus
        toggle_mode();
        return;
    }

    if (first != REMOTE_START_BYTE) {      // Kein Startbyte — Synchronisation verloren
        if (++sync > 3) { motor_stop(); return; } // Nach 3 Versuchen aufgeben
        return;
    }

    uint8_t data[4];
    for (uint8_t i = 0; i < 4; i++) {     // 4 Datenbytes lesen (x_high, x_low, y_high, y_low)
        data[i] = remote_receive_timeout(&timeout);
        if (timeout) { motor_stop(); return; }
    }

    // 2 Bytes zu 16-Bit Wert zusammensetzen
    int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]); // Links/Rechts (-200 bis +200)
    int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]); // Vorwärts/Rückwärts (-200 bis +200)

    if (y > REMOTE_THRESHOLD) {            // L2 gedrückt — vorwärts
        if      (x >  REMOTE_THRESHOLD) motor_curve_right();      // + rechter Stick rechts
        else if (x < -REMOTE_THRESHOLD) motor_curve_left();       // + rechter Stick links
        else                            motor_forward();           // geradeaus
    } else if (y < -REMOTE_THRESHOLD) {   // R2 gedrückt — rückwärts
        if      (x >  REMOTE_THRESHOLD) motor_backward_curve_right();
        else if (x < -REMOTE_THRESHOLD) motor_backward_curve_left();
        else                            motor_backward();
    } else if (x >  REMOTE_THRESHOLD)  motor_turn_right(); // Rechter Stick rechts — auf der Stelle drehen
    else if   (x < -REMOTE_THRESHOLD)  motor_turn_left();  // Rechter Stick links — auf der Stelle drehen
    else                               motor_stop();        // Nichts gedrückt — stoppen
}

void autonomous_mode(void) {
    check_mode_switch();                   // Auf Moduswechsel prüfen

    uint16_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = state_timer_ms; } // Timer atomar lesen
    if (t > 0) return;                     // Timer läuft noch — warten

    switch (auto_state) {
        case AUTO_FORWARD: {               // Vorwärts fahren und Sensor prüfen
            int d = read_distance();
            if (d > 0 && d < OBSTACLE_DISTANCE_CM) { // Hindernis erkannt
                motor_stop();
                auto_state = AUTO_REVERSE;
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { state_timer_ms = 500; } // 500ms warten
            } else {
                motor_forward();           // Weg frei — vorwärts
            }
            break;
        }
        case AUTO_REVERSE:                 // Rückwärts fahren
            motor_backward();
            auto_state = AUTO_TURN;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { state_timer_ms = 2000; } // 2 Sek rückwärts
            break;

        case AUTO_TURN:                    // Rechts drehen
            motor_turn_right();
            auto_state = AUTO_PAUSE;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { state_timer_ms = 800; } // 800ms drehen
            break;

        case AUTO_PAUSE:                   // Kurze Pause
            motor_stop();
            auto_state = AUTO_FORWARD;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { state_timer_ms = 200; } // 200ms warten
            break;
    }
}

int main(void) {
    motor_init();   // Motor-Pins und PWM initialisieren
    bt_init();      // UART für Bluetooth initialisieren
    sensor_init();  // Ultraschall-Sensor initialisieren
    timer0_init();  // Timer0 als 1ms Takt starten

    sei();          // Alle Interrupts global aktivieren
    _delay_ms(1000);// 1 Sek warten — ESP32 Zeit zum Verbinden geben
    motor_set_speed(180); // Startgeschwindigkeit setzen

    while (1) {                            // Endlosschleife
        check_mode_switch();               // Auf Moduswechsel prüfen

        if (current_mode == MODE_AUTONOMOUS)
            autonomous_mode();             // Sensor steuert
        else
            process_remote();              // Controller steuert
    }
}