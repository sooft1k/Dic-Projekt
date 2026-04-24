// main.c – Hauptprogramm des Roboter-Autos
// Verwaltet zwei Modi: Autonomer Modus (Hindernisse ausweichen)
// und Manueller Modus (Fernsteuerung per PS5-Controller via ESP32/Bluetooth)

#define F_CPU 16000000UL  // Taktfrequenz: 16 MHz (ATmega328P Standard)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>  // Für ATOMIC_BLOCK – schützt kritische Codestellen
#include <stdlib.h>
#include "motors.h"
#include "sensors.h"
#include "bt.h"

// --- Konstanten ---
#define OBSTACLE_DISTANCE_CM 20  // Hindernis erkannt wenn näher als 20 cm
#define REMOTE_START_BYTE 0xFF   // Erstes Byte eines gültigen Steuerpakets
#define REMOTE_TIMEOUT_MS 50     // Warten max. 50 ms auf das nächste Byte
#define REMOTE_THRESHOLD 100     // Joystick-Totzone: unter 100 = ignorieren

// --- Zustände ---

// Die zwei Hauptmodi des Roboters
typedef enum { MODE_AUTONOMOUS, MODE_MANUEL } RobotMode;

// Die vier Phasen im autonomen Modus
typedef enum { AUTO_FORWARD, AUTO_REVERSE, AUTO_TURN, AUTO_PAUSE } AutoState;

// --- Globale Variablen ---
volatile RobotMode current_mode   = MODE_AUTONOMOUS;  // Startet im Auto-Modus
static AutoState   auto_state     = AUTO_FORWARD;     // Autonomer Startzustand
volatile uint16_t  state_timer_ms = 0;                // Countdown-Timer in ms
                                                      // volatile = wird im ISR verändert

// --- Timer 0: 1ms Interrupt ---
// Timer 0 erzeugt alle 1 ms einen Interrupt → wird für Zeitsteuerung genutzt
void timer0_init(void) {
  TCCR0A = (1 << WGM01);               // CTC-Modus: zählt bis OCR0A dann Reset
  TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64 → 16MHz/64 = 250kHz
  OCR0A  = 249;                        // 250kHz / 250 = 1000 Hz = 1 ms pro Tick
  TIMSK0 = (1 << OCIE0A);              // Interrupt bei Erreichen von OCR0A
}

// Wird automatisch aufgerufen alle 1 ms
// Zählt state_timer_ms herunter bis 0
ISR(TIMER0_COMPA_vect) {
  if (state_timer_ms > 0) state_timer_ms--;
}

// --- Moduswechsel ---

// Wechselt zwischen autonomem und manuellem Modus
void toggle_mode(void) {
  if (current_mode == MODE_AUTONOMOUS) {
    current_mode = MODE_MANUEL;  // Wechsel zu manuell
    motor_set_speed(220);
  } else {
    current_mode = MODE_AUTONOMOUS;  // Wechsel zu autonom
    auto_state   = AUTO_FORWARD;     // Autonomen Zustand zurücksetzen
    motor_set_speed(220);
    motor_stop();  // Sicherheitshalber stoppen
  }
}

// Prüft ob ein Moduswechsel-Befehl (0xFE) per Bluetooth angekommen ist
void check_mode_switch(void) {
  if (!bt_data_available()) return;  // Nichts empfangen → nichts tun
  uint8_t b = bt_receive();
  if (b == 0xFE) toggle_mode();  // 0xFE = Sonderbefehl: Modus wechseln
}

// --- Manueller Modus: Datenempfang ---

// Wartet auf ein Byte vom ESP32, maximal REMOTE_TIMEOUT_MS Millisekunden
// Setzt *timeout auf 1 wenn die Zeit abläuft
static uint8_t remote_receive_timeout(uint8_t* timeout) {
  uint16_t ms = 0;
  while (!bt_data_available()) {
    _delay_ms(1);
    if (++ms >= REMOTE_TIMEOUT_MS) {
      *timeout = 1;  // Timeout markieren
      return 0;
    }
  }
  return bt_receive();  // Byte zurückgeben wenn rechtzeitig angekommen
}

// Verarbeitet ein vollständiges Steuerpaket vom PS5-Controller
// Paketformat: [0xFF] [X-High] [X-Low] [Y-High] [Y-Low]
// X = rechter Stick horizontal, Y = rechter Stick vertikal
void process_remote(void) {
  uint8_t  timeout = 0;
  uint16_t sync    = 0;

  // Erstes Byte lesen
  uint8_t first = remote_receive_timeout(&timeout);
  if (timeout) {
    motor_stop();
    return;
  }  // Kein Signal → stoppen
  if (first == 0xFE) {
    toggle_mode();
    return;
  }                                  // Moduswechsel-Befehl
  if (first != REMOTE_START_BYTE) {  // Kein gültiger Paketstart
    if (++sync > 3) {
      motor_stop();
      return;
    }  // Nach 3 Fehlversuchen stoppen
    return;
  }

  // 4 Datenbytes lesen (X und Y, je 2 Bytes = 16-Bit Wert)
  uint8_t data[4];
  for (uint8_t i = 0; i < 4; i++) {
    data[i] = remote_receive_timeout(&timeout);
    if (timeout) {
      motor_stop();
      return;
    }
  }

  // Zwei Bytes zu einem 16-Bit Signed Integer zusammensetzen
  int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);  // Rechts/Links
  int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);  // Vor/Zurück

  // Steuerlogik: Y-Achse bestimmt ob vorwärts, rückwärts oder Drehung
  // X-Achse bestimmt ob geradeaus oder Kurve
  if (y > REMOTE_THRESHOLD) {  // Stick nach vorne
    if (x > REMOTE_THRESHOLD)
      motor_curve_right();
    else if (x < -REMOTE_THRESHOLD)
      motor_curve_left();
    else
      motor_forward();
  } else if (y < -REMOTE_THRESHOLD) {  // Stick nach hinten
    if (x > REMOTE_THRESHOLD)
      motor_backward_curve_right();
    else if (x < -REMOTE_THRESHOLD)
      motor_backward_curve_left();
    else
      motor_backward();
  } else if (x > REMOTE_THRESHOLD)  // Stick nur seitlich
    motor_turn_right();
  else if (x < -REMOTE_THRESHOLD)
    motor_turn_left();
  else
    motor_stop();  // Stick in Mitte → stoppen
}

// --- Autonomer Modus ---

// Zustandsmaschine: fährt vorwärts, weicht Hindernissen aus
// Zustände: FORWARD → REVERSE → TURN → PAUSE → FORWARD → ...
void autonomous_mode(void) {
  static uint8_t turn_direction = 0;  // Merkt sich letzte Drehrichtung

  check_mode_switch();  // Immer prüfen ob manuell gewechselt werden soll

  // Timer auslesen (ATOMIC = sicher, da Timer im Interrupt verändert wird)
  uint16_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t = state_timer_ms;
  }
  if (t > 0) return;  // Noch im Wartezustand → nichts tun

  switch (auto_state) {
    case AUTO_FORWARD: {
      int d = read_distance();
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) {  // Hindernis erkannt!
        motor_stop();
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 800;
        }  // 800ms warten
      } else {
        motor_forward();  // Kein Hindernis → einfach weiterfahren
      }
      break;
    }

    case AUTO_REVERSE:
      motor_backward();  // Kurz rückwärts fahren
      auto_state = AUTO_TURN;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 500;
      }  // 500ms rückwärts
      break;

    case AUTO_TURN:
      // Abwechselnd links und rechts drehen
      if (turn_direction == 0)
        motor_curve_left();
      else
        motor_curve_right();
      auto_state = AUTO_PAUSE;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 1500;
      }  // 1.5s drehen
      break;

    case AUTO_PAUSE: {
      motor_stop();
      int d = read_distance();
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) {  // Immer noch Hindernis?
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 300;
        }  // Nochmal zurück
      } else {
        turn_direction = !turn_direction;  // Nächste Drehrichtung umschalten
        auto_state     = AUTO_FORWARD;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 500;
        }
      }
      break;
    }
  }
}

// --- Hauptprogramm ---
int main(void) {
  motor_init();   // Motorpins und PWM initialisieren
  bt_init();      // UART für Bluetooth-Empfang initialisieren
  sensor_init();  // Ultraschallsensor-Pins initialisieren
  timer0_init();  // 1ms-Timer initialisieren

  sei();            // Interrupts global aktivieren (nötig für Timer-ISR)
  _delay_ms(1000);  // 1 Sekunde warten damit alles hochgefahren ist

  motor_set_speed(200);  // Startgeschwindigkeit setzen

  // Endlosschleife: läuft für immer
  while (1) {
    check_mode_switch();  // Moduswechsel prüfen

    if (current_mode == MODE_AUTONOMOUS)
      autonomous_mode();  // Autonomes Fahren
    else
      process_remote();  // Fernsteuerung verarbeiten
  }
}