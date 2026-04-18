#define F_CPU 16000000UL  // Arduino Takt: 16 Millionen Schritte pro Sekunde

#include <avr/io.h>         // Gibt Zugriff auf alle Hardware-Pins und Register
#include <avr/interrupt.h>  // Ermöglicht Interrupts (automatische Funktionsaufrufe)
#include <util/delay.h>     // Gibt _delay_ms() und _delay_us() zum Warten
#include <util/atomic.h>    // Schützt wichtige Operationen vor Unterbrechungen
#include <stdlib.h>         // Standard C Hilfsfunktionen
#include "motors.h"
#include "sensors.h"
#include "bt.h"

#define OBSTACLE_DISTANCE_CM 20  // Unter 20cm = Hindernis erkannt
#define REMOTE_START_BYTE 0xFF   // Jedes Steuerpaket beginnt mit diesem Byte (255)
#define REMOTE_TIMEOUT_MS 100    // Nach 100ms ohne Daten → Verbindung unterbrochen
#define REMOTE_THRESHOLD 100     // Stick muss über 100 bewegt werden um zu reagieren

typedef enum { MODE_AUTONOMOUS, MODE_MANUEL } RobotMode;  // Zwei Fahrmodi: Auto oder Manuell
typedef enum {
  AUTO_FORWARD,  // Vorwärts fahren
  AUTO_REVERSE,  // Rückwärts fahren
  AUTO_TURN,     // Kurve machen
  AUTO_PAUSE     // Kurz anhalten und messen
} AutoState;

volatile RobotMode current_mode =
    MODE_AUTONOMOUS;  // Aktueller Modus (volatile = ISR kann ihn ändern)
static AutoState  auto_state     = AUTO_FORWARD;  // Aktueller Schritt im autonomen Ablauf
volatile uint16_t state_timer_ms = 0;             // Countdown-Timer in Millisekunden

void timer0_init(void) {
  TCCR0A = (1 << WGM01);               // Timer0 im CTC-Modus: zählt bis OCR0A, dann Reset
  TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64 → bei 16MHz = 250.000 Ticks/Sek
  OCR0A  = 249;                        // Bei 249 auslösen → 250.000/250 = 1000x pro Sek = 1ms
  TIMSK0 = (1 << OCIE0A);              // Erlaubt dem Timer einen Interrupt auszulösen
}

ISR(TIMER0_COMPA_vect) {  // Diese Funktion wird automatisch jede 1ms aufgerufen
  if (state_timer_ms > 0)
    state_timer_ms--;  // Timer um 1 verringern
}

void toggle_mode(void) {
  if (current_mode == MODE_AUTONOMOUS) {  // War autonom → wechsle zu manuell
    current_mode = MODE_MANUEL;
    motor_set_speed(220);  // Etwas schneller für manuelle Steuerung
  } else {                 // War manuell → wechsle zu autonom
    current_mode = MODE_AUTONOMOUS;
    auto_state   = AUTO_FORWARD;  // Autonomen Ablauf von vorne starten
    motor_set_speed(200);         // Etwas langsamer für autonomen Modus
    motor_stop();                 // Zuerst anhalten bevor autonom gestartet wird
  }
}

void check_mode_switch(void) {
  if (!bt_data_available())
    return;                  // Kein Byte da → nichts tun
  uint8_t b = bt_receive();  // Byte vom ESP32 lesen
  if (b == 0xFE)
    toggle_mode();  // 0xFE = Spezialbefehl: Modus wechseln
}

static uint8_t remote_receive_timeout(uint8_t* timeout) {
  uint16_t ms = 0;
  while (!bt_data_available()) {  // Warten bis ein Byte ankommt
    _delay_ms(1);
    if (++ms >= REMOTE_TIMEOUT_MS) {  // Nach 100ms aufgeben
      *timeout = 1;
      return 0;
    }
  }
  return bt_receive();  // Byte zurückgeben
}

void process_remote(void) {
  uint8_t  timeout = 0;
  uint16_t sync    = 0;

  uint8_t first = remote_receive_timeout(&timeout);
  if (timeout) {
    motor_stop();
    return;
  }  // Timeout → sicherheitshalber stoppen

  if (first == 0xFE) {  // Moduswechsel auch im manuellen Modus möglich
    toggle_mode();
    return;
  }

  if (first != REMOTE_START_BYTE) {  // Falsches Startbyte → Synchronisation verloren
    if (++sync > 3) {
      motor_stop();
      return;
    }  // Nach 3 Fehlern aufgeben
    return;
  }

  uint8_t data[4];
  for (uint8_t i = 0; i < 4; i++) {  // 4 Bytes lesen: x-Achse (2 Bytes) + y-Achse (2 Bytes)
    data[i] = remote_receive_timeout(&timeout);
    if (timeout) {
      motor_stop();
      return;
    }
  }

  // Zwei einzelne Bytes zu einem 16-Bit Wert zusammenbauen
  int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);  // Links/Rechts: -200 bis +200
  int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);  // Vor/Zurück:   -200 bis +200

  if (y > REMOTE_THRESHOLD) {  // L2 gedrückt → vorwärts
    if (x > REMOTE_THRESHOLD)
      motor_curve_right();  // + rechter Stick rechts → Kurve
    else if (x < -REMOTE_THRESHOLD)
      motor_curve_left();  // + rechter Stick links → Kurve
    else
      motor_forward();                 // nur L2 → geradeaus
  } else if (y < -REMOTE_THRESHOLD) {  // R2 gedrückt → rückwärts
    if (x > REMOTE_THRESHOLD)
      motor_backward_curve_right();
    else if (x < -REMOTE_THRESHOLD)
      motor_backward_curve_left();
    else
      motor_backward();
  } else if (x > REMOTE_THRESHOLD)
    motor_turn_right();  // Rechter Stick → auf der Stelle drehen
  else if (x < -REMOTE_THRESHOLD)
    motor_turn_left();
  else
    motor_stop();  // Nichts gedrückt → stoppen
}

void autonomous_mode(void) {
  check_mode_switch();  // Immer prüfen ob Modus gewechselt werden soll

  uint16_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t = state_timer_ms;
  }  // Timer sicher lesen
  if (t > 0)
    return;  // Timer läuft noch → warten, nichts tun

  switch (auto_state) {
    case AUTO_FORWARD: {
      int d = read_distance();                  // Distanz zum nächsten Hindernis messen
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) {  // Hindernis unter 20cm
        motor_stop();
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 800;
        }  // 800ms warten
      } else {
        motor_forward();  // Kein Hindernis → vorwärts fahren
      }
      break;
    }

    case AUTO_REVERSE:
      motor_backward();  // Kurz zurückfahren um Platz zu schaffen
      auto_state = AUTO_TURN;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 500;
      }  // 500ms zurück
      break;

    case AUTO_TURN:
      motor_curve_right();  // Kurve nach rechts versuchen um Hindernis zu umfahren
      auto_state = AUTO_PAUSE;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 1500;
      }  // 1.5 Sek Kurve
      break;

    case AUTO_PAUSE: {
      motor_stop();
      int d = read_distance();  // Nochmal messen: ist der Weg jetzt frei?
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) {
        // Immer noch Hindernis → wahrscheinlich eine Wand → nochmal zurück
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 300;
        }
      } else {
        // Weg frei → kleines Hindernis umfahren → weiterfahren
        auto_state = AUTO_FORWARD;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 500;
        }
      }
      break;
    }
  }
}

int main(void) {
  motor_init();   // Alle Motor-Pins einrichten und PWM starten
  bt_init();      // Serielle Verbindung zum ESP32 einrichten (9600 Baud)
  sensor_init();  // Ultraschall-Sensor-Pins einrichten
  timer0_init();  // Timer0 starten → erzeugt 1ms Takt für State-Machine

  sei();                 // Alle Interrupts einschalten (ohne das läuft gar nichts)
  _delay_ms(1000);       // 1 Sekunde warten → ESP32 und Sensor Zeit zum Starten
  motor_set_speed(200);  // Startgeschwindigkeit auf 200 von 255 setzen

  while (1) {             // Endlosschleife → läuft solange der Arduino Strom hat
    check_mode_switch();  // Prüfen ob X-Taste gedrückt wurde

    if (current_mode == MODE_AUTONOMOUS)
      autonomous_mode();  // Sensor übernimmt die Steuerung
    else
      process_remote();  // PS4/PS5 Controller übernimmt die Steuerung
  }
}