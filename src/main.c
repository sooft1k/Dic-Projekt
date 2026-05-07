#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>  // ATOMIC_BLOCK - schützt Code vor Interrupts (siehe unten)
#include <stdlib.h>
#include "motors.h"
#include "sensors.h"
#include "bt.h"

#define OBSTACLE_DISTANCE_CM 20  // Hindernis erkannt wenn Sensor < 20 cm misst
#define REMOTE_START_BYTE 0xFF   // Startbyte für PS5 Controller Pakete
#define REMOTE_TIMEOUT_MS 50     // Wartet max. 50 ms auf das nächste Byte
#define REMOTE_THRESHOLD 100     // Joystick-Totzone: Werte zwischen -100 und +100 = keine Bewegung

typedef enum { MODE_AUTONOMOUS, MODE_MANUEL } RobotMode;  // 2 Modi des Autos
typedef enum {
  AUTO_FORWARD,
  AUTO_REVERSE,
  AUTO_TURN,
  AUTO_PAUSE
} AutoState;  // Zustände der autonomen State-Machine

volatile RobotMode current_mode   = MODE_AUTONOMOUS;  // Startet im autonomen Modus
static AutoState   auto_state     = AUTO_FORWARD;     // Startzustand der State-Machine
volatile uint16_t  state_timer_ms = 0;                // Countdown-Timer in Millisekunden

// TCCR0A, TCCR0B = "Timer Counter Control Register" - konfigurieren den Timer
// OCR0A = "Output Compare Register" - Vergleichswert
// TIMSK0 = "Timer Interrupt Mask" - erlaubt den Interrupt
// Timer 0 zählt von 0 bis OCR0A (249), dann wird der Interrupt ausgelöst und der Timer
// zurückgesetzt.
void timer0_init(void) {
  TCCR0A = (1 << WGM01);               // CTC-Modus: Timer zählt bis OCR0A, dann Reset
  TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64: 16MHz / 64 = 250.000 Ticks/Sek
  OCR0A  = 249;                        // Bei 249 Interrupt auslösen → 250.000/250 = 1ms Takt
  TIMSK0 = (1 << OCIE0A);              // Timer0 Compare-Interrupt erlauben
}

// ── ISR = Interrupt Service Routine ──
//
// Diese Funktion wird automatisch alle 1ms vom Timer aufgerufen.
// TIMER0_COMPA_vect ist der "Vektor" (Adresse) der diesen Interrupt identifiziert.
//
// Hier zählen wir state_timer_ms herunter. So entsteht ein präziser Countdown
// ohne dass das Hauptprogramm warten muss.
ISR(TIMER0_COMPA_vect) {
  if (state_timer_ms > 0) state_timer_ms--;  // Jede ms um 1 verringern bis 0
}

void toggle_mode(void) {
  if (current_mode == MODE_AUTONOMOUS) {
    current_mode = MODE_MANUEL;
    motor_set_speed(255);
  } else {
    current_mode = MODE_AUTONOMOUS;
    auto_state   = AUTO_FORWARD;
    motor_set_speed(255);
    motor_stop();
  }
}

void check_mode_switch(void) {
  if (!bt_data_available()) return;  // Kein Byte da → sofort zurück
  uint8_t b = bt_receive();
  if (b == 0xFE) toggle_mode();  // 0xFE = Sonderbyte für Moduswechsel
}

// ── Byte vom ESP32 empfangen mit Timeout ──
// uint8_t* timeout = Zeiger auf eine Variable.
// Mit "*timeout = 1" schreibt die Funktion direkt in die Variable des Aufrufers.
// Gibt zwei Infos zurück: das Byte UND ob ein Timeout passiert ist.
static uint8_t remote_receive_timeout(uint8_t* timeout) {
  uint16_t ms = 0;
  while (!bt_data_available()) {
    _delay_ms(1);
    if (++ms >= REMOTE_TIMEOUT_MS) {  // ++ms = um 1 erhöhen vor dem Vergleich
      *timeout = 1;
      return 0;
    }
  }
  return bt_receive();
}

void process_remote(void) {
  uint8_t  timeout = 0;
  uint16_t sync    = 0;

  // "&timeout" = Adresse der Variable timeout übergeben
  uint8_t first = remote_receive_timeout(&timeout);
  if (timeout) {
    motor_stop();
    return;
  }

  if (first == 0xFE) {
    toggle_mode();
    return;
  }

  // Wenn das erste Byte nicht 0xFF ist, ist das Paket kaputt
  if (first != REMOTE_START_BYTE) {
    if (++sync > 3) {
      motor_stop();
      return;
    }
    return;
  }

  uint8_t data[4];
  for (uint8_t i = 0; i < 4; i++) {
    data[i] = remote_receive_timeout(&timeout);
    if (timeout) {
      motor_stop();
      return;
    }
  }

  int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);  // Links/Rechts
  int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);  // Vor/Zurück

  // Joystick-Werte in Fahrbefehle umwandeln.
  // REMOTE_THRESHOLD verhindert dass kleine Joystick-Wackler eine Bewegung auslösen.
  if (y > REMOTE_THRESHOLD) {
    if (x > REMOTE_THRESHOLD)
      motor_curve_right();
    else if (x < -REMOTE_THRESHOLD)
      motor_curve_left();
    else
      motor_forward();
  } else if (y < -REMOTE_THRESHOLD) {
    if (x > REMOTE_THRESHOLD)
      motor_backward_curve_right();
    else if (x < -REMOTE_THRESHOLD)
      motor_backward_curve_left();
    else
      motor_backward();
  } else if (x > REMOTE_THRESHOLD)
    motor_turn_right();
  else if (x < -REMOTE_THRESHOLD)
    motor_turn_left();
  else
    motor_stop();
}

void autonomous_mode(void) {
  static uint8_t turn_direction = 0;  // 0 = links, 1 = rechts

  check_mode_switch();

  // ATOMIC_BLOCK: Code-Block der NICHT durch einen Interrupt unterbrochen werden darf.
  // Warum nötig? state_timer_ms ist 16 Bit groß. Der ATmega kann aber nur 8 Bit
  // auf einmal lesen. Das passiert in 2 Schritten:
  //   1. Untere 8 Bit lesen
  //   2. Interrupt könnte dazwischen kommen und die Variable ändern
  //   3. Obere 8 Bit lesen
  // Ergebnis: Falscher bzw. Ungenauer Wert
  // ATOMIC_BLOCK schaltet kurz alle Interrupts aus, liest die Variable komplett,
  // und schaltet Interrupts danach wieder an.
  uint16_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t = state_timer_ms;
  }
  if (t > 0) return;  // Bleibt noch in diesem Zustand weil timer noch läuft

  switch (auto_state) {
    case AUTO_FORWARD: {
      int d = read_distance();
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) {  // Hindernis erkannt
        motor_stop();                           // Sofort anhalten
        auto_state = AUTO_REVERSE;              // Fährt Rückwärts
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 800;  // Timer auf 800ms setzen
        }
      } else {
        motor_forward();  // Wenn kein Hindernis erkannt wurde fährt es weiter vorwärts
      }
      break;
    }

    case AUTO_REVERSE:
      motor_backward();
      auto_state = AUTO_TURN;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 500;
      }
      break;

    case AUTO_TURN:
      // Gleiche Richtung weiterdrehen bis Hindernis weg ist
      if (turn_direction == 0)
        motor_curve_left();
      else
        motor_curve_right();
      auto_state = AUTO_PAUSE;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 1500;
      }
      break;

    case AUTO_PAUSE: {
      motor_stop();
      int d = read_distance();
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) {
        // Hindernis noch da → gleiche Richtung weiterdrehen
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 300;
        }
      } else {
        // Weg frei → für nächstes Hindernis Richtung wechseln.
        turn_direction = !turn_direction;
        auto_state     = AUTO_FORWARD;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 500;
        }
      }
      break;
    }
  }
}

int main(void) {
  motor_init();
  bt_init();
  sensor_init();
  timer0_init();
  sei();            // sei() = "Set Enable Interrupt" - schaltet ALLE Interrupts global ein
  _delay_ms(1000);  // ESP und Sensor brauchen Zeit zum Starten (1 Sekunde)
  motor_set_speed(255);
  while (1) {
    check_mode_switch();

    if (current_mode == MODE_AUTONOMOUS)
      autonomous_mode();
    else
      process_remote();
  }
}