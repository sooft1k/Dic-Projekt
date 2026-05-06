#define F_CPU 16000000UL /* CPU-Takt: 16 Millionen Schwingungen pro Sekunde */

#include <avr/io.h>        /* Gibt Zugriff auf alle Hardware-Register des ATmega328P */
#include <avr/interrupt.h> /* Ermöglicht ISR()-Funktionen und sei()/cli() */
#include <util/delay.h>    /* _delay_ms() und _delay_us() – Warte-Funktionen */
#include <util/atomic.h>   /* ATOMIC_BLOCK – schützt kritische Code-Stellen vor Interrupts */
#include <stdlib.h>        /* Standard C Bibliothek */
#include "motors.h"        /* Unsere eigene Motorsteuerung */
#include "sensors.h"       /* Unsere eigene Sensorauswertung */
#include "bt.h"            /* Unsere eigene Bluetooth/UART-Kommunikation */

/* ── Konfigurationswerte ────────────────────────────────────────────────── */
#define OBSTACLE_DISTANCE_CM 20 /* Hindernis erkannt wenn Sensor < 20 cm misst */
#define REMOTE_START_BYTE 0xFF  /* 0xFF = 255 = erstes Byte jedes Steuerpakets */
#define REMOTE_TIMEOUT_MS 50    /* Wartet max. 50 ms auf das nächste Byte */
#define REMOTE_THRESHOLD 100 /* Joystick-Totzone: Werte zwischen -100 und +100 = keine Bewegung */

/* ── Betriebsmodi ───────────────────────────────────────────────────────── */
/*
 * typedef enum erstellt einen eigenen Datentyp mit festen erlaubten Werten.
 * So ist klar was ein Wert bedeutet statt einfach 0 oder 1 zu schreiben.
 */
typedef enum { MODE_AUTONOMOUS, MODE_MANUEL } RobotMode;
/*   MODE_AUTONOMOUS = Roboter fährt selbst
 *   MODE_MANUEL     = Controller steuert */

typedef enum { AUTO_FORWARD, AUTO_REVERSE, AUTO_TURN, AUTO_PAUSE } AutoState;
/*   AUTO_FORWARD = vorwärts fahren und messen
 *   AUTO_REVERSE = rückwärts fahren
 *   AUTO_TURN    = Kurve machen
 *   AUTO_PAUSE   = stoppen und nochmal messen */

/* ── Globale Variablen ──────────────────────────────────────────────────── */
volatile RobotMode current_mode   = MODE_AUTONOMOUS; /* Startet im autonomen Modus */
static AutoState   auto_state     = AUTO_FORWARD;    /* Startzustand der State-Machine */
volatile uint16_t  state_timer_ms = 0;               /* Countdown-Timer in Millisekunden */

void timer0_init(void) {
  TCCR0A = (1 << WGM01);              /* CTC-Modus aktivieren (zählt bis OCR0A, dann Reset) */
  TCCR0B = (1 << CS01) | (1 << CS00); /* Prescaler 64: 16MHz / 64 = 250.000 Ticks/Sek */
  OCR0A  = 249;                       /* Zählziel: bei 249 Interrupt auslösen → 1ms Takt */
  TIMSK0 = (1 << OCIE0A);             /* Timer0 Compare-Interrupt erlauben */
}

ISR(TIMER0_COMPA_vect) {
  if (state_timer_ms > 0) state_timer_ms--; /* Jede ms um 1 verringern bis 0 */
}

void toggle_mode(void) {
  if (current_mode == MODE_AUTONOMOUS) { /* War autonom → wechsle zu manuell */
    current_mode = MODE_MANUEL;
    motor_set_speed(255); /* Geschwindigkeit für manuellen Modus */
  } else {                /* War manuell → wechsle zu autonom */
    current_mode = MODE_AUTONOMOUS;
    auto_state   = AUTO_FORWARD; /* State-Machine von vorne beginnen */
    motor_set_speed(255);
    motor_stop(); /* Sofort stoppen damit der Roboter nicht unkontrolliert weiterfährt */
  }
}

void check_mode_switch(void) {
  if (!bt_data_available()) return; /* Kein Byte angekommen → sofort zurück */
  uint8_t b = bt_receive();         /* Byte lesen */
  if (b == 0xFE) toggle_mode();     /* 0xFE = Sonderbyte für Moduswechsel */
}

static uint8_t remote_receive_timeout(uint8_t* timeout) {
  uint16_t ms = 0;
  while (!bt_data_available()) { /* Warten bis Byte angekommen */
    _delay_ms(1);
    if (++ms >= REMOTE_TIMEOUT_MS) { /* ++ = um 1 erhöhen vor dem Vergleich */
      *timeout = 1;                  /* Timeout! Fehler markieren */
      return 0;
    }
  }
  return bt_receive(); /* Byte zurückgeben */
}

void process_remote(void) {
  uint8_t  timeout = 0;
  uint16_t sync    = 0;

  uint8_t first = remote_receive_timeout(&timeout); /* & = Adresse der Variable übergeben */
  if (timeout) {
    motor_stop();
    return;
  } /* Timeout → Notbremse */

  if (first == 0xFE) {
    toggle_mode();
    return;
  } /* Moduswechsel-Befehl */

  if (first != REMOTE_START_BYTE) { /* Kein gültiges Startbyte */
    if (++sync > 3) {
      motor_stop();
      return;
    } /* Nach 3 Fehlern aufgeben */
    return;
  }

  uint8_t data[4]; /* Array = 4 aufeinanderfolgende Bytes im Speicher */
  for (uint8_t i = 0; i < 4; i++) {
    data[i] = remote_receive_timeout(&timeout);
    if (timeout) {
      motor_stop();
      return;
    }
  }

  /* Zwei Bytes zu einem 16-Bit Wert zusammenbauen */
  int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]); /* Links/Rechts */
  int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]); /* Vor/Zurück */

  /* Joystick-Werte in Fahrbefehle übersetzen */
  if (y > REMOTE_THRESHOLD) { /* L2 gedrückt → vorwärts */
    if (x > REMOTE_THRESHOLD)
      motor_curve_right();
    else if (x < -REMOTE_THRESHOLD)
      motor_curve_left();
    else
      motor_forward();
  } else if (y < -REMOTE_THRESHOLD) { /* R2 gedrückt → rückwärts */
    if (x > REMOTE_THRESHOLD)
      motor_backward_curve_right();
    else if (x < -REMOTE_THRESHOLD)
      motor_backward_curve_left();
    else
      motor_backward();
  } else if (x > REMOTE_THRESHOLD)
    motor_turn_right(); /* Rechter Stick → drehen */
  else if (x < -REMOTE_THRESHOLD)
    motor_turn_left();
  else
    motor_stop(); /* Nichts gedrückt → stoppen */
}

void autonomous_mode(void) {
  static uint8_t turn_direction = 0; /* 0 = links drehen, 1 = rechts drehen */

  check_mode_switch(); /* Immer prüfen ob Modus gewechselt werden soll */

  uint16_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t = state_timer_ms;
  } /* Timer sicher lesen */
  if (t > 0) return; /* Timer läuft noch → in diesem Zustand bleiben */

  switch (auto_state) {
    case AUTO_FORWARD: {                       /* Vorwärts fahren und Sensor prüfen */
      int d = read_distance();                 /* Distanz in cm messen */
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) { /* Hindernis unter 20 cm */
        motor_stop();
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 800;
        } /* 800 ms warten */
      } else {
        motor_forward(); /* Kein Hindernis → weiterfahren */
      }
      break;
    }

    case AUTO_REVERSE: /* Rückwärts fahren */
      motor_backward();
      auto_state = AUTO_TURN;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 500;
      } /* 500 ms rückwärts */
      break;

    case AUTO_TURN: /* Kurve machen – immer in gleicher Richtung bis Weg frei */
      if (turn_direction == 0)
        motor_curve_left();
      else
        motor_curve_right();
      auto_state = AUTO_PAUSE;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 1500;
      } /* 1,5 Sek drehen */
      break;

    case AUTO_PAUSE: { /* Stoppen und nochmal messen */
      motor_stop();
      int d = read_distance();
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) { /* Immer noch Hindernis → gleiche Richtung */
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 300;
        }
      } else {                            /* Weg frei → Richtung wechseln für nächstes Hindernis */
        turn_direction = !turn_direction; /* ! = logisches NICHT: 0→1 oder 1→0 */
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
  motor_init();  /* Motorpins und PWM konfigurieren */
  bt_init();     /* UART für ESP32-Kommunikation (9600 Baud) */
  sensor_init(); /* Ultraschall-Sensor-Pins einrichten */
  timer0_init(); /* 1-ms-Timer starten */

  sei();                /* Alle Interrupts global einschalten – NACH den init-Funktionen */
  _delay_ms(1000);      /* 1 Sekunde warten: ESP32 und Sensor brauchen Zeit zum Starten */
  motor_set_speed(255); /* Startgeschwindigkeit auf 255 von 255 setzen */

  while (1) {            /* Endlosschleife – läuft solange der Arduino Strom hat */
    check_mode_switch(); /* Wurde X-Taste gedrückt? */

    if (current_mode == MODE_AUTONOMOUS)
      autonomous_mode(); /* Sensor übernimmt die Steuerung */
    else
      process_remote(); /* PS5 Controller übernimmt die Steuerung */
  }
}