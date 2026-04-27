/*
 * ============================================================
 *  main.c – Hauptprogramm des 4WD Mini-Roboters
 * ============================================================
 *
 *  Was macht dieser Code?
 *  ─────────────────────
 *  Dieser Code steuert ein 4-Rad Roboter-Auto mit einem Arduino (ATmega328P).
 *  Der Roboter kann in zwei Modi betrieben werden:
 *
 *  1. AUTONOMER MODUS:
 *     Der Roboter fährt selbstständig vorwärts.
 *     Erkennt er ein Hindernis (näher als 20 cm), fährt er zurück,
 *     dreht sich und sucht einen freien Weg.
 *
 *  2. MANUELLER MODUS:
 *     Ein PS5-Controller (verbunden mit ESP32 per Bluetooth) steuert den Roboter.
 *     Der ESP32 schickt die Joystick-Werte per Kabel (UART) an den Arduino.
 *
 *  Moduswechsel: X-Taste am Controller drücken.
 *
 *  Abkürzungen in dieser Datei:
 *  ─────────────────────────────
 *  F_CPU   = CPU Frequency           – Taktfrequenz des Prozessors (16 MHz)
 *  ISR     = Interrupt Service Routine – Funktion die automatisch bei einem Ereignis aufgerufen
 * wird CTC     = Clear Timer on Compare   – Timer-Modus: zählt bis OCR0A, dann Reset OCR0A   =
 * Output Compare Register  – Vergleichswert für Timer0 TCCR0A/B= Timer Counter Control Reg–
 * Konfiguriert wie der Timer arbeitet TIMSK0  = Timer Interrupt Mask Reg – Legt fest welcher
 * Timer-Interrupt aktiv ist WGM01   = Waveform Generation Mode – Wählt den Timer-Betriebsmodus
 *  CS01/00 = Clock Select             – Wählt den Prescaler (Vorteiler) des Timers
 *  OCIE0A  = Output Compare Int Enable– Erlaubt den Timer-Interrupt
 *  sei()   = Set Enable Interrupts    – Schaltet alle Interrupts global ein
 *  volatile= C-Schlüsselwort          – Variable wird nie vom Compiler wegoptimiert
 *  uint8_t = Unsigned Integer 8-bit   – Ganzzahl 0–255
 *  uint16_t= Unsigned Integer 16-bit  – Ganzzahl 0–65535
 *  int16_t = Signed Integer 16-bit    – Ganzzahl -32768 bis +32767
 * ============================================================
 */

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
/* volatile: Diese Variable wird durch einen Interrupt verändert.
 * Ohne volatile könnte der Compiler sie "wegoptimieren" und Änderungen verpassen. */

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: timer0_init
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Richtet Timer0 so ein, dass er genau alle 1 Millisekunde einen Interrupt auslöst.
 *   Dieser Interrupt ruft automatisch ISR(TIMER0_COMPA_vect) auf.
 *
 * Warum ist sie wichtig?
 *   Ohne diesen Timer müsste der Code blockierend warten (_delay_ms).
 *   Mit dem Timer läuft die Hauptschleife weiter und state_timer_ms
 *   wird im Hintergrund heruntergezählt.
 *
 * Wie funktioniert es?
 *   CTC-Modus: Timer zählt 0 → 249, dann Reset auf 0 → löst Interrupt aus.
 *   Prescaler 64: 16.000.000 / 64 = 250.000 Ticks/Sek
 *   250.000 / 250 Schritte = 1.000 Interrupts/Sek = alle 1 ms
 * ═══════════════════════════════════════════════════════════════════════════ */
void timer0_init(void) {
  TCCR0A = (1 << WGM01);              /* CTC-Modus aktivieren (zählt bis OCR0A, dann Reset) */
  TCCR0B = (1 << CS01) | (1 << CS00); /* Prescaler 64: 16MHz / 64 = 250.000 Ticks/Sek */
  OCR0A  = 249;                       /* Zählziel: bei 249 Interrupt auslösen → 1ms Takt */
  TIMSK0 = (1 << OCIE0A);             /* Timer0 Compare-Interrupt erlauben */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ISR: TIMER0_COMPA_vect
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Wird automatisch jede 1 ms aufgerufen.
 *   Zählt state_timer_ms um 1 herunter.
 *
 * Warum ist sie wichtig?
 *   So kann die State-Machine warten ohne zu blockieren.
 *   Man setzt state_timer_ms auf z.B. 500 → nach 500 ms ist er 0.
 *   Die Hauptschleife läuft die ganze Zeit weiter.
 * ═══════════════════════════════════════════════════════════════════════════ */
ISR(TIMER0_COMPA_vect) {
  if (state_timer_ms > 0) state_timer_ms--; /* Jede ms um 1 verringern bis 0 */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: toggle_mode
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Wechselt zwischen autonomem und manuellem Modus.
 *
 * Warum ist sie wichtig?
 *   Wenn der Spieler X drückt, schickt der ESP32 das Byte 0xFE.
 *   Diese Funktion reagiert darauf und schaltet den Modus um.
 * ═══════════════════════════════════════════════════════════════════════════ */
void toggle_mode(void) {
  if (current_mode == MODE_AUTONOMOUS) { /* War autonom → wechsle zu manuell */
    current_mode = MODE_MANUEL;
    motor_set_speed(220); /* Geschwindigkeit für manuellen Modus */
  } else {                /* War manuell → wechsle zu autonom */
    current_mode = MODE_AUTONOMOUS;
    auto_state   = AUTO_FORWARD; /* State-Machine von vorne beginnen */
    motor_set_speed(220);
    motor_stop(); /* Sofort stoppen damit der Roboter nicht unkontrolliert weiterfährt */
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: check_mode_switch
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Prüft kurz ob ein Moduswechsel-Befehl (0xFE) angekommen ist.
 *   Blockiert NICHT – kehrt sofort zurück wenn keine Daten da sind.
 *
 * Warum ist sie wichtig?
 *   Wird in jedem Schleifendurchlauf aufgerufen, damit der Spieler
 *   jederzeit den Modus wechseln kann.
 * ═══════════════════════════════════════════════════════════════════════════ */
void check_mode_switch(void) {
  if (!bt_data_available()) return; /* Kein Byte angekommen → sofort zurück */
  uint8_t b = bt_receive();         /* Byte lesen */
  if (b == 0xFE) toggle_mode();     /* 0xFE = Sonderbyte für Moduswechsel */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: remote_receive_timeout (intern)
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Wartet auf ein Byte vom ESP32, aber maximal REMOTE_TIMEOUT_MS (50 ms).
 *   Kommt nichts → setzt *timeout auf 1 (Fehler-Signal).
 *
 * Was bedeutet uint8_t* timeout?
 *   Der Stern (*) bedeutet "Zeiger" – die Funktion schreibt direkt in die
 *   Variable des Aufrufers. So kann sie zwei Informationen zurückgeben:
 *   das Byte UND ob ein Timeout aufgetreten ist.
 * ═══════════════════════════════════════════════════════════════════════════ */
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

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: process_remote
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Empfängt ein komplettes Steuerpaket vom ESP32 und führt den Fahrbefehl aus.
 *
 * Paket-Aufbau (5 Bytes):
 *   Byte 0:   0xFF         – Startbyte (zeigt Beginn eines neuen Pakets)
 *   Byte 1+2: X-Achse      – links/rechts Wert (-200 bis +200)
 *   Byte 3+4: Y-Achse      – vor/zurück Wert  (-200 bis +200)
 *
 * Was bedeutet ((uint16_t)data[0] << 8) | data[1]?
 *   Ein Wert bis 200 braucht 16 Bit (2 Bytes). Die werden getrennt gesendet.
 *   << 8 = 8 Stellen nach links schieben (obere Hälfte)
 *   |    = bitweises ODER (untere Hälfte dazufügen)
 *   So werden zwei 8-Bit Bytes zu einem 16-Bit Wert zusammengebaut.
 * ═══════════════════════════════════════════════════════════════════════════ */
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

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: autonomous_mode
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   State-Machine für autonomes Fahren. Durchläuft 4 Zustände:
 *   FORWARD → REVERSE → TURN → PAUSE → FORWARD → ...
 *
 * Was ist ATOMIC_BLOCK?
 *   state_timer_ms ist 16 Bit groß. Der ATmega liest nur 8 Bit auf einmal.
 *   Ein Interrupt könnte dazwischenkommen → falscher Wert gelesen.
 *   ATOMIC_BLOCK sperrt kurz alle Interrupts damit der Wert korrekt gelesen wird.
 *
 * Was ist static uint8_t turn_direction?
 *   static bei lokaler Variable: Wert bleibt zwischen Funktionsaufrufen erhalten.
 *   turn_direction wechselt nur wenn Hindernis erfolgreich umfahren wurde.
 * ═══════════════════════════════════════════════════════════════════════════ */
void autonomous_mode(void) {
  static uint8_t turn_direction = 0; /* 0 = links drehen, 1 = rechts drehen */

  check_mode_switch(); /* Immer prüfen ob Modus gewechselt werden soll */

  uint16_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t = state_timer_ms;
  }                  /* Timer sicher lesen */
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

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: main
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Startpunkt des Programms. Initialisiert alles und läuft dann endlos.
 *
 * Wichtig: sei() muss NACH allen init-Funktionen aufgerufen werden,
 *   damit kein Interrupt mit halbfertigem Zustand aufgerufen wird.
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void) {
  motor_init();  /* Motorpins und PWM konfigurieren */
  bt_init();     /* UART für ESP32-Kommunikation (9600 Baud) */
  sensor_init(); /* Ultraschall-Sensor-Pins einrichten */
  timer0_init(); /* 1-ms-Timer starten */

  sei();                /* Alle Interrupts global einschalten – NACH den init-Funktionen */
  _delay_ms(1000);      /* 1 Sekunde warten: ESP32 und Sensor brauchen Zeit zum Starten */
  motor_set_speed(200); /* Startgeschwindigkeit auf 200 von 255 setzen */

  while (1) {            /* Endlosschleife – läuft solange der Arduino Strom hat */
    check_mode_switch(); /* Wurde X-Taste gedrückt? */

    if (current_mode == MODE_AUTONOMOUS)
      autonomous_mode(); /* Sensor übernimmt die Steuerung */
    else
      process_remote(); /* PS5 Controller übernimmt die Steuerung */
  }
}