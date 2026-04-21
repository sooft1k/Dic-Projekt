/*
 * main.c – Hauptprogramm des 4WD-Roboters
 * =========================================
 *
 * Zwei Betriebsmodi:
 *   MODE_AUTONOMOUS – Roboter fährt selbstständig und weicht Hindernissen aus.
 *   MODE_MANUEL     – Fernsteuerung über einen Webserver im Browser (Handy/PC).
 *                     Der ESP32 empfängt Klicks auf der Webseite und schickt
 *                     sie als Bytes per UART-Kabel an diesen Arduino weiter.
 *
 * Moduswechsel: "Modus wechseln"-Button im Browser → ESP32 sendet Byte 0xFE.
 *
 * Überblick der Dateien:
 *   motors.h/c  – Motorsteuerung (4 Motoren, H-Brücken, PWM)
 *   sensors.h/c – Ultraschall-Sensor HC-SR04 (Abstandsmessung)
 *   bt.h/c      – UART-Empfang vom ESP32
 */

#define F_CPU                                                                   \
  16000000UL               /* CPU-Takt 16 MHz – muss vor den Includes stehen, \
                              weil _delay_ms() diesen Wert für seine Berechnung braucht */
#include <avr/io.h>        /* Register-Namen: TCCR, OCR, DDR, PORT, ... */
#include <avr/interrupt.h> /* ISR()-Makro, sei() = Interrupts ein, cli() = aus */
#include <util/delay.h>    /* _delay_ms() und _delay_us() */
#include <util/atomic.h>   /* ATOMIC_BLOCK – unterbrechungssicheres Lesen von Variablen */
#include <stdlib.h>
#include "motors.h"
#include "sensors.h"
#include "bt.h"

#define OBSTACLE_DISTANCE_CM 20 /* Unter 20 cm → Hindernis erkannt */
#define REMOTE_START_BYTE 0xFF  /* Jedes Steuerpaket beginnt mit diesem Byte */
#define REMOTE_TIMEOUT_MS 50    /* Kommen 50 ms lang keine Daten → Motoren stoppen */
#define REMOTE_THRESHOLD 100    /* Totzone: Steuerwerte unter ±100 = keine Bewegung */

/*
 * RobotMode – Betriebsmodus des Roboters
 *
 * typedef enum: eigener Datentyp mit genau zwei erlaubten Werten.
 * volatile: Die Variable kann jederzeit durch einen Interrupt geändert werden.
 * Ohne volatile würde der Compiler sie im Register cachen und Änderungen
 * durch die ISR nie bemerken – ein schwer findender Fehler.
 */
typedef enum { MODE_AUTONOMOUS, MODE_MANUEL } RobotMode;

/*
 * AutoState – Zustände der autonomen Fahrlogik (Zustandsmaschine)
 *
 * Eine Zustandsmaschine: Der Roboter ist immer in GENAU EINEM Zustand.
 * Jeder Zustand läuft einmal, setzt einen Timer und kehrt zurück.
 * So wird die Hauptschleife nie blockiert.
 *
 *   AUTO_FORWARD – vorwärts fahren, Abstand messen
 *   AUTO_REVERSE – rückwärts (direkt nach Hinderniserkennung)
 *   AUTO_TURN    – Kurve drehen um am Hindernis vorbeizukommen
 *   AUTO_PAUSE   – kurz stoppen, erneut messen: Weg frei?
 */
typedef enum { AUTO_FORWARD, AUTO_REVERSE, AUTO_TURN, AUTO_PAUSE } AutoState;

volatile RobotMode current_mode   = MODE_AUTONOMOUS;
static AutoState   auto_state     = AUTO_FORWARD;
volatile uint16_t  state_timer_ms = 0; /* Countdown in ms, wird durch ISR heruntergezählt */

/*
 * timer0_init – Timer0 als 1-ms-Taktgeber einrichten
 *
 * Modus CTC (Clear Timer on Compare): Timer zählt hoch, erreicht OCR0A,
 * setzt sich auf 0 zurück und löst einen Interrupt aus – dann von vorne.
 *
 * Berechnung:
 *   Prescaler 64 (CS01+CS00): Timer-Takt = 16 MHz / 64 = 250.000 Ticks/s
 *   OCR0A = 249: Interrupt alle 250 Ticks → 250.000 / 250 = 1000/s = alle 1 ms
 *
 * TIMSK0 = OCIE0A: erlaubt den Compare-Match-Interrupt für Timer0.
 */
void timer0_init(void) {
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A  = 249;
  TIMSK0 = (1 << OCIE0A);
}

/*
 * ISR(TIMER0_COMPA_vect) – läuft automatisch jede 1 ms
 *
 * ISR = Interrupt Service Routine: der Prozessor unterbricht die Hauptschleife,
 * führt diese Funktion aus und macht danach genau dort weiter wo er aufgehört hat.
 * Zählt state_timer_ms um 1 herunter → präziser Timer ohne Blockierung der main()-Schleife.
 */
ISR(TIMER0_COMPA_vect) {
  if (state_timer_ms > 0) state_timer_ms--;
}

/*
 * toggle_mode – Zwischen autonomem und manuellem Modus wechseln
 *
 * Beim Wechsel zu MANUEL: Geschwindigkeit auf 220 (etwas unter Vollgas = bessere Kontrolle).
 * Beim Wechsel zurück zu AUTONOMOUS: Motoren sofort stoppen.
 * Ohne motor_stop() würde der Roboter kurz unkontrolliert mit dem letzten Webbefehl weiterfahren.
 */
void toggle_mode(void) {
  if (current_mode == MODE_AUTONOMOUS) {
    current_mode = MODE_MANUEL;
    motor_set_speed(220);
  } else {
    current_mode = MODE_AUTONOMOUS;
    auto_state   = AUTO_FORWARD;
    motor_set_speed(220);
    motor_stop();
  }
}

/*
 * check_mode_switch – Prüft ob ein Moduswechsel-Byte angekommen ist
 *
 * Blockiert NICHT – kehrt sofort zurück wenn nichts da ist.
 * Wird regelmäßig aufgerufen damit ein Moduswechsel jederzeit möglich ist.
 */
void check_mode_switch(void) {
  if (!bt_data_available()) return;
  uint8_t b = bt_receive();
  if (b == 0xFE) toggle_mode();
}

/*
 * remote_receive_timeout – Byte empfangen mit Zeitlimit
 *
 * Wartet maximal REMOTE_TIMEOUT_MS Millisekunden auf ein Byte.
 * Bei Timeout wird das Timeout-Flag im Aufrufer gesetzt.
 *
 * uint8_t* timeout: Ein Zeiger (= Adresse einer Variable im Aufrufer).
 * Mit "*timeout = 1" schreibt die Funktion direkt in diese Variable –
 * nötig weil der Rückgabewert (return) schon für das Byte verwendet wird.
 *
 * Zeiger kurz erklärt:
 *   int x = 0;       x = normale Variable
 *   int* p = &x;     p = Zeiger auf x  (&x = Adresse von x)
 *   *p = 5;          schreibt 5 in x   (* = "gehe zur Adresse und schreibe dort")
 */
static uint8_t remote_receive_timeout(uint8_t* timeout) {
  uint16_t ms = 0;
  while (!bt_data_available()) {
    _delay_ms(1);
    if (++ms >= REMOTE_TIMEOUT_MS) {
      *timeout = 1;
      return 0;
    }
  }
  return bt_receive();
}

/*
 * process_remote – Webserver-Steuerpaket empfangen und in Fahrbefehle umwandeln
 *
 * Protokoll (5 Bytes):
 *   Byte 0:   0xFF – Startbyte (markiert Beginn des Pakets)
 *   Byte 1+2: X-Achse als int16_t (−200 bis +200): Links/Rechts-Steuerung
 *   Byte 3+4: Y-Achse als int16_t (−200 bis +200): Vor/Zurück-Steuerung
 *
 * Warum 2 Bytes pro Wert?
 *   1 Byte = 0 bis 255. Joystick-Werte sind −200 bis +200 (auch negativ!).
 *   int16_t = 16-Bit-Wert mit Vorzeichen, aufgeteilt auf 2 Bytes.
 *
 * Bytes zusammensetzen:
 *   (uint16_t)data[0] << 8  → Byte ins obere Hälfte schieben (8 Stellen nach links)
 *   | data[1]               → Byte in die untere Hälfte einfügen (bitweises ODER)
 *   (int16_t)(...)          → Ergebnis als vorzeichenbehafteten Wert interpretieren
 */
void process_remote(void) {
  uint8_t  timeout = 0;
  uint16_t sync    = 0;

  uint8_t first = remote_receive_timeout(&timeout);
  if (timeout) {
    motor_stop();
    return;
  } /* Keine Daten → Notbremse */
  if (first == 0xFE) {
    toggle_mode();
    return;
  } /* Moduswechsel */
  if (first != REMOTE_START_BYTE) {
    if (++sync > 3) {
      motor_stop();
      return;
    } /* Zu viele Fehlbytes → aufgeben */
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

  int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]); /* Links/Rechts */
  int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]); /* Vor/Zurück   */

  if (y > REMOTE_THRESHOLD) { /* Vorwärts-Befehl */
    if (x > REMOTE_THRESHOLD)
      motor_curve_right();
    else if (x < -REMOTE_THRESHOLD)
      motor_curve_left();
    else
      motor_forward();
  } else if (y < -REMOTE_THRESHOLD) { /* Rückwärts-Befehl */
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
    motor_stop(); /* Alles in Totzone → stopp */
}

/*
 * autonomous_mode – Selbstständiges Fahren mit Hindernis-Ausweichen
 *
 * Wird in jedem Hauptschleifendurchlauf aufgerufen.
 * Prüft zuerst ob der aktuelle Zustand noch aktiv ist (Timer > 0 → warten).
 * Danach führt er je nach auto_state eine Aktion aus, setzt einen neuen Timer
 * und kehrt sofort zurück – kein blockierendes Warten!
 *
 * ATOMIC_BLOCK(ATOMIC_RESTORESTATE):
 *   state_timer_ms ist 16 Bit, der ATmega liest aber nur 8 Bit auf einmal.
 *   Genau zwischen den zwei Leseschritten könnte der Timer-Interrupt feuern
 *   und den Wert verändern → man liest halb alt, halb neu = falscher Wert.
 *   ATOMIC_BLOCK sperrt kurz alle Interrupts (cli()), liest vollständig,
 *   stellt danach den vorherigen Zustand wieder her (RESTORESTATE).
 *
 * static uint8_t turn_direction:
 *   "static" bei einer lokalen Variable = Wert bleibt zwischen Aufrufen erhalten.
 *   Normale lokale Variablen verschwinden nach dem Funktionsaufruf.
 *   So merkt sich die Funktion dauerhaft die letzte Drehrichtung.
 */
void autonomous_mode(void) {
  static uint8_t turn_direction = 0; /* 0 = links, 1 = rechts */

  check_mode_switch();

  uint16_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t = state_timer_ms;
  }
  if (t > 0) return; /* Zustand noch aktiv → nichts tun */

  switch (auto_state) {
    case AUTO_FORWARD: {
      int d = read_distance();
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) {
        motor_stop();
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 800;
        }
      } else {
        motor_forward();
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
        /* Noch immer Hindernis → gleiche Richtung weitermachen */
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 300;
        }
      } else {
        /* Weg frei → Drehrichtung für das nächste Hindernis wechseln */
        turn_direction = !turn_direction; /* 0→1 oder 1→0  (! = logische Negation) */
        auto_state     = AUTO_FORWARD;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 500;
        }
      }
      break;
    }
  }
}

/*
 * main – Einstiegspunkt des Programms (läuft niemals durch, kehrt nie zurück)
 *
 * Reihenfolge: ERST alle init()-Funktionen, DANN sei().
 * sei() = Set Enable Interrupt: schaltet alle Interrupts global frei.
 * Würde ein Interrupt vor der Initialisierung feuern, könnte die ISR
 * auf halbfertige Register zugreifen → undefiniertes Verhalten.
 *
 * _delay_ms(1000): 1 Sekunde warten damit ESP32 und Sensor nach dem
 * Einschalten vollständig hochgefahren sind bevor der Roboter losfährt.
 */
int main(void) {
  motor_init();
  bt_init();
  sensor_init();
  timer0_init();

  sei();
  _delay_ms(1000);
  motor_set_speed(200);

  while (1) {
    check_mode_switch();
    if (current_mode == MODE_AUTONOMOUS)
      autonomous_mode();
    else
      process_remote();
  }
}