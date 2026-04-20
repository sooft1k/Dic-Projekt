/*
 * ============================================================
 *  main.c  –  Hauptprogramm des 4WD-Roboters
 * ============================================================
 *
 *  Der Roboter kennt zwei Betriebsmodi:
 *
 *    MODE_AUTONOMOUS – Der Roboter fährt selbstständig.
 *      Er fährt vorwärts, bis er ein Hindernis erkennt,
 *      weicht dann aus und sucht einen freien Weg.
 *
 *    MODE_MANUEL – Fernsteuerung per PS4/PS5-Controller.
 *      Der ESP32 verbindet sich per Bluetooth mit dem Controller,
 *      liest die Analog-Stick-Werte aus und schickt sie als
 *      Joystick-Koordinaten (X/Y) per UART an den ATmega.
 *      Der ATmega übersetzt diese dann in Fahrbewegungen.
 *
 *  Zwischen den Modi wechselt man durch die Taste X am
 *  PS4/PS5-Controller – der ESP32 sendet dafür das Sonder-Byte 0xFE.
 * ──────────────────────────────────────────────────────────────────────── */

#define F_CPU 16000000UL
#include <avr/io.h>        /* Alle AVR-Register (TCCR, OCR, TIMSK, ...) */
#include <avr/interrupt.h> /* ISR()-Makro und sei()/cli()-Funktionen */
#include <util/delay.h>    /* _delay_ms() und _delay_us() */
#include <util/atomic.h>   /* ATOMIC_BLOCK – Interrupt-sicheres Lesen/Schreiben */
#include <stdlib.h>        /* Standardbibliothek (hier für Typ-Definitionen) */
#include "motors.h"        /* Motorsteuerung */
#include "sensors.h"       /* Ultraschall-Abstandsmessung */
#include "bt.h"            /* ESP32/Bluetooth-Kommunikation */

/* ── Schwellwerte und Protokoll-Konstanten ───────────────────────────────
 *
 *  OBSTACLE_DISTANCE_CM (20):
 *    Kommt ein Hindernis näher als 20 cm, reagiert der Roboter im
 *    autonomen Modus und weicht aus. Wert in Zentimeter.
 *
 *  REMOTE_START_BYTE (0xFF):
 *    "0x" bedeutet: diese Zahl ist im Hexadezimalsystem (Basis 16).
 *    0xFF = 255 in Dezimal = 11111111 in Binär.
 *    Jedes gültige Steuerpaket vom ESP32 beginnt mit diesem Byte.
 *    So erkennt der ATmega, dass ein neues Paket anfängt.
 *
 *  REMOTE_TIMEOUT_MS (50):
 *    Wenn kein vollständiges Paket innerhalb von 50 Millisekunden
 *    ankommt, stoppt der Roboter. Sicherheitsmaßnahme für den Fall,
 *    dass die Bluetooth-Verbindung abbricht oder der Controller ausgeht.
 *
 *  REMOTE_THRESHOLD (100):
 *    Analog-Stick-Totzone. PS4/PS5-Controller geben nie exakt 0 zurück
 *    wenn der Stick losgelassen wird – es gibt immer ein leichtes Zittern
 *    ("Drift"). Werte zwischen -100 und +100 gelten als "Mitte" und
 *    führen zu keiner Bewegung.
 * ──────────────────────────────────────────────────────────────────────── */
#define OBSTACLE_DISTANCE_CM 20
#define REMOTE_START_BYTE 0xFF
#define REMOTE_TIMEOUT_MS 50
#define REMOTE_THRESHOLD 100

/* ── Betriebsmodus-Enumeration ───────────────────────────────────────────
 *
 *  Was ist "volatile"?
 *    Das "volatile" sagt dem Compiler: "Diese Variable kann
 *    sich jederzeit von außen ändern – auch zwischen zwei Zeilen Code."
 *    Ohne "volatile" könnte der Compiler die Variable in einem Register
 *    zwischenspeichern und Änderungen durch Interrupts verpassen.
 *    Mit "volatile" liest der Compiler die Variable jedes Mal frisch
 *    aus dem RAM – nie aus einem optimierten Zwischenspeicher.
 *    current_mode ist volatile, weil ein Interrupt (oder die ISR) sie
 *    theoretisch verändern könnte.
 * ──────────────────────────────────────────────────────────────────────── */
typedef enum { MODE_AUTONOMOUS, MODE_MANUEL } RobotMode;

/* ── Zustände im autonomen Modus ─────────────────────────────────────────
 *
 *  Was ist eine State Machine (Zustandsmaschine)?
 *    Das Programm befindet sich immer in genau einem von mehreren
 *    definierten Zuständen. Jeder Zustand macht etwas Bestimmtes und
 *    entscheidet, wann in welchen anderen Zustand gewechselt wird.
 *
 *    AUTO_FORWARD → fährt vorwärts, misst Distanz
 *    AUTO_REVERSE → fährt rückwärts (nach Hinderniserkennung)
 *    AUTO_TURN    → dreht in eine Richtung
 *    AUTO_PAUSE   → stoppt kurz und misst nochmal
 * ──────────────────────────────────────────────────────────────────────── */
typedef enum { AUTO_FORWARD, AUTO_REVERSE, AUTO_TURN, AUTO_PAUSE } AutoState;

volatile RobotMode current_mode   = MODE_AUTONOMOUS; /* Startet im Autonommodus */
static AutoState   auto_state     = AUTO_FORWARD;    /* Beginnt mit Vorwärtsfahrt */
volatile uint16_t  state_timer_ms = 0;               /* Countdown-Timer (Millisekunden) */

/* ── timer0_init: Millisekunden-Timer konfigurieren ──────────────────────
 *
 *  Timer 0 wird so eingestellt, dass er genau alle 1 ms einen Interrupt
 *  auslöst. Das gibt uns eine präzise Zeitbasis für die Zustandsmaschine.
 *
 *  Was ist CTC-Modus?
 *    CTC = "Clear Timer on Compare Match"
 *    Im CTC-Modus zählt der Timer von 0 bis zum Wert in OCR0A.
 *    Erreicht er OCR0A, setzt er sich auf 0 zurück ("Clear") und
 *    löst einen Interrupt aus. Dann beginnt er wieder von vorne.
 *
 *  TCCR0A = (1 << WGM01)
 *    TCCR0A = "Timer/Counter 0 Control Register A"
 *    WGM01 = "Waveform Generation Mode, Bit 1"
 *    Nur WGM01=1 (WGM00=0, WGM02=0) aktiviert den CTC-Modus für Timer 0.
 *
 *  TCCR0B = (1 << CS01) | (1 << CS00)
 *    TCCR0B = "Timer/Counter 0 Control Register B"
 *    CS01 und CS00 = "Clock Select Bits 1 und 0"
 *    CS01=1 und CS00=1 zusammen = Prescaler 64:
 *      Timer-Takt = 16.000.000 Hz / 64 = 250.000 Hz
 *      Der Timer zählt 250.000 mal pro Sekunde.
 *
 *  OCR0A = 249
 *    OCR0A = "Output Compare Register 0 A"
 *    Der Timer zählt von 0 bis 249 – das sind 250 Schritte.
 *    Bei 250.000 Schritten pro Sekunde und 250 Schritten pro Zyklus:
 *    250.000 / 250 = 1000 Zyklen pro Sekunde = 1 Interrupt pro ms ✓
 *
 *  TIMSK0 = (1 << OCIE0A)
 *    TIMSK0 = "Timer Interrupt Mask Register 0"
 *    Dieses Register steuert, welche Timer-Interrupts aktiviert sind.
 *    OCIE0A = "Output Compare Interrupt Enable for Timer 0, Channel A"
 *    Mit diesem Bit auf 1 wird der Interrupt freigegeben, der jedes Mal
 *    ausgelöst wird, wenn Timer 0 den Wert in OCR0A erreicht.
 *    Damit Interrupts überhaupt funktionieren, muss zusätzlich
 *    sei() aufgerufen werden (globale Interrupt-Freischaltung).
 * ──────────────────────────────────────────────────────────────────────── */
void timer0_init(void) {
  TCCR0A = (1 << WGM01);              /* CTC-Modus aktivieren */
  TCCR0B = (1 << CS01) | (1 << CS00); /* Prescaler 64 */
  OCR0A  = 249;                       /* Zählziel für 1 ms Takt */
  TIMSK0 = (1 << OCIE0A);             /* Interrupt bei Compare Match freischalten */
}

/* ── ISR: Interrupt Service Routine (wird jede Millisekunde aufgerufen) ───
 *
 *  Was ist eine ISR?
 *    Eine ISR (Interrupt Service Routine) ist eine spezielle Funktion,
 *    die der Mikrocontroller automatisch aufruft, wenn ein bestimmtes
 *    Ereignis eintritt – hier: wenn Timer 0 den Wert OCR0A erreicht.
 *    Die CPU unterbricht dabei das Hauptprogramm, führt die ISR aus,
 *    und macht dann genau dort weiter, wo sie unterbrochen wurde.
 *    Das passiert unsichtbar im Hintergrund, ohne dass main() etwas tun muss.
 *
 *  Was bedeutet ISR(TIMER0_COMPA_vect)?
 *    ISR() ist ein Makro aus <avr/interrupt.h>. Es registriert die Funktion
 *    als Interrupt-Handler für den angegebenen Vektor.
 *    TIMER0_COMPA_vect = "Timer 0 Compare Match A Vektor"
 *    "Vektor" = eine feste Speicheradresse, an der der ATmega nachschaut,
 *    welche Funktion er aufrufen soll, wenn dieser Interrupt auftritt.
 *    Jeder mögliche Interrupt hat seinen eigenen Vektor.
 *
 *  Was macht die ISR hier?
 *    Sie zählt state_timer_ms um 1 herunter. Das passiert genau 1000 mal
 *    pro Sekunde. Setzt man state_timer_ms auf 800, ist es nach 800 ms = 0.
 *    Das Hauptprogramm wartet auf diesen Nullwert, bevor es weitermacht.
 * ──────────────────────────────────────────────────────────────────────── */
ISR(TIMER0_COMPA_vect) {
  if (state_timer_ms > 0)
    state_timer_ms--; /* Einmal pro ms herunterzählen */
}

/* ── toggle_mode: Zwischen autonomem und manuellem Modus wechseln ────────
 *
 *  Wird aufgerufen, wenn der ESP32 das Byte 0xFE sendet (ausgelöst durch
 *  die Taste X am PS4/PS5-Controller).
 *
 *  Wechsel zu MANUEL:
 *    Geschwindigkeit auf 220 setzen – etwas weniger als Vollgas (255),
 *    damit der Roboter mit dem Controller besser kontrollierbar ist.
 *
 *  Wechsel zurück zu AUTONOMOUS:
 *    Zustand auf AUTO_FORWARD zurücksetzen und sofort stoppen, damit
 *    der Roboter nicht unkontrolliert mit der letzten Controller-Eingabe
 *    weiterfährt.
 * ──────────────────────────────────────────────────────────────────────── */
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

/* ── check_mode_switch: Kurz auf Modus-Wechsel prüfen ────────────────────
 *
 *  Schaut ob neue Daten vom ESP32 vorliegen. Falls ja, liest genau
 *  ein Byte. Ist es 0xFE, wird der Modus gewechselt. Alles andere
 *  wird ignoriert.
 *
 *  Diese Funktion blockiert NICHT – sie prüft nur kurz und kehrt sofort
 *  zurück. Das ist wichtig im autonomen Modus, damit die Fahrt nicht
 *  unterbrochen wird während auf einen Modus-Wechsel gewartet wird.
 * ──────────────────────────────────────────────────────────────────────── */
void check_mode_switch(void) {
  if (!bt_data_available())
    return;
  uint8_t b = bt_receive();
  if (b == 0xFE)
    toggle_mode();
}

/* ── remote_receive_timeout: Byte lesen mit Zeitlimit ────────────────────
 *
 *  Versucht ein Byte vom ESP32 zu empfangen. Wartet maximal
 *  REMOTE_TIMEOUT_MS (50 ms). Kommt nichts, setzt es *timeout = 1.
 *
 *  Was bedeutet "uint8_t* timeout"?
 *    Der Stern (*) bedeutet: Dies ist ein Zeiger auf eine uint8_t-Variable.
 *    Ein Zeiger speichert nicht den Wert selbst, sondern die Speicheradresse
 *    der Variable. Mit "*timeout = 1" schreibt die Funktion direkt in die
 *    Variable des Aufrufers – so kann sie ein "Ergebnis-Flag" zurückgeben,
 *    obwohl sie bereits den uint8_t-Rückgabewert für das Byte nutzt.
 *
 *  "static" vor der Funktion: Nur in dieser Datei sichtbar (intern).
 * ──────────────────────────────────────────────────────────────────────── */
static uint8_t remote_receive_timeout(uint8_t* timeout) {
  uint16_t ms = 0;
  while (!bt_data_available()) {
    _delay_ms(1);
    if (++ms >= REMOTE_TIMEOUT_MS) { /* "++" = um 1 erhöhen vor dem Vergleich */
      *timeout = 1;
      return 0;
    }
  }
  return bt_receive();
}

/* ── process_remote: Ein vollständiges ESP32-Paket verarbeiten ────────────
 *
 *  Der ESP32 liest den Analog-Stick des PS4/PS5-Controllers aus und
 *  schickt die Werte als 5-Byte-Paket an den ATmega. Protokoll:
 *
 *    Byte 0:   0xFF – Startbyte, kennzeichnet Paketbeginn
 *    Byte 1+2: X-Achse als 16-Bit-Zahl (int16_t: -32768 bis +32767)
 *    Byte 3+4: Y-Achse als 16-Bit-Zahl (int16_t: -32768 bis +32767)
 *
 *  Was ist int16_t?
 *    Ein vorzeichenbehafteter 16-Bit-Integer. Das "i" (statt "u") steht
 *    für "signed" – er kann negative Werte speichern.
 *    Wertebereich: -32.768 bis +32.767.
 *    Nötig weil Joystick-Werte sowohl positiv (rechts/oben) als auch
 *    negativ (links/unten) sein können.
 *
 *  Was macht "((uint16_t)data[0] << 8) | data[1]"?
 *    Die X-Achse wird als zwei separate Bytes (data[0] und data[1])
 *    übertragen. Dieser Ausdruck setzt sie wieder zu einer 16-Bit-Zahl zusammen:
 *    1. data[0] wird zu uint16_t gecastet (auf 16 Bit erweitert)
 *    2. "<< 8" schiebt es um 8 Bit nach links (= in die obere Hälfte)
 *    3. "|" (bitweises ODER) fügt data[1] in die untere Hälfte ein
 *    Beispiel: data[0]=0x01, data[1]=0xF4 → 0x0001F4 = 500
 *    Dann wird das Ergebnis zu int16_t gecastet, damit es vorzeichenbehaftet ist.
 *
 *  Sonderfall 0xFE: Modus-Wechsel-Signal (statt eines Datenpakets).
 *  Bei Timeout: Notbremse – motor_stop() wird sofort aufgerufen.
 *
 *  Koordinaten-Interpretation (nach Abzug der Totzone REMOTE_THRESHOLD):
 *    Y > 0 (Stick vorwärts):  vorwärts fahren, optional mit X-Kurve
 *    Y < 0 (Stick rückwärts): rückwärts fahren, optional mit X-Kurve
 *    Y ≈ 0, X > 0:            auf der Stelle rechts drehen
 *    Y ≈ 0, X < 0:            auf der Stelle links drehen
 *    Alles in Totzone:         stoppen (Stick losgelassen)
 * ──────────────────────────────────────────────────────────────────────── */
void process_remote(void) {
  uint8_t  timeout = 0;
  uint16_t sync    = 0;

  uint8_t first = remote_receive_timeout(&timeout); /* "&" = Adresse der Variable */
  if (timeout) {
    motor_stop();
    return;
  }

  if (first == 0xFE) {
    toggle_mode();
    return;
  }

  if (first != REMOTE_START_BYTE) {
    if (++sync > 3) {
      motor_stop();
      return;
    }
    return;
  }

  uint8_t data[4]; /* Array = 4 aufeinanderfolgende uint8_t-Werte */
  for (uint8_t i = 0; i < 4; i++) {
    data[i] = remote_receive_timeout(&timeout);
    if (timeout) {
      motor_stop();
      return;
    }
  }

  /* Zwei Bytes zu einem 16-Bit-Wert zusammensetzen */
  int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]); /* X-Achse */
  int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]); /* Y-Achse */

  /* Analog-Stick-Werte in Fahrbefehle übersetzen */
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

/* ── autonomous_mode: Selbstständiges Fahren und Ausweichen ──────────────
 *
 *  Was ist ATOMIC_BLOCK(ATOMIC_RESTORESTATE)?
 *    "Atomar" bedeutet: unteilbar. Ein atomarer Vorgang kann nicht durch
 *    einen Interrupt unterbrochen werden.
 *    state_timer_ms ist 16 Bit groß. Der ATmega328P kann aber nur 8 Bit
 *    auf einmal lesen. Beim Lesen der ersten 8 Bit könnte ein Interrupt
 *    kommen und die anderen 8 Bit verändern – wir würden einen falschen
 *    Wert lesen (halb alt, halb neu).
 *    ATOMIC_BLOCK deaktiviert kurz alle Interrupts (mit cli() intern),
 *    liest die Variable vollständig, und schaltet Interrupts danach
 *    wieder ein. ATOMIC_RESTORESTATE stellt dabei den vorherigen
 *    Interrupt-Zustand wieder her (statt immer sei() zu rufen).
 *    cli() = "Clear Interrupt" (Interrupts sperren)
 *    sei() = "Set Enable Interrupt" (Interrupts freischalten)
 *
 *  Was ist "static uint8_t turn_direction"?
 *    "static" bei einer lokalen Variable (innerhalb einer Funktion) hat
 *    eine besondere Bedeutung: Die Variable wird NUR EINMAL beim ersten
 *    Aufruf initialisiert und behält ihren Wert zwischen allen folgenden
 *    Aufrufen. Normale lokale Variablen werden bei jedem Funktionsaufruf
 *    neu angelegt und verlieren ihren Wert. So "merkt sich" turn_direction,
 *    in welche Richtung zuletzt gedreht wurde – auch nach Rückkehr zu main().
 * ──────────────────────────────────────────────────────────────────────── */
void autonomous_mode(void) {
  static uint8_t turn_direction = 0; /* 0 = nächste Drehung rechts, 1 = links */

  check_mode_switch(); /* Zwischendurch auf Modus-Wechsel prüfen */

  /* state_timer_ms atomar lesen (Interrupt-sicher) */
  uint16_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t = state_timer_ms;
  }
  if (t > 0)
    return; /* Timer läuft noch – Zustand nicht wechseln */

  switch (auto_state) {
    /* Vorwärts fahren – bei Hindernis < 20 cm sofort reagieren */
    case AUTO_FORWARD: {
      int d = read_distance();
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) {
        motor_stop();
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 800; /* 800 ms rückwärts fahren */
        }
      } else {
        motor_forward();
      }
      break;
    }

    /* 800 ms rückwärts fahren (Timer wurde oben gesetzt), dann drehen */
    case AUTO_REVERSE:
      motor_backward();
      auto_state = AUTO_TURN;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 500; /* 500 ms rückwärts, dann Drehung */
      }
      break;

    /* Abwechselnd rechts und links drehen um Hindernisse zu umfahren */
    case AUTO_TURN:
      if (turn_direction == 0) {
        motor_curve_right();
        turn_direction = 1; /* Beim nächsten Mal links drehen */
      } else {
        motor_curve_left();
        turn_direction = 0; /* Beim nächsten Mal rechts drehen */
      }
      auto_state = AUTO_PAUSE;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_timer_ms = 1500; /* 1,5 Sekunden drehen */
      }
      break;

    /* Kurz stoppen und nochmal messen – Weg frei? Weiterfahren. Blockiert? Nochmal ausweichen. */
    case AUTO_PAUSE: {
      motor_stop();
      int d = read_distance();
      if (d > 0 && d < OBSTACLE_DISTANCE_CM) {
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 300;
        }
      } else {
        auto_state = AUTO_FORWARD;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          state_timer_ms = 500;
        }
      }
      break;
    }
  }
}

/* ── main: Einstiegspunkt des Programms ──────────────────────────────────
 *
 *  main() ist die erste Funktion, die nach dem Einschalten läuft.
 *  Sie kehrt niemals zurück – der Roboter läuft in while(1) endlos.
 *
 *  sei() = "Set Enable Interrupt"
 *    Schaltet alle Interrupts global frei. Ohne diesen Aufruf würde
 *    kein einziger Interrupt funktionieren – Timer, UART, alles würde
 *    stillstehen. sei() und cli() sind die globalen Ein/Aus-Schalter
 *    für das gesamte Interrupt-System.
 *
 *  Initialisierungsreihenfolge:
 *    1. motor_init()    – Richtungspins und PWM konfigurieren
 *    2. bt_init()       – UART zum ESP32 einrichten (9600 Baud)
 *    3. sensor_init()   – Ultraschall-Sensor vorbereiten
 *    4. timer0_init()   – 1-ms-Interrupt-Timer einrichten
 *    5. sei()           – Interrupts global freischalten
 *    6. _delay_ms(1000) – 1 Sekunde warten: alle Module stabilisieren
 *    7. motor_set_speed – Startgeschwindigkeit festlegen
 *
 *  Hauptschleife (while(1) = läuft für immer):
 *    Jede Iteration: Modus prüfen, dann entsprechenden Modus ausführen.
 *    Die Schleife läuft sehr schnell (Mikrosekunden pro Durchlauf),
 *    daher wird check_mode_switch() tausende Male pro Sekunde aufgerufen.
 * ──────────────────────────────────────────────────────────────────────── */
int main(void) {
  motor_init();  /* Motorpins und PWM initialisieren */
  bt_init();     /* UART-Verbindung zum ESP32 (9600 Baud) */
  sensor_init(); /* Ultraschall-Sensor vorbereiten */
  timer0_init(); /* 1-ms-Timer konfigurieren */

  sei();                /* Interrupts global freischalten – ab hier läuft die ISR */
  _delay_ms(1000);      /* 1 Sekunde warten: alle Module hochfahren lassen */
  motor_set_speed(200); /* Startgeschwindigkeit: 200 von 255 */

  while (1) {
    check_mode_switch();

    if (current_mode == MODE_AUTONOMOUS)
      autonomous_mode();
    else
      process_remote();
  }
}