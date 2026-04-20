#include "sensors.h"
#include <avr/io.h>     /* Alle AVR-Register-Definitionen (DDRx, PORTx, PINx, ...) */
#include <util/delay.h> /* Stellt _delay_ms() und _delay_us() zur Verfügung */

/* ── Was sind DDR, PORT und PIN? ─────────────────────────────────────────

*  DDRx  = "Data Direction Register"
 *    Legt fest, ob ein Pin Ausgang oder Eingang ist.
 *    Bit = 1 → Pin ist Ausgang (Output)
 *    Bit = 0 → Pin ist Eingang (Input)
 *    Beispiel: DDRC |= (1 << PC5) macht Pin PC5 zum Ausgang.
 *
 *  PORTx = "Port Output/Pull-up Register"
 *    Wenn der Pin als Ausgang konfiguriert ist:
 *      Bit = 1 → Pin geht auf HIGH 
 *      Bit = 0 → Pin geht auf LOW 
 *    Wenn der Pin als Eingang konfiguriert ist:
 *      Bit = 1 → Internen Pull-up-Widerstand aktivieren
 *      Bit = 0 → Kein Pull-up (Pin "schwimmt frei")
 *
 *  PINx  = "Port Input Register"
 *    Dieses Register kann NUR gelesen werden (nicht geschrieben).
 *    Es zeigt den aktuellen elektrischen Zustand der Pins an.
 *    Bit = 1 → Aktuell HIGH (z. B. weil der Sensor HIGH anlegt)
 *    Bit = 0 → Aktuell LOW
 *    Beispiel: PINB & (1 << PB4) liest ob PB4 gerade HIGH ist.
 * ──────────────────────────────────────────────────────────────────────── */

/* ── Pin-Definitionen ─────────────────────────────────────────────────────
 *
 *  TRIG-Pin (PC5): Mikrocontroller → Sensor ("jetzt messen!")
 *    Ein 10 µs langer HIGH-Impuls startet eine Messung.
 *
 *  ECHO-Pin (PB4): Sensor → Mikrocontroller (Echo-Rückmeldung)
 *    Geht HIGH wenn der Schall losgeschickt wurde, und wieder LOW
 *    wenn das Echo empfangen wird. Die Dauer = Schall-Laufzeit.
 *
 *  PINB = das Eingangs-Leseregister für Port B (erklärt oben).
 *  Es wird mit "#define ECHO_PIN_REG PINB" umbenannt, damit im Code
 *  klarer ist, wofür es verwendet wird.
 * ──────────────────────────────────────────────────────────────────────── */
#define TRIG_PIN PC5
#define TRIG_DDR DDRC
#define TRIG_PORT PORTC

#define ECHO_PIN PB4
#define ECHO_DDR DDRB
#define ECHO_PORT PORTB
#define ECHO_PIN_REG PINB

/* ── Was sind _delay_ms() und _delay_us()? ───────────────────────────────
 *
 *  Diese Funktionen aus <util/delay.h> lassen die CPU eine bestimmte
 *  Zeit lang "nichts tun" – man nennt das "Busy-Wait" oder "Spin-Wait".
 *  Statt Interrupts oder Timer zu verwenden, zählt die CPU intern
 *  einfach Taktzyklen, bis die gewünschte Zeit vergangen ist.
 *
 *  _delay_ms(n) = warte n Millisekunden  (1 ms = 0,001 Sekunden)
 *  _delay_us(n) = warte n Mikrosekunden  (1 µs = 0,000001 Sekunden)
 * ──────────────────────────────────────────────────────────────────────── */

/* ── Messgrenzen ──────────────────────────────────────────────────────────
 *
 *  TIMEOUT_US (3000 µs):
 *    Kommt kein Echo innerhalb von 3 ms zurück, war entweder kein
 *    Hindernis in Reichweite, oder der Sensor hat versagt. In beiden
 *    Fällen wird die Messung als ungültig gewertet und abgebrochen.
 *
 *  MAX_DISTANCE_CM (100 cm):
 *    Messwerte über 100 cm werden ignoriert. Der Roboter muss nicht
 *    weiter als 1 Meter vorausschauen. Außerdem wird die Messung bei
 *    größeren Distanzen ungenauer.
 *
 *  MIN_DISTANCE_CM (2 cm):
 *    Der HC-SR04 kann physikalisch nicht zuverlässig unter 2 cm messen
 *    (der Sensor braucht Zeit, um vom Senden auf Empfangen umzuschalten).
 *    Werte darunter werden als Messfehler verworfen.
 *
 *  NUM_MEASUREMENTS (1):
 *    Wie viele Einzelmessungen gemacht werden, um einen stabilen Wert
 *    zu erhalten. Aktuell nur 1 (schnell, aber leicht störanfällig).
 *    Bei Problemen mit Fehlmessungen: auf 3 erhöhen für Mittelwert.
 * ──────────────────────────────────────────────────────────────────────── */
#define TIMEOUT_US 3000
#define MAX_DISTANCE_CM 100
#define MIN_DISTANCE_CM 2
#define NUM_MEASUREMENTS 1

/* ── wait_for_echo_high: Warte bis ECHO-Pin auf HIGH geht ────────────────
 *
 *  Was bedeutet "static"?
 *    Das "static" vor einer Funktion bedeutet, dass diese
 *    Funktion nur innerhalb dieser Datei (sensors.c) sichtbar ist.
 *    Andere Dateien (main.c, motors.c) können sie nicht aufrufen.
 *    Das ist eine bewusste Kapselung: Hilfsfunktionen nach außen verstecken.
 *
 *  Was macht die Funktion?
 *    Nach dem TRIG-Impuls schickt der HC-SR04 intern 8 Ultraschall-Pulse
 *    los, bevor er den ECHO-Pin auf HIGH setzt. Das dauert ca. 400–500 µs.
 *    In dieser Zeit muss man warten. Diese Funktion
 *    prüft in einer Schleife den ECHO-Pin und bricht nach timeout_us
 *    Mikrosekunden ab, falls er nie HIGH wird.
 *
 *  uint16_t = "unsigned integer, 16 Bit"
 *    Ganzzahl ohne Vorzeichen, Wertebereich 0 bis 65.535.
 *    Nötig weil TIMEOUT_US = 3000 größer als 255 (Maximum von uint8_t) ist.
 *
 *  Rückgabe: 1 = Echo-Pin ist HIGH (Messung läuft), 0 = Timeout (Fehler).
 * ──────────────────────────────────────────────────────────────────────── */
static uint8_t wait_for_echo_high(uint16_t timeout_us) {
  while (!(ECHO_PIN_REG & (1 << ECHO_PIN))) {
    if (timeout_us == 0)
      return 0;
    _delay_us(1);
    timeout_us--;
  }
  return 1;
}

/* ── sensor_init: Sensor beim Programmstart vorbereiten ──────────────────
 *
 *  Konfiguriert die Pins und wartet auf die Stabilisierung des Sensors.
 *
 *  TRIG_DDR |= (1 << TRIG_PIN):
 *    Setzt das TRIG-Bit im DDR-Register auf 1 → TRIG-Pin wird Ausgang.
 *    "|=" bedeutet: behalte alle anderen Bits wie sie sind, setze nur dieses.
 *
 *  TRIG_PORT &= ~(1 << TRIG_PIN):
 *    Setzt den TRIG-Pin auf LOW (Ruhezustand).
 *    "&=" mit dem invertierten Bit (~) setzt gezielt ein Bit auf 0.
 *
 *  ECHO_DDR &= ~(1 << ECHO_PIN):
 *    Setzt das ECHO-Bit auf 0 → ECHO-Pin wird Eingang.
 *
 *  ECHO_PORT &= ~(1 << ECHO_PIN):
 *    Deaktiviert den internen Pull-up-Widerstand am ECHO-Pin.
 *    Der HC-SR04 treibt den Pin selbst – ein Pull-up würde stören.
 *
 *  _delay_ms(50):
 *    50 ms warten. Der HC-SR04 braucht nach dem Einschalten eine kurze
 *    Aufwärmzeit, bevor die erste Messung zuverlässig ist.
 * ──────────────────────────────────────────────────────────────────────── */
void sensor_init(void) {
  TRIG_DDR |= (1 << TRIG_PIN);   /* TRIG → Ausgang */
  TRIG_PORT &= ~(1 << TRIG_PIN); /* TRIG auf LOW (Ruhezustand) */
  ECHO_DDR &= ~(1 << ECHO_PIN);  /* ECHO → Eingang */
  ECHO_PORT &= ~(1 << ECHO_PIN); /* Pull-up deaktivieren */
  _delay_ms(50);
}

/* ── ultrasonic_measure_us: Rohe Echo-Zeit in Mikrosekunden messen ───────
 *
 *  Führt genau das durch, was das HC-SR04 Datenblatt vorschreibt:
 *
 *  Schritt 1 – Sauberer Ausgangszustand:
 *    TRIG auf LOW setzen und 5 ms warten. Sichert, dass der Sensor
 *    nicht noch mit einer vorherigen Messung beschäftigt ist.
 *
 *  Schritt 2 – Trigger-Impuls:
 *    TRIG für genau 10 µs auf HIGH setzen. Das ist das Signal an den
 *    HC-SR04: "Jetzt messen!" Der Sensor schickt daraufhin 8 Ultraschall-
 *    Pulse bei 40 kHz los und setzt dann ECHO auf HIGH.
 *
 *  Schritt 3 – Auf Echo warten:
 *    wait_for_echo_high() wartet, bis ECHO HIGH wird. Passiert es nicht
 *    innerhalb von TIMEOUT_US, gilt die Messung als gescheitert.
 *
 *  Schritt 4 – Echo messen:
 *    Die while-Schleife zählt jeden Mikrosekunde, die ECHO HIGH bleibt.
 *    Sobald der Schall reflektiert zurückkommt und der Sensor ECHO wieder
 *    auf LOW zieht, endet die Schleife. duration_us ist jetzt die
 *    Gesamtlaufzeit des Schalls in beide Richtungen.
 *
 *  duration_us++ nach jedem _delay_us(1) ist der Zähler.
 *  "++" bedeutet: Variable um 1 erhöhen (Kurzform für duration_us = duration_us + 1).
 * ──────────────────────────────────────────────────────────────────────── */
uint16_t ultrasonic_measure_us(void) {
  uint16_t duration_us = 0;

  TRIG_PORT &= ~(1 << TRIG_PIN); /* Schritt 1: TRIG LOW */
  _delay_ms(5);
  TRIG_PORT |= (1 << TRIG_PIN);  /* Schritt 2: TRIG HIGH → Sensor triggern */
  _delay_us(10);                 /* Genau 10 µs HIGH halten */
  TRIG_PORT &= ~(1 << TRIG_PIN); /* TRIG wieder LOW */

  if (!wait_for_echo_high(TIMEOUT_US)) /* Schritt 3: auf Echo warten */
    return 0;

  while (ECHO_PIN_REG & (1 << ECHO_PIN)) { /* Schritt 4: Echo-Dauer messen */
    _delay_us(1);
    duration_us++;
    if (duration_us >= TIMEOUT_US)
      return 0;
  }

  return duration_us;
}

/* ── ultrasonic_distance_cm: Echo-Zeit in Zentimeter umrechnen ───────────
 *
 *  Schall bewegt sich bei ~20°C mit 343 m/s = 34.300 cm/s.
 *  In 1 Mikrosekunde legt er also 0,0343 cm zurück.
 *
 *  Weil das Echo den Weg hin UND zurück macht, teilen wir durch 2:
 *    Entfernung = (duration_us × 0,0343) / 2
 *               = duration_us / 58,3
 *               ≈ duration_us / 58
 *
 *  Die Division durch 58 ist eine ausreichend genaue Integer-Näherung.
 *  Bei exakter Messung: Formel mit Temperaturkorrektur nötig.
 *
 *  "int" (ohne uint8_t oder uint16_t):
 *    Ein vorzeichenbehafteter Integer in der Standardgröße des Systems
 *    (auf AVR = 16 Bit, Bereich -32768 bis +32767). Wir brauchen hier
 *    einen vorzeichenbehafteten Typ, weil wir -1 als Fehlerwert zurückgeben.
 * ──────────────────────────────────────────────────────────────────────── */
int ultrasonic_distance_cm(void) {
  uint16_t duration_us = ultrasonic_measure_us();
  if (duration_us == 0)
    return -1;

  int distance_cm = duration_us / 58; /* µs → cm */

  if (distance_cm < MIN_DISTANCE_CM || distance_cm > MAX_DISTANCE_CM)
    return -1;

  return distance_cm;
}

/* ── read_distance: Einziger öffentlicher Zugangspunkt für Messungen ──────
 *
 *  "Öffentlich" bedeutet: Diese Funktion ist NICHT "static", sie kann
 *  also aus anderen Dateien (z. B. main.c) aufgerufen werden.
 *
 *  Die Funktion macht NUM_MEASUREMENTS Einzelmessungen, filtert
 *  ungültige Ergebnisse (Rückgabe -1) heraus und gibt den Durchschnitt
 *  der gültigen Messungen zurück.
 *
 *  Warum 10 ms Pause zwischen Messungen?
 *    Der HC-SR04 braucht ca. 60 ms von einer Messung zur nächsten.
 *    10 ms reicht für NUM_MEASUREMENTS=1 – bei mehr Messungen sollte
 *    man die Pause auf mindestens 20 ms erhöhen.
 *
 *  Rückgabe: Entfernung in cm, oder -1 wenn kein gültiger Messwert.
 * ──────────────────────────────────────────────────────────────────────── */
int read_distance(void) {
  int valid_count = 0;
  int sum         = 0;

  for (uint8_t i = 0; i < NUM_MEASUREMENTS; i++) {
    int dist = ultrasonic_distance_cm();
    if (dist > 0) {
      sum += dist;
      valid_count++;
    }
    _delay_ms(10);
  }

  if (valid_count == 0)
    return -1;

  return sum / valid_count;
}