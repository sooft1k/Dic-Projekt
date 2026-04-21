/*
 * sensors.c – Abstandsmessung mit HC-SR04 (Ultraschall)
 * ======================================================
 *
 * Der HC-SR04 funktioniert wie ein Echolot (Fledermaus-Prinzip):
 *   1. Wir senden einen kurzen Impuls an TRIG → Sensor schickt 8 Ultraschall-Pulse
 *   2. Sensor zieht ECHO-Pin HIGH (= Ton ist losgeschickt)
 *   3. Ton prallt vom Hindernis zurück
 *   4. Sensor zieht ECHO-Pin LOW (= Echo ist zurück)
 *   5. Wir messen wie lange ECHO HIGH war → Distanz = Laufzeit / 58 (in cm)
 *      (Schallgeschwindigkeit 343 m/s, Hin- UND Rückweg → geteilt durch 58)
 *
 * Drei Arten von Registern am AVR (für jeden Port x = B, C, D, ...):
 *   DDRx  – Data Direction: Bit=1 → Pin ist Ausgang, Bit=0 → Pin ist Eingang
 *   PORTx – Ausgabe: Pin auf HIGH oder LOW setzen (nur wenn Ausgang)
 *   PINx  – Eingang: aktuellen Zustand eines Eingangs-Pins lesen
 *
 * Pins: TRIG = PC5 (Port C, Pin 5) → Ausgang, sendet Impuls
 *       ECHO = PB4 (Port B, Pin 4) → Eingang, empfängt Echo
 */

#include "sensors.h"
#include <avr/io.h>
#include <util/delay.h>

#define TRIG_PIN PC5
#define TRIG_DDR DDRC
#define TRIG_PORT PORTC

#define ECHO_PIN PB4
#define ECHO_DDR DDRB
#define ECHO_PORT PORTB
#define ECHO_PIN_REG PINB /* PINB = Eingangsregister für Port B (nur lesen!) */

#define TIMEOUT_US 3000     /* Maximal 3000 µs warten ≈ 51 cm Reichweite */
#define MAX_DISTANCE_CM 100 /* Über 100 cm als ungültig verwerfen */
#define MIN_DISTANCE_CM 2   /* Unter 2 cm unzuverlässig → verwerfen */
#define NUM_MEASUREMENTS 1  /* Messungen pro read_distance()-Aufruf */

/*
 * wait_for_echo_high – Warten bis ECHO-Pin HIGH wird (internes Hilfsfunktion)
 *
 * static = nur innerhalb dieser Datei sichtbar.
 * uint16_t = 16-Bit-Wert (0–65535), nötig weil 3000 nicht in uint8_t (0–255) passt.
 *
 * ECHO_PIN_REG & (1 << ECHO_PIN): filtert mit bitweisem UND genau das ECHO-Bit heraus.
 * Das "!" davor kehrt die Logik um: "solange ECHO noch LOW ist → weiterwarten".
 * Rückgabe: 1 = ECHO ist HIGH (Ton losgeschickt), 0 = Timeout (Sensor reagiert nicht).
 */
static uint8_t wait_for_echo_high(uint16_t timeout_us) {
  while (!(ECHO_PIN_REG & (1 << ECHO_PIN))) {
    if (timeout_us == 0) return 0;
    _delay_us(1);
    timeout_us--;
  }
  return 1;
}

/*
 * sensor_init – Pins konfigurieren und Sensor 50 ms aufwärmen
 *
 * Bit-Operationen:
 *   |=  (1 << PIN) → Bit auf 1 setzen = Pin ist Ausgang
 *   &= ~(1 << PIN) → Bit auf 0 setzen = Pin ist Eingang
 * Das "~" invertiert alle Bits, "&=" löscht damit genau das eine Bit.
 *
 * Der Pull-up des ECHO-Pins wird deaktiviert: der Sensor liefert sein eigenes
 * Signal, ein interner Pull-up-Widerstand würde das Signal verfälschen.
 */
void sensor_init(void) {
  TRIG_DDR |= (1 << TRIG_PIN);   /* TRIG → Ausgang */
  TRIG_PORT &= ~(1 << TRIG_PIN); /* TRIG startet LOW (kein Impuls) */
  ECHO_DDR &= ~(1 << ECHO_PIN);  /* ECHO → Eingang */
  ECHO_PORT &= ~(1 << ECHO_PIN); /* Internen Pull-up deaktivieren */
  _delay_ms(50);                 /* Sensor nach dem Einschalten stabilisieren */
}

/*
 * ultrasonic_measure_us – Echo-Laufzeit in Mikrosekunden messen
 *
 * Ablauf:
 *   TRIG LOW + 5 ms Pause  → Sensor zurücksetzen (altes Signal löschen)
 *   TRIG für 10 µs HIGH    → Messimpuls auslösen (Datenblatt-Vorgabe: min. 10 µs)
 *   TRIG wieder LOW        → Impuls beenden
 *   Warten auf ECHO HIGH   → Ton ist losgeschickt
 *   Zählen solange ECHO HIGH → jede µs wird duration_us um 1 erhöht
 *
 * Rückgabe: Laufzeit in µs, oder 0 bei Fehler/Timeout.
 */
uint16_t ultrasonic_measure_us(void) {
  uint16_t duration_us = 0;

  TRIG_PORT &= ~(1 << TRIG_PIN);
  _delay_ms(5);
  TRIG_PORT |= (1 << TRIG_PIN);
  _delay_us(10);
  TRIG_PORT &= ~(1 << TRIG_PIN);

  if (!wait_for_echo_high(TIMEOUT_US)) return 0;

  while (ECHO_PIN_REG & (1 << ECHO_PIN)) {
    _delay_us(1);
    duration_us++;
    if (duration_us >= TIMEOUT_US) return 0;
  }
  return duration_us;
}

/*
 * ultrasonic_distance_cm – Laufzeit in Zentimeter umrechnen
 *
 * Schall = 343 m/s = 0,0343 cm/µs. Hin- UND Rückweg: / 2.
 * → Distanz = duration_us × 0,0343 / 2 = duration_us / 58
 *
 * Rückgabetyp int (mit Vorzeichen), weil −1 als Fehlerwert genutzt wird.
 */
int ultrasonic_distance_cm(void) {
  uint16_t duration_us = ultrasonic_measure_us();
  if (duration_us == 0) return -1;

  int distance_cm = duration_us / 58;
  if (distance_cm < MIN_DISTANCE_CM || distance_cm > MAX_DISTANCE_CM) return -1;
  return distance_cm;
}

/*
 * read_distance – Öffentliche Schnittstelle: Abstand in cm liefern
 *
 * Macht NUM_MEASUREMENTS Messungen, filtert ungültige (−1) heraus
 * und gibt den Durchschnitt der gültigen Messungen zurück.
 * 10 ms Pause zwischen Messungen: Sensor braucht Zeit zum Zurücksetzen.
 * Rückgabe: Durchschnitt in cm, oder −1 wenn keine Messung gültig war.
 */
int read_distance(void) {
  int sum = 0, valid_count = 0;
  for (uint8_t i = 0; i < NUM_MEASUREMENTS; i++) {
    int dist = ultrasonic_distance_cm();
    if (dist > 0) {
      sum += dist;
      valid_count++;
    }
    _delay_ms(10);
  }
  return (valid_count == 0) ? -1 : sum / valid_count;
}