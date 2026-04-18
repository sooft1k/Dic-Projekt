#include "sensors.h"
#include <avr/io.h>
#include <util/delay.h>

#define TRIG_PIN PC5     // Trigger-Pin: sendet den Ultraschall-Impuls aus
#define TRIG_DDR DDRC    // Port C Direction Register
#define TRIG_PORT PORTC  // Port C Ausgabe

#define ECHO_PIN PB4  // Echo-Pin: empfängt das zurückgeworfene Signal
#define ECHO_DDR DDRB
#define ECHO_PORT PORTB
#define ECHO_PIN_REG PINB  // Port B Eingabe (zum Lesen des Echo-Pins)

#define TIMEOUT_US 3000      // Max. Wartezeit 3000µs → ca. 50cm maximale Messdistanz
#define MAX_DISTANCE_CM 100  // Über 100cm wird ignoriert — zu weit
#define MIN_DISTANCE_CM 2    // Unter 2cm wird ignoriert — zu nah (Messrauschen)
#define NUM_MEASUREMENTS 1   // Nur 1 Messung pro Aufruf (schneller)

/* Wartet bis der Echo-Pin HIGH wird (Signal kommt zurück)
   Gibt 0 zurück wenn Timeout überschritten → kein Echo empfangen */
static uint8_t wait_for_echo_high(uint16_t timeout_us) {
  while (!(ECHO_PIN_REG & (1 << ECHO_PIN))) {  // Solange Echo-Pin LOW ist warten
    if (timeout_us == 0)
      return 0;  // Zu lange gewartet → aufgeben
    _delay_us(1);
    timeout_us--;
  }
  return 1;  // Echo-Pin ist HIGH → Signal kommt zurück
}

void sensor_init(void) {
  TRIG_DDR |= (1 << TRIG_PIN);    // TRIG als Ausgang setzen (wir senden)
  TRIG_PORT &= ~(1 << TRIG_PIN);  // TRIG auf LOW starten
  ECHO_DDR &= ~(1 << ECHO_PIN);   // ECHO als Eingang setzen (wir empfangen)
  ECHO_PORT &= ~(1 << ECHO_PIN);  // Kein Pull-Up auf ECHO
  _delay_ms(50);                  // Sensor 50ms Zeit zum Starten geben
}

/* Misst wie lange das Echo-Signal braucht um zurückzukommen (in Mikrosekunden) */
uint16_t ultrasonic_measure_us(void) {
  uint16_t duration_us = 0;

  TRIG_PORT &= ~(1 << TRIG_PIN);  // TRIG auf LOW — Sensor zurücksetzen
  _delay_ms(5);                   // 5ms warten damit Sensor bereit ist

  TRIG_PORT |= (1 << TRIG_PIN);   // TRIG kurz auf HIGH → Ultraschall-Impuls senden
  _delay_us(10);                  // 10µs warten (Mindestimpulslänge laut Datenblatt)
  TRIG_PORT &= ~(1 << TRIG_PIN);  // TRIG wieder auf LOW

  if (!wait_for_echo_high(TIMEOUT_US))
    return 0;  // Kein Echo → 0 zurückgeben

  while (ECHO_PIN_REG & (1 << ECHO_PIN)) {  // Solange Echo HIGH ist zählen
    _delay_us(1);
    duration_us++;
    if (duration_us >= TIMEOUT_US)
      return 0;  // Zu lange → Timeout
  }

  return duration_us;  // Gemessene Zeit in Mikrosekunden zurückgeben
}

/* Rechnet die gemessene Zeit in Zentimeter um */
int ultrasonic_distance_cm(void) {
  uint16_t duration_us = ultrasonic_measure_us();
  if (duration_us == 0)
    return -1;  // Keine gültige Messung

  int distance_cm = duration_us / 58;  // Schall braucht ~58µs pro cm (hin und zurück)

  if (distance_cm < MIN_DISTANCE_CM || distance_cm > MAX_DISTANCE_CM)
    return -1;  // Außerhalb des gültigen Bereichs → ungültig

  return distance_cm;
}

/* Macht mehrere Messungen und gibt den Durchschnitt zurück */
int read_distance(void) {
  int valid_count = 0;
  int sum         = 0;

  for (uint8_t i = 0; i < NUM_MEASUREMENTS; i++) {
    int dist = ultrasonic_distance_cm();
    if (dist > 0) {  // Nur gültige Messungen zählen
      sum += dist;
      valid_count++;
    }
    _delay_ms(10);  // 10ms Pause zwischen Messungen — Sensor braucht Zeit
  }

  if (valid_count == 0)
    return -1;               // Keine gültige Messung → Fehler
  return sum / valid_count;  // Durchschnitt zurückgeben
}