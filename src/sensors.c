#include "sensors.h"
#include <avr/io.h>
#include <util/delay.h>
#define TRIG_PIN PC5
#define TRIG_DDR DDRC
#define TRIG_PORT PORTC

#define ECHO_PIN PB4
#define ECHO_DDR DDRB
#define ECHO_PORT PORTB
#define ECHO_PIN_REG PINB

#define TIMEOUT_US 3000  // Max. Wartezeit 3000µs ≈ 50cm Reichweite
#define MAX_DISTANCE_CM 100
#define MIN_DISTANCE_CM 2
#define NUM_MEASUREMENTS 1

static uint8_t wait_for_echo_high(uint16_t timeout_us) {
  while (!(ECHO_PIN_REG & (1 << ECHO_PIN))) {
    if (timeout_us == 0) return 0;  // Zu lange gewartet → Fehler
    _delay_us(1);
    timeout_us--;
  }
  return 1;  // Echo ist HIGH gegangen
}

void sensor_init(void) {
  TRIG_DDR |= (1 << TRIG_PIN);
  TRIG_PORT &= ~(1 << TRIG_PIN);
  ECHO_DDR &= ~(1 << ECHO_PIN);
  ECHO_PORT &= ~(1 << ECHO_PIN);
  _delay_ms(50);
}

//   1. TRIG kurz auf LOW (sauberer Startzustand)
//   2. TRIG für genau 10µs auf HIGH → Sensor sendet 8 Schallimpulse
//   3. Warten bis ECHO HIGH wird
//   4. Zählen wie lange ECHO HIGH bleibt = Schall-Laufzeit
uint16_t ultrasonic_measure_us(void) {
  uint16_t duration_us = 0;

  // Schritt 1: Reset
  TRIG_PORT &= ~(1 << TRIG_PIN);
  _delay_ms(5);

  // Schritt 2: 10µs HIGH-Impuls
  TRIG_PORT |= (1 << TRIG_PIN);
  _delay_us(10);
  TRIG_PORT &= ~(1 << TRIG_PIN);

  // Schritt 3: Warten auf Echo-Start
  if (!wait_for_echo_high(TIMEOUT_US)) return 0;  // Kein Echo bekommen → Fehler

  // Schritt 4: Echo-Dauer messen
  // Solange ECHO HIGH ist, zählen wir Mikrosekunden hoch
  while (ECHO_PIN_REG & (1 << ECHO_PIN)) {
    _delay_us(1);
    duration_us++;
    if (duration_us >= TIMEOUT_US) return 0;  // Echo zu lang → Fehler
  }

  return duration_us;
}

// Laufzeit in Zentimeter umrechnen
// Schallgeschwindigkeit ≈ 343 m/s ≈ 0,0343 cm/µs
// Schall geht hin UND zurück → durch 2 teilen
// → Distanz [cm] = duration_us × 0,0343 / 2 ≈ duration_us / 58
int ultrasonic_distance_cm(void) {
  uint16_t duration_us = ultrasonic_measure_us();
  if (duration_us == 0) return -1;

  int distance_cm = duration_us / 58;

  // Werte außerhalb des sinnvollen Bereichs verwerfen
  if (distance_cm < MIN_DISTANCE_CM || distance_cm > MAX_DISTANCE_CM) return -1;

  return distance_cm;
}

// Öffentliche Mess-Funktion (wird von main.c aufgerufen)
// Macht NUM_MEASUREMENTS Messungen und gibt den Durchschnitt zurück.
// Bei keiner gültigen Messung → -1.
int read_distance(void) {
  int sum = 0, valid_count = 0;

  for (uint8_t i = 0; i < NUM_MEASUREMENTS; i++) {
    int dist = ultrasonic_distance_cm();
    if (dist > 0) {
      sum += dist;
      valid_count++;
    }
    _delay_ms(10);  // Pause zwischen Messungen damit der Sensor sich erholt
  }
  return (valid_count == 0) ? -1 : sum / valid_count;
}