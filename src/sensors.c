// sensors.c – Ultraschallsensor HC-SR04
// Misst die Entfernung zu einem Hindernis in Zentimetern.
// Prinzip: Schallimpuls senden → Echo messen → Zeit in cm umrechnen

#include "sensors.h"
#include <avr/io.h>
#include <util/delay.h>

// Pin-Definitionen für TRIG (Senden) und ECHO (Empfangen)
#define TRIG_PIN PC5
#define TRIG_DDR DDRC
#define TRIG_PORT PORTC

#define ECHO_PIN PB4
#define ECHO_DDR DDRB
#define ECHO_PORT PORTB
#define ECHO_PIN_REG PINB  // PIN-Register = lesen ob Pin HIGH oder LOW ist

#define TIMEOUT_US 3000      // Maximale Wartezeit in Mikrosekunden
#define MAX_DISTANCE_CM 100  // Alles über 100 cm wird ignoriert
#define MIN_DISTANCE_CM 2    // Alles unter 2 cm ist zu nah (Messartefakt)
#define NUM_MEASUREMENTS 1   // Anzahl Messungen pro read_distance()-Aufruf

// Wartet bis der Echo-Pin auf HIGH geht (Schall kommt zurück)
// Gibt 0 zurück wenn Timeout erreicht wurde (kein Echo)
static uint8_t wait_for_echo_high(uint16_t timeout_us) {
  while (!(ECHO_PIN_REG & (1 << ECHO_PIN))) {  // Solange ECHO noch LOW ist
    if (timeout_us == 0) return 0;             // Zu lange gewartet → Fehler
    _delay_us(1);
    timeout_us--;
  }
  return 1;  // Echo ist HIGH → Schall kommt zurück
}

// Initialisiert die Sensor-Pins
void sensor_init(void) {
  TRIG_DDR |= (1 << TRIG_PIN);    // TRIG als Ausgang (wir senden)
  TRIG_PORT &= ~(1 << TRIG_PIN);  // TRIG startet auf LOW

  ECHO_DDR &= ~(1 << ECHO_PIN);   // ECHO als Eingang (wir empfangen)
  ECHO_PORT &= ~(1 << ECHO_PIN);  // Kein interner Pull-up

  _delay_ms(50);  // Sensor braucht kurz zum Hochfahren
}

// Führt eine Ultraschallmessung durch und gibt die Laufzeit in µs zurück
// Gibt 0 zurück bei Fehler oder Timeout
uint16_t ultrasonic_measure_us(void) {
  uint16_t duration_us = 0;

  // Schritt 1: TRIG kurz LOW setzen um sauberen Startzustand zu haben
  TRIG_PORT &= ~(1 << TRIG_PIN);
  _delay_ms(5);

  // Schritt 2: 10 µs langen HIGH-Impuls senden → Sensor startet Messung
  TRIG_PORT |= (1 << TRIG_PIN);
  _delay_us(10);
  TRIG_PORT &= ~(1 << TRIG_PIN);

  // Schritt 3: Warten bis Echo HIGH wird (Schall wurde gesendet)
  if (!wait_for_echo_high(TIMEOUT_US)) return 0;  // Kein Echo → Fehler

  // Schritt 4: Zählen wie lange Echo HIGH bleibt = Laufzeit des Schalls
  while (ECHO_PIN_REG & (1 << ECHO_PIN)) {
    _delay_us(1);
    duration_us++;
    if (duration_us >= TIMEOUT_US) return 0;  // Zu lange → Fehler
  }

  return duration_us;  // Laufzeit in Mikrosekunden zurückgeben
}

// Rechnet die Laufzeit in Zentimeter um
// Gibt -1 zurück bei ungültiger Messung
int ultrasonic_distance_cm(void) {
  uint16_t duration_us = ultrasonic_measure_us();
  if (duration_us == 0) return -1;  // Keine gültige Messung

  // Schall braucht ~58 µs pro cm (Hin- UND Rückweg zusammen)
  int distance_cm = duration_us / 58;

  // Messung außerhalb des sinnvollen Bereichs → ungültig
  if (distance_cm < MIN_DISTANCE_CM || distance_cm > MAX_DISTANCE_CM) return -1;

  return distance_cm;
}

// Öffentliche Funktion: Gibt die Entfernung in cm zurück
// Macht NUM_MEASUREMENTS Messungen und mittelt gültige Werte
// Gibt -1 zurück wenn keine einzige Messung gültig war
int read_distance(void) {
  int sum = 0, valid_count = 0;

  for (uint8_t i = 0; i < NUM_MEASUREMENTS; i++) {
    int dist = ultrasonic_distance_cm();
    if (dist > 0) {  // Nur gültige Messungen mitzählen
      sum += dist;
      valid_count++;
    }
    _delay_ms(10);  // Kurze Pause zwischen Messungen
  }

  // Kein einziger gültiger Wert → -1, sonst Durchschnitt
  return (valid_count == 0) ? -1 : sum / valid_count;
}