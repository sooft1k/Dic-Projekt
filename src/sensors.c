#include "sensors.h"
#include <avr/io.h>
#include <util/delay.h>

/* ── Pin-Definitionen ───────────────────────────────────────────────────── */
#define TRIG_PIN PC5    /* TRIG: Ausgang → Messimpuls senden (Port C, Pin 5 = A5) */
#define TRIG_DDR DDRC   /* Port C Direction Register */
#define TRIG_PORT PORTC /* Port C Ausgabe-Register */

#define ECHO_PIN PB4      /* ECHO: Eingang → Echo empfangen (Port B, Pin 4 = Pin 12) */
#define ECHO_DDR DDRB     /* Port B Direction Register */
#define ECHO_PORT PORTB   /* Port B Ausgabe-Register */
#define ECHO_PIN_REG PINB /* PINB = Eingangs-Register von Port B (zum Lesen des Pins) */

/* ── Messgrenzen ────────────────────────────────────────────────────────── */
#define TIMEOUT_US 3000     /* Max. Wartezeit 3000µs → ca. 50cm Reichweite */
#define MAX_DISTANCE_CM 100 /* Über 100cm ignorieren (zu weit) */
#define MIN_DISTANCE_CM 2   /* Unter 2cm ignorieren (Sensor unzuverlässig) */
#define NUM_MEASUREMENTS 1  /* Anzahl Messungen pro read_distance()-Aufruf */

static uint8_t wait_for_echo_high(uint16_t timeout_us) {
  while (!(ECHO_PIN_REG & (1 << ECHO_PIN))) { /* Solange ECHO LOW ist: warten */
    if (timeout_us == 0) return 0;            /* Zu lange gewartet → Fehler */
    _delay_us(1);                             /* 1 Mikrosekunde warten */
    timeout_us--;                             /* Zähler verringern */
  }
  return 1; /* ECHO ist HIGH → Echo ist angekommen */
}

void sensor_init(void) {
  TRIG_DDR |= (1 << TRIG_PIN);   /* TRIG → Ausgang (wir senden) */
  TRIG_PORT &= ~(1 << TRIG_PIN); /* TRIG auf LOW (Ruhezustand) */
  ECHO_DDR &= ~(1 << ECHO_PIN);  /* ECHO → Eingang (wir empfangen) */
  ECHO_PORT &= ~(1 << ECHO_PIN); /* Pull-up deaktivieren */
  _delay_ms(50);                 /* 50ms warten: Sensor braucht Zeit zum Starten */
}

uint16_t ultrasonic_measure_us(void) {
  uint16_t duration_us = 0; /* Hier wird die Laufzeit gezählt */

  TRIG_PORT &= ~(1 << TRIG_PIN); /* Schritt 1: TRIG LOW → Sensor zurücksetzen */
  _delay_ms(5);                  /* 5ms warten damit Sensor bereit ist */
  TRIG_PORT |= (1 << TRIG_PIN);  /* Schritt 2: TRIG HIGH → Messimpuls starten */
  _delay_us(10);                 /* 10µs HIGH halten (Mindestlänge laut Datenblatt) */
  TRIG_PORT &= ~(1 << TRIG_PIN); /* TRIG wieder LOW */

  if (!wait_for_echo_high(TIMEOUT_US)) /* Schritt 3: Warten bis Echo kommt */
    return 0;                          /* Kein Echo → Fehler zurückgeben */

  while (ECHO_PIN_REG & (1 << ECHO_PIN)) {   /* Schritt 4: Solange ECHO HIGH: zählen */
    _delay_us(1);                            /* 1 Mikrosekunde warten */
    duration_us++;                           /* Zähler erhöhen */
    if (duration_us >= TIMEOUT_US) return 0; /* Zu lange → Timeout */
  }

  return duration_us; /* Gemessene Laufzeit in Mikrosekunden zurückgeben */
}

int ultrasonic_distance_cm(void) {
  uint16_t duration_us = ultrasonic_measure_us(); /* Laufzeit messen */
  if (duration_us == 0) return -1;                /* Keine gültige Messung */

  int distance_cm = duration_us / 58; /* Laufzeit in cm umrechnen */

  if (distance_cm < MIN_DISTANCE_CM || distance_cm > MAX_DISTANCE_CM)
    return -1; /* Außerhalb des gültigen Bereichs → ungültig */

  return distance_cm; /* Gültige Distanz in cm zurückgeben */
}

int read_distance(void) {
  int sum = 0, valid_count = 0; /* Summe und Anzahl gültiger Messungen */

  for (uint8_t i = 0; i < NUM_MEASUREMENTS; i++) {
    int dist = ultrasonic_distance_cm(); /* Eine Messung durchführen */
    if (dist > 0) {                      /* Nur gültige Messungen zählen */
      sum += dist;                       /* Zur Summe addieren */
      valid_count++;                     /* Zähler erhöhen */
    }
    _delay_ms(10); /* 10ms Pause zwischen Messungen */
  }

  /* Ternärer Operator: Bedingung ? Wert_wenn_wahr : Wert_wenn_falsch */
  return (valid_count == 0) ? -1 : sum / valid_count;
  /* → Keine gültige Messung → -1, sonst Durchschnitt */
}