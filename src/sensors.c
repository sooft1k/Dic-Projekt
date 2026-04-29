/*
 * ============================================================
 *  sensors.c – Ultraschall-Distanzmessung mit HC-SR04
 * ============================================================
 *
 *  Was macht dieser Code?
 *  ─────────────────────
 *  Misst die Entfernung zu Hindernissen in Zentimeter.
 *  Prinzip:
 *    1. Arduino sendet kurzen Schallimpuls über TRIG-Pin
 *    2. Schall breitet sich aus und prallt vom Hindernis zurück
 *    3. Sensor setzt ECHO-Pin HIGH solange Echo kommt
 *    4. Arduino misst wie lange ECHO HIGH war → das ist die Laufzeit
 *    5. Distanz = Laufzeit / 58  (Schall braucht ~58µs pro cm hin+zurück)
 *
 *  Abkürzungen:
 *  ─────────────
 *  DDR  = Data Direction Register – Bit=1 → Ausgang, Bit=0 → Eingang
 *  PORT = Ausgabe-Register – setzt Pin auf HIGH oder LOW
 *  PIN  = Eingangs-Register – liest aktuellen Pin-Zustand (nur lesen!)
 *  TRIG = Trigger – Ausgangspin: Arduino sendet Messimpuls
 *  ECHO = Echo    – Eingangspin: Arduino empfängt Echo-Signal
 *  µs   = Mikrosekunden (1µs = 0,000001 Sekunden)
 *  ms   = Millisekunden (1ms = 0,001 Sekunden)
 * ============================================================
 */

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

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: wait_for_echo_high (intern)
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Wartet bis der ECHO-Pin auf HIGH geht (Signal kommt zurück).
 *   Bricht nach timeout_us Mikrosekunden ab wenn kein Echo kommt.
 *
 * Rückgabe: 1 = Echo kam, 0 = Timeout (kein Echo)
 *
 * Was bedeutet !(ECHO_PIN_REG & (1 << ECHO_PIN))?
 *   & (1 << ECHO_PIN) = isoliert nur das ECHO-Bit aus dem Register
 *   !                 = logisches NICHT → Schleife läuft solange ECHO LOW ist
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint8_t wait_for_echo_high(uint16_t timeout_us) {
  while (!(ECHO_PIN_REG & (1 << ECHO_PIN))) { /* Solange ECHO LOW ist: warten */
    if (timeout_us == 0) return 0;            /* Zu lange gewartet → Fehler */
    _delay_us(1);                             /* 1 Mikrosekunde warten */
    timeout_us--;                             /* Zähler verringern */
  }
  return 1; /* ECHO ist HIGH → Echo ist angekommen */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: sensor_init
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Konfiguriert TRIG als Ausgang und ECHO als Eingang.
 *   Wartet 50ms damit der Sensor hochfahren kann.
 *
 * Was bedeutet |= und &= ~?
 *   |=  (1 << PIN) → setzt dieses Bit auf 1 ohne andere zu ändern
 *   &= ~(1 << PIN) → setzt dieses Bit auf 0 ohne andere zu ändern
 * ═══════════════════════════════════════════════════════════════════════════ */
void sensor_init(void) {
  TRIG_DDR |= (1 << TRIG_PIN);   /* TRIG → Ausgang (wir senden) */
  TRIG_PORT &= ~(1 << TRIG_PIN); /* TRIG auf LOW (Ruhezustand) */
  ECHO_DDR &= ~(1 << ECHO_PIN);  /* ECHO → Eingang (wir empfangen) */
  ECHO_PORT &= ~(1 << ECHO_PIN); /* Pull-up deaktivieren */
  _delay_ms(50);                 /* 50ms warten: Sensor braucht Zeit zum Starten */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: ultrasonic_measure_us
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Führt eine Ultraschallmessung durch und gibt die Echo-Laufzeit
 *   in Mikrosekunden zurück. Gibt 0 zurück bei Fehler/Timeout.
 *
 * Ablauf laut HC-SR04 Datenblatt:
 *   Schritt 1: TRIG auf LOW setzen und 5ms warten (sauberer Startzustand)
 *   Schritt 2: TRIG für 10µs auf HIGH → Sensor schickt Schallimpuls
 *   Schritt 3: Warten bis ECHO HIGH wird (Sensor hat gesendet)
 *   Schritt 4: Zählen wie lange ECHO HIGH bleibt = Laufzeit des Schalls
 *
 * duration_us++ = Kurzform für duration_us = duration_us + 1
 * ═══════════════════════════════════════════════════════════════════════════ */
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

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: ultrasonic_distance_cm
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Rechnet die gemessene Laufzeit in Zentimeter um.
 *   Gibt -1 zurück bei ungültiger Messung.
 *
 * Formel: Schall = 343 m/s = 0,0343 cm/µs
 *   Hin UND zurück → durch 2 teilen
 *   Distanz = duration_us × 0,0343 / 2 = duration_us / 58
 *
 * int (mit Vorzeichen) weil -1 als Fehlerwert zurückgegeben wird.
 * ═══════════════════════════════════════════════════════════════════════════ */
int ultrasonic_distance_cm(void) {
  uint16_t duration_us = ultrasonic_measure_us(); /* Laufzeit messen */
  if (duration_us == 0) return -1;                /* Keine gültige Messung */

  int distance_cm = duration_us / 58; /* Laufzeit in cm umrechnen */

  if (distance_cm < MIN_DISTANCE_CM || distance_cm > MAX_DISTANCE_CM)
    return -1; /* Außerhalb des gültigen Bereichs → ungültig */

  return distance_cm; /* Gültige Distanz in cm zurückgeben */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: read_distance
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Öffentliche Funktion die von main.c aufgerufen wird.
 *   Macht NUM_MEASUREMENTS Messungen, filtert ungültige heraus,
 *   und gibt den Durchschnitt zurück.
 *   Gibt -1 zurück wenn keine einzige Messung gültig war.
 *
 * Warum 10ms Pause?
 *   Der HC-SR04 braucht Zeit zwischen Messungen zum Zurücksetzen.
 * ═══════════════════════════════════════════════════════════════════════════ */
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