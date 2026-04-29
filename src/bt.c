/*
 * ============================================================
 *  bt.c – Bluetooth-Empfang über UART
 * ============================================================
 *
 *  Was macht dieser Code?
 *  ─────────────────────
 *  Empfängt Daten vom ESP32 über eine serielle Kabelverbindung (UART).
 *  Der ESP32 ist per Bluetooth mit dem PS5-Controller verbunden und
 *  schickt die Joystick-Daten per Kabel an den Arduino weiter.
 *
 *  Verbindung:
 *    ESP32 TX (Pin 17) → Arduino Pin 0 (RX)
 *    ESP32 GND         → Arduino GND
 *
 *  Abkürzungen:
 *  ─────────────
 *  UART   = Universal Asynchronous Receiver Transmitter
 *           Serielle Schnittstelle: Bits werden nacheinander übertragen
 *           "Asynchron" = kein gemeinsamer Takt, beide Seiten brauchen gleiche Baudrate
 *  USART  = Universal Synchronous/Asynchronous Receiver Transmitter
 *           Der offizielle Name des UART-Moduls im ATmega328P
 *  Baud   = Bits pro Sekunde. 9600 Baud = 9600 Bits/Sek
 *  UBRR0  = USART Baud Rate Register – stellt die Übertragungsgeschwindigkeit ein
 *  UCSR0A = USART Control and Status Register A – enthält Status-Flags
 *  UCSR0B = USART Control and Status Register B – aktiviert Sender/Empfänger
 *  UCSR0C = USART Control and Status Register C – Datenformat einstellen
 *  UDR0   = USART Data Register – hier liegt das empfangene Byte
 *  RXC0   = Receive Complete – Flag: 1 wenn neues Byte im Puffer liegt
 *  RXEN0  = Receiver Enable – schaltet den Empfänger ein
 *  UCSZ01 = USART Character Size Bit 1 – Anzahl Datenbits
 *  UCSZ00 = USART Character Size Bit 0 – zusammen mit UCSZ01: 8 Datenbits
 *  FE0    = Framing Error – Stoppbit fehlt (falsche Baudrate oder Leitungsproblem)
 *  DOR0   = Data OverRun – Byte verloren weil Puffer voll war
 *  UPE0   = USART Parity Error – Paritätsfehler bei der Übertragung
 * ============================================================
 */

#include "bt.h"
#include <avr/io.h> /* AVR Register-Definitionen */

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: bt_init
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Richtet die serielle Schnittstelle (UART) für den Empfang ein.
 *   Stellt Baudrate auf 9600 ein (muss mit ESP32 übereinstimmen).
 *
 * Baudrate-Formel:
 *   UBRR = (F_CPU / (16 × Baudrate)) - 1
 *         = (16.000.000 / (16 × 9600)) - 1
 *         = 104 - 1 = 103
 *   → UBRR0H = 0 (obere 8 Bit, hier 0 weil 103 < 256)
 *   → UBRR0L = 103 (untere 8 Bit)
 *
 * Datenformat 8N1:
 *   8 Datenbits, kein Paritätsbit, 1 Stoppbit (Standard für UART)
 *
 * (1 << BITNAME): schiebt 1 an die Stelle des Bits im 8-Bit Register
 * |= : setzt dieses Bit ohne andere zu ändern (bitweises ODER)
 * ═══════════════════════════════════════════════════════════════════════════ */
void bt_init(void) {
  UBRR0H = 0;   /* Baudrate oberes Byte: 0 (da 103 < 256) */
  UBRR0L = 103; /* Baudrate unteres Byte: 103 → 9600 Baud bei 16MHz */

  UCSR0B = (1 << RXEN0);                  /* Nur Empfänger einschalten (kein Sender nötig) */
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8 Datenbits pro Byte (Standard) */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: bt_data_available
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Prüft ob ein neues Byte vom ESP32 angekommen ist.
 *   Blockiert NICHT – kehrt sofort zurück.
 *
 * Rückgabe: 1 wenn Byte bereit liegt, 0 wenn nichts da ist.
 *
 * RXC0-Flag wird von der Hardware automatisch auf 1 gesetzt
 * wenn ein vollständiges Byte im Empfangspuffer (UDR0) liegt.
 * ═══════════════════════════════════════════════════════════════════════════ */
uint8_t bt_data_available(void) {
  return (UCSR0A & (1 << RXC0)); /* RXC0 = 1 → Byte liegt bereit */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNKTION: bt_receive
 * ───────────────────────────────────────────────────────────────────────────
 * Was macht sie?
 *   Liest ein Byte aus dem Empfangspuffer und gibt es zurück.
 *   Wartet falls noch kein Byte angekommen ist (blockierend).
 *   Prüft vorher ob ein Übertragungsfehler aufgetreten ist.
 *
 * Fehlerprüfung MUSS vor dem Lesen von UDR0 passieren,
 * weil das Lesen von UDR0 die Fehlerbits automatisch löscht!
 *
 * Bei Fehler: UDR0 trotzdem lesen um Puffer zu leeren → 0 zurückgeben.
 * (void)UDR0: "(void)" sagt dem Compiler: ich verwerfe den Wert absichtlich.
 *
 * UDR0 lesen löscht automatisch das RXC0-Flag.
 * ═══════════════════════════════════════════════════════════════════════════ */
uint8_t bt_receive(void) {
  while (!(UCSR0A & (1 << RXC0))); /* Warten bis Byte vollständig empfangen */

  /* Fehlerprüfung: Fehlerbits auslesen BEVOR UDR0 gelesen wird */
  if (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
    (void)UDR0; /* Puffer leeren – Ergebnis absichtlich verworfen */
    return 0;   /* 0 als Fehler-Signal zurückgeben */
  }

  return UDR0; /* Fehlerfreies Byte aus dem Puffer lesen und zurückgeben */
}