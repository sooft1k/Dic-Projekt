#include "bt.h"
#include <avr/io.h> /* AVR Register-Definitionen */

void bt_init(void) {
  UBRR0H = 0;   /* Baudrate oberes Byte: 0 (da 103 < 256) */
  UBRR0L = 103; /* Baudrate unteres Byte: 103 → 9600 Baud bei 16MHz */

  UCSR0B = (1 << RXEN0);                  /* Nur Empfänger einschalten (kein Sender nötig) */
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8 Datenbits pro Byte (Standard) */
}

uint8_t bt_data_available(void) {
  return (UCSR0A & (1 << RXC0)); /* RXC0 = 1 → Byte liegt bereit */
}

uint8_t bt_receive(void) {
  while (!(UCSR0A & (1 << RXC0))); /* Warten bis Byte vollständig empfangen */

  /* Fehlerprüfung: Fehlerbits auslesen BEVOR UDR0 gelesen wird */
  if (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
    (void)UDR0; /* Puffer leeren – Ergebnis absichtlich verworfen */
    return 0;   /* 0 als Fehler-Signal zurückgeben */
  }

  return UDR0; /* Fehlerfreies Byte aus dem Puffer lesen und zurückgeben */
}