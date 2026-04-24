// bt.c – Bluetooth-Empfang über UART (serielle Schnittstelle)
// Der ESP32 empfängt die PS5-Controller-Daten per Bluetooth und
// leitet sie per UART an den ATmega328P weiter.

#include "bt.h"
#include <avr/io.h>

// Initialisiert die serielle Schnittstelle (UART) für den Empfang
void bt_init(void) {
  UBRR0H = 0;    // Oberes Byte der Baudrate (hier 0)
  UBRR0L = 103;  // Unteres Byte → ergibt 9600 Baud bei 16 MHz
                 // Formel: 16000000 / (16 × 9600) - 1 = 103

  UCSR0B = (1 << RXEN0);                   // Nur Empfangen (RX) aktivieren
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-Bit Datenpakete (Standard)
}

// Prüft ob ein neues Byte vom ESP32 angekommen ist
// Gibt 1 zurück wenn ja, 0 wenn nichts da ist
uint8_t bt_data_available(void) {
  return (UCSR0A & (1 << RXC0));  // RXC0-Flag = 1 bedeutet: Byte liegt bereit
}

// Liest ein Byte aus dem Empfangspuffer und gibt es zurück
uint8_t bt_receive(void) {
  while (!(UCSR0A & (1 << RXC0)));  // Warten bis ein Byte angekommen ist

  // Fehlerprüfung: FE0 = kaputtes Paket, DOR0 = Überlauf, UPE0 = Paritätsfehler
  if (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
    (void)UDR0;  // Fehlerhaftes Byte lesen um den Puffer zu leeren
    return 0;    // 0 zurückgeben als Fehlerindikator
  }

  return UDR0;  // Gültiges Byte aus dem Datenpuffer zurückgeben
}