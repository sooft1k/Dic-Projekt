#include "bt.h"
#include <avr/io.h>

/* Richtet die serielle Verbindung (UART) zum ESP32 ein
   Der ESP32 sendet Controller-Daten → Arduino empfängt sie hier */
void bt_init(void) {
  UBRR0H = 0;             // Baudrate oberes Byte (bei 9600 Baud = 0)
  UBRR0L = 103;           // Baudrate unteres Byte → Formel: (16MHz / (16 × 9600)) - 1 = 103
  UCSR0B = (1 << RXEN0);  // Empfänger aktivieren (nur empfangen, nicht senden)
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 Datenbits einstellen (Standard)
}

/* Prüft ob ein neues Byte vom ESP32 angekommen ist
   Gibt 1 zurück wenn Daten bereit liegen, sonst 0 */
uint8_t bt_data_available(void) {
  return (UCSR0A & (1 << RXC0));  // RXC0 Flag = 1 wenn neues Byte im Puffer
}

/* Liest ein Byte vom ESP32
   Prüft vorher ob ein Übertragungsfehler aufgetreten ist */
uint8_t bt_receive(void) {
  while (!(UCSR0A & (1 << RXC0)))
    ;  // Warten bis Byte angekommen ist

  // Fehlerprüfung: FE0=Stopbit fehlt, DOR0=Puffer voll, UPE0=Paritätsfehler
  if (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
    (void)UDR0;  // Fehlerhaftes Byte verwerfen (muss trotzdem gelesen werden)
    return 0;    // 0 zurückgeben als Fehlersignal
  }

  return UDR0;  // Korrektes Byte aus dem Empfangspuffer lesen und zurückgeben
}