#include "bt.h"
#include <avr/io.h>
// UBRR0 = "USART Baud Rate Register"
//   16-Bit Wert aufgeteilt in zwei 8-Bit Hälften (UBRR0H = high, UBRR0L = low).
// Wert berechnen:
//   UBRR = (F_CPU / (16 × Baudrate)) - 1
//        = (16.000.000 / (16 × 9600)) - 1
//        = 104 - 1 = 103
// UCSR0B = "USART Control Status Register B"
//   RXEN0 = "Receiver Enable" - Empfänger einschalten
// UCSR0C = "USART Control Status Register C"

void bt_init(void) {
  UBRR0H = 0;    // Oberes Byte: 0
  UBRR0L = 103;  // Unteres Byte: 103 → 9600 Baud bei 16MHz

  UCSR0B = (1 << RXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  //   UCSZ01 + UCSZ00 zusammen = 8 Datenbits pro Zeichen
}

// Prüft OB ein Byte angekommen ist
// UCSR0A = Status-Register
// RXC0 = "Receive Complete" - Bit wird automatisch auf 1 gesetzt wenn ein vollständiges Byte im
// Empfangspuffer liegt.
uint8_t bt_data_available(void) {
  return (UCSR0A & (1 << RXC0));
}

uint8_t bt_receive(void) {
  while (!(UCSR0A & (1 << RXC0)));  // Warten bis ein Byte angekommen ist.
  // Drei mögliche Übertragungsfehler:
  //   FE0  = "Frame Error" - das Stoppbit fehlt (oft falsche Baudrate)
  //   DOR0 = "Data Overrun" - Byte ging verloren weil Puffer voll war
  //   UPE0 = "USART Parity Error" - Paritätsfehler
  if (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
    (void)UDR0;  // Byte trotzdem lesen damit der Puffer leer wird,
    return 0;    // 0 als Fehler-Signal zurückgeben
  }
  return UDR0;
}