/*
 * bt.c – Kommunikation mit dem ESP32 über UART
 * =============================================
 *
 * Der ESP32 hostet einen Webserver. Wenn jemand im Browser einen Fahrbefehl
 * drückt, schickt der ESP32 diesen Befehl per UART-Kabel an den Arduino.
 *
 * Was ist UART?
 *   Eine serielle Schnittstelle: Bits wandern nacheinander über eine Leitung.
 *   "Asynchron" = kein gemeinsamer Takt, beide Seiten müssen auf die gleiche
 *   Baudrate (Übertragungsgeschwindigkeit) eingestellt sein.
 *   9600 Baud = 9600 Bits/Sekunde. Format 8N1: 8 Datenbits, kein Paritätsbit, 1 Stoppbit.
 *
 * Verbindung: ESP32 TX (Pin 17) → Arduino RX (PD0). Nur eine Richtung –
 * der Arduino empfängt nur, er sendet nichts zurück.
 *
 * Was ist ein Register?
 *   Eine spezielle Speicherstelle direkt im Chip. Einen Wert hineinschreiben
 *   bedeutet: die Hardware reagiert sofort – kein Umweg über Software-Logik.
 *
 * Bit-Operationen (tauchen überall in AVR-Code auf):
 *   (1 << N)    → Zahl mit genau Bit N gesetzt  (z.B. N=4 → 0b00010000)
 *   REG |= ...  → Bit setzen,   alle anderen unberührt  (bitweises ODER)
 *   REG &= ~... → Bit löschen,  alle anderen unberührt  (bitweises UND + Invertierung)
 */

#include "bt.h"
#include <avr/io.h>

/*
 * bt_init – UART-Empfänger einmalig beim Start konfigurieren
 *
 * UBRR0 (Baud Rate Register, 16 Bit aufgeteilt in High- und Low-Byte):
 *   Formel: UBRR = F_CPU / (16 × Baudrate) − 1 = 16.000.000 / 153.600 − 1 ≈ 103
 *   103 passt in 8 Bit → UBRR0H = 0, UBRR0L = 103
 *
 * UCSR0B = Control Register B:
 *   RXEN0 schaltet den Empfänger ein. TXEN0 (Sender) bleibt aus –
 *   der Arduino schickt keine Daten zurück.
 *
 * UCSR0C = Control Register C:
 *   UCSZ01 + UCSZ00 beide auf 1 → 8 Datenbits pro Byte (Standard).
 */
void bt_init(void) {
  UBRR0H = 0;
  UBRR0L = 103;
  UCSR0B = (1 << RXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/*
 * bt_data_available – Liegt ein empfangenes Byte im Puffer bereit?
 *
 * Blockiert NICHT – gibt sofort zurück ob etwas da ist oder nicht.
 * Das Bit RXC0 im Status-Register UCSR0A wird von der Hardware automatisch
 * auf 1 gesetzt, sobald ein vollständiges Byte empfangen wurde.
 * Rückgabe: Wert > 0 = Byte bereit, 0 = noch nichts angekommen.
 */
uint8_t bt_data_available(void) {
  return (UCSR0A & (1 << RXC0));
}

/*
 * bt_receive – Ein Byte blockierend lesen
 *
 * Wartet bis ein Byte angekommen ist, prüft auf Übertragungsfehler
 * und gibt das Byte zurück. Bei Fehler: Puffer leeren, 0 zurückgeben.
 *
 * Wichtig: Fehlerprüfung MUSS vor dem Lesen von UDR0 passieren,
 * weil das Lesen von UDR0 die Fehlerbits automatisch löscht!
 *   FE0  = Frame Error  – Stoppbit fehlt → falsche Baudrate oder Kabeldefekt
 *   DOR0 = Data OverRun – Byte verloren weil Puffer bereits voll war
 *   UPE0 = Parity Error – Paritätsfehler (bei 8N1 nicht aktiv, trotzdem geprüft)
 *
 * (void)UDR0: Liest den Puffer und wirft das Ergebnis weg.
 * Das "(void)" sagt dem Compiler, dass das Absicht ist.
 */
uint8_t bt_receive(void) {
  while (!(UCSR0A & (1 << RXC0)))
    ;
  if (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
    (void)UDR0;
    return 0;
  }
  return UDR0;
}