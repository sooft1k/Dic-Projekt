/*
 * ============================================================
 *  bt.c  –  Kommunikation mit dem ESP32 über UART
 * ============================================================
 *
 *  Dieses Modul empfängt Daten vom ESP32, der als Bluetooth-Brücke
 *  zwischen dem PS4/PS5-Controller und dem ATmega328P fungiert.
 *  Der ESP32 läuft eigenen Code: Er verbindet sich per Bluetooth mit dem
 *  PS4/PS5-Controller, liest dessen Joystick- und Tastendaten aus,
 *  verpackt sie in das vereinbarte Protokoll und schickt sie seriell
 *  per UART an den ATmega328P weiter.
 *  Für den ATmega sieht das genauso aus wie eine direkte Kabelverbindung –
 *  er "weiß" nicht, dass dahinter ein ESP32 und ein Controller steckt.
 *
 *  Was ist UART?
 *  UART steht für "Universal Asynchronous Receiver/Transmitter".
 *  Es ist eine serielle Schnittstelle, über die zwei Geräte miteinander
 *  reden können, indem sie Bits nacheinander über eine einzige Leitung
 *  schicken. "Asynchron" bedeutet: Es gibt keine gemeinsame Taktleitung.
 *  Stattdessen müssen beide Seiten vorher auf eine feste Geschwindigkeit
 *  (die "Baudrate") einigen – wie zwei Menschen, die vereinbaren, in
 *  gleichem Tempo zu sprechen, ohne auf ein Metronom zu hören.
 *
 *  Verbindung ESP32 → ATmega328P:
 *    ESP32 TX  →  ATmega328P RX (Pin PD0)
 *    ESP32 GND →  ATmega328P GND  (gemeinsame Masse – zwingend!)
 *
 *  Konfigurierte Baudrate: 9600 Baud = 9600 Bits pro Sekunde.
 *  Datenformat: 8N1 (8 Datenbits, kein Paritätsbit, 1 Stoppbit).
 * ============================================================
 */

#include "bt.h"
#include <avr/io.h> /* Enthält alle Register-Definitionen des ATmega328P */

/* ── bt_init: UART-Schnittstelle konfigurieren ───────────────────────────
 *
 *  Bereitet den Hardware-UART (USART0) des ATmega328P vor.
 *  USART = "Universal Synchronous/Asynchronous Receiver/Transmitter" –
 *  das ist der offizielle Name des Kommunikationsmoduls im Chip.
 *  Wir nutzen es im asynchronen Modus (= normales UART).
 *
 *  ── UBRR0: Baudrate einstellen ──────────────────────────────────────────
 *
 *  UBRR0 steht für "USART Baud Rate Register 0".
 *  Es ist 16 Bit groß und in zwei 8-Bit-Hälften aufgeteilt:
 *    UBRR0H = "High Byte" – die oberen 8 Bits (Bits 8–15)
 *    UBRR0L = "Low Byte"  – die unteren 8 Bits (Bits 0–7)
 *
 *  Der Wert im Register bestimmt, wie schnell der UART-Takt läuft.
 *  Die Formel lautet:
 *
 *      UBRR = (F_CPU / (16 × Baudrate)) − 1
 *           = (16.000.000 / (16 × 9600)) − 1
 *           = 104,17 − 1
 *           ≈ 103
 *
 *    Da 103 kleiner als 256 ist, passt der Wert komplett ins Low-Byte.
 *    Das High-Byte bleibt 0. Im ESP32-Code muss identisch eingestellt
 *    sein: Serial.begin(9600) oder Serial2.begin(9600).
 *
 *  ── UCSR0B: Empfänger und Sender aktivieren ─────────────────────────────
 *
 *  UCSR0B steht für "USART Control and Status Register 0 B".
 *  Es gibt drei solcher Kontrollregister (A, B, C), die verschiedene
 *  Aspekte des UART-Betriebs steuern.
 *
 *  RXEN0 = "Receiver Enable 0"
 *    Schaltet den Empfänger ein. Ohne dieses Bit ignoriert der ATmega
 *    alle eingehenden Daten komplett. Das "0" am Ende steht für USART0
 *    (der ATmega328P hat nur einen UART, daher immer "0").
 *    Wir aktivieren NUR den Empfänger (nicht TXEN0 = Sender), weil
 *    der Datenfluss einseitig ist: ESP32 → ATmega.
 *
 *  ── UCSR0C: Datenformat festlegen ───────────────────────────────────────
 *
 *  UCSR0C steht für "USART Control and Status Register 0 C".
 *  Es legt fest, wie ein einzelnes Datenwort aufgebaut ist.
 *
 *  UCSZ01 und UCSZ00 = "USART Character Size" Bits 1 und 0
 *    Zusammen legen diese zwei Bits die Anzahl der Datenbits pro
 *    übertragenen Byte fest. Die Kombination beider Bits auf 1
 *    ergibt 8 Datenbits – das ist der universelle Standard.
 *    Mögliche Kombinationen:
 *      UCSZ01=0, UCSZ00=0 → 5 Datenbits
 *      UCSZ01=0, UCSZ00=1 → 6 Datenbits
 *      UCSZ01=1, UCSZ00=0 → 7 Datenbits
 *      UCSZ01=1, UCSZ00=1 → 8 Datenbits ← wir verwenden das
 *
 *  Paritätsbit und Stoppbits bleiben auf Standardwert (0):
 *    Kein Paritätsbit (UPM01=0, UPM00=0)
 *    1 Stoppbit (USBS0=0)
 *
 *  ── Was bedeutet (1 << BITNAME)? ────────────────────────────────────────
 *
 *  Das ist eine Bit-Manipulation. Jedes Kontrollregister ist 8 Bit groß.
 *  Jedes Bit hat einen Namen. "1 << RXEN0" bedeutet:
 *    Nimm die Zahl 1 (binär: 00000001) und schiebe sie um RXEN0 Stellen
 *    nach links. Wenn RXEN0 = 4, entsteht: 00010000.
 *  Mit "|=" wird dieses Bit in das Register gesetzt, ohne die anderen
 *  Bits zu verändern. So kann man einzelne Bits gezielt setzen.
 * ──────────────────────────────────────────────────────────────────────── */
void bt_init(void) {
  UBRR0H = 0;   /* High-Byte der Baudrate: 0, da Wert 103 < 256 */
  UBRR0L = 103; /* Low-Byte der Baudrate: 103 → 9600 Baud bei 16 MHz */

  UCSR0B = (1 << RXEN0);                  /* Empfänger (RX) aktivieren */
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8 Datenbits pro Byte */
}

/* ── bt_data_available: Prüft ob der ESP32 ein Byte geschickt hat ────────
 *
 *  ── UCSR0A: Status-Register ─────────────────────────────────────────────
 *
 *  UCSR0A steht für "USART Control and Status Register 0 A".
 *  Im Gegensatz zu B und C enthält dieses Register vor allem Statusbits –
 *  also Bits, die der Hardware automatisch setzt, um Ereignisse zu melden.
 *
 *  RXC0 = "USART Receive Complete 0"
 *    Dieses Bit wird von der Hardware automatisch auf 1 gesetzt, sobald
 *    ein vollständiges Byte im Empfangspuffer (UDR0) angekommen ist und
 *    darauf wartet, gelesen zu werden.
 *    Sobald das Byte mit UDR0 gelesen wird, setzt die Hardware das Bit
 *    wieder auf 0.
 *
 *  Der Ausdruck (UCSR0A & (1 << RXC0)):
 *    Mit "&" (bitweises UND) isolieren wir genau das RXC0-Bit aus dem
 *    Register. Ist es 1, gibt der Ausdruck einen Wert > 0 zurück (= "true").
 *    Ist es 0, kommt 0 zurück (= "false").
 *
 *  Diese Funktion blockiert NICHT – sie schaut nur kurz nach und kehrt
 *  sofort zurück. Dadurch kann das Hauptprogramm weiterlaufen.
 *
 *  uint8_t = "unsigned integer, 8 Bit"
 *    Ein ganzzahliger Wert ohne Vorzeichen, der Werte von 0 bis 255
 *    speichern kann. Das "u" steht für "unsigned" (vorzeichenlos).
 *    Der Typ kommt aus der Bibliothek <stdint.h>, die <avr/io.h> automatisch
 *    einbindet. Er ist klarer als "unsigned char", das technisch dasselbe wäre.
 * ──────────────────────────────────────────────────────────────────────── */
uint8_t bt_data_available(void) {
  return (UCSR0A & (1 << RXC0));
}

/* ── bt_receive: Ein vom ESP32 gesendetes Byte lesen ─────────────────────
 *
 *  Liest ein Byte aus dem Empfangspuffer. Wenn noch keines da ist,
 *  wartet die Funktion in einer leeren Schleife (blockierendes Lesen).
 *
 *  ── Die Warteschleife ───────────────────────────────────────────────────
 *
 *  while (!(UCSR0A & (1 << RXC0)));
 *    Das "!" ist das logische NICHT. Die Schleife läuft solange, wie
 *    RXC0 NICHT gesetzt ist – also solange kein Byte angekommen ist.
 *    Das ";" am Ende ist der leere Schleifenrumpf – die CPU macht
 *    buchstäblich nichts außer immer wieder zu prüfen.
 *
 *  ── Fehlerbehandlung ────────────────────────────────────────────────────
 *
 *  Bevor das Byte gelesen wird, prüfen wir drei Fehlerbits in UCSR0A.
 *  WICHTIG: Diese Bits müssen gelesen werden BEVOR UDR0 gelesen wird,
 *  weil das Lesen von UDR0 sie automatisch löscht.
 *
 *  FE0 = "Frame Error 0"
 *    "Frame" = Rahmen. Ein UART-Byte hat einen festen Rahmen:
 *    Startbit → 8 Datenbits → Stoppbit. Wenn das empfangene Stoppbit
 *    nicht HIGH ist (weil z. B. ESP32 und ATmega verschiedene Baudraten
 *    haben), wird FE0 auf 1 gesetzt. Das Byte ist dann unbrauchbar.
 *
 *  DOR0 = "Data OverRun 0"
 *    "Overrun" = Überlauf. Der Empfangspuffer fasst nur 2 Bytes.
 *    Wenn ein drittes eintrifft, bevor die ersten zwei gelesen wurden,
 *    geht es verloren und DOR0 wird gesetzt. Passiert wenn der ATmega
 *    zu langsam liest oder der ESP32 zu schnell sendet.
 *
 *  UPE0 = "USART Parity Error 0"
 *    Paritätsfehler. Da wir kein Paritätsbit konfiguriert haben
 *    (UCSR0C: UPM01=0, UPM00=0), kann dieses Bit nie gesetzt werden.
 *    Es wird trotzdem geprüft – als Sicherheitsnetz für die Zukunft.
 *
 *  (void)UDR0:
 *    Wenn ein Fehler aufgetreten ist, muss UDR0 trotzdem gelesen werden,
 *    damit der Puffer geleert wird und der nächste Empfang funktioniert.
 *    Das Ergebnis wollen wir aber nicht verwenden. Der Cast "(void)" sagt
 *    dem Compiler explizit: "Ich weiß, dass ich den Rückgabewert wegwerfe,
 *    das ist Absicht." Ohne "(void)" würde der Compiler eine Warnung ausgeben.
 *
 *  ── UDR0: Der Datenpuffer ───────────────────────────────────────────────
 *
 *  UDR0 = "USART Data Register 0"
 *    Dieses Register ist der eigentliche Briefkasten. Wenn der UART
 *    ein Byte empfangen hat, liegt es hier drin und wartet darauf,
 *    gelesen zu werden. Das Lesen gibt den Puffer automatisch frei.
 * ──────────────────────────────────────────────────────────────────────── */
uint8_t bt_receive(void) {
  /* Warten bis RXC0 = 1 (Byte ist vollständig empfangen) */
  while (!(UCSR0A & (1 << RXC0)))
    ;

  /* Fehlerbits prüfen – MUSS vor dem Lesen von UDR0 passieren */
  if (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
    (void)UDR0; /* Puffer leeren, Ergebnis absichtlich verwerfen */
    return 0;
  }

  return UDR0; /* Fehlerfreies Byte aus dem Puffer holen und zurückgeben */
}