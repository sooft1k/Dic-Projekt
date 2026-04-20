/*
 * ============================================================
 *  motors.c  –  Motorsteuerung für 4WD-Roboter
 * ============================================================
 *
 *  Dieses Modul steuert vier DC-Motoren über zwei L298N Motor-Treiber.
 *  Ein L298N ist eine H-Brücke (H-Bridge): Er kann Strom in beide
 *  Richtungen durch einen Motor schicken und damit Vorwärts- und
 *  Rückwärtsfahrt ermöglichen. Die Geschwindigkeit wird über PWM geregelt.
 *
 *  Was ist PWM (Pulsweitenmodulation)?
 *    Der Mikrocontroller schaltet den Ausgangspin sehr schnell ein und aus.
 *    Ist er pro Zyklus länger "ein" als "aus", spürt der Motor im Schnitt
 *    mehr Spannung und dreht schneller. Ist er kürzer "ein", läuft er
 *    langsamer. Ein OCR-Wert von 255 = 100% eingeschaltet = Vollgas.
 *    Ein OCR-Wert von 0 = 0% = komplett aus.
 *
 *  Aufbau:
 *    FL = Front Left  (vorne links)   – L298N #1, Kanal A
 *    RL = Rear Left   (hinten links)  – L298N #1, Kanal B
 *    FR = Front Right (vorne rechts)  – L298N #2, Kanal A
 *    RR = Rear Right  (hinten rechts) – L298N #2, Kanal B
 * ============================================================
 */

#include "motors.h"
#include <avr/io.h>
#include <util/delay.h>

#define FL_IN1_PIN PD4 /* Vorne links:   IN1 → Port D, Pin 4 */
#define FL_IN2_PIN PD5 /* Vorne links:   IN2 → Port D, Pin 5 */
#define FL_DDR DDRD
#define FL_PORT PORTD

#define RL_IN3_PIN PD6 /* Hinten links:  IN3 → Port D, Pin 6 */
#define RL_IN4_PIN PD7 /* Hinten links:  IN4 → Port D, Pin 7 */
#define RL_DDR DDRD
#define RL_PORT PORTD

#define FR_IN1_PIN PC0 /* Vorne rechts:  IN1 → Port C, Pin 0 */
#define FR_IN2_PIN PC1 /* Vorne rechts:  IN2 → Port C, Pin 1 */
#define FR_DDR DDRC
#define FR_PORT PORTC

#define RR_IN3_PIN PC2 /* Hinten rechts: IN3 → Port C, Pin 2 */
#define RR_IN4_PIN PC3 /* Hinten rechts: IN4 → Port C, Pin 3 */
#define RR_DDR DDRC
#define RR_PORT PORTC

/* ── PWM-Pins (Geschwindigkeitssteuerung) ────────────────────────────────
 *
 *  Was ist Hardware-PWM?
 *    Der ATmega328P hat eingebaute Timer, die PWM-Signale vollautomatisch
 *    erzeugen. Man schreibt nur einen Wert in ein Register (OCR), und der
 *    Timer schaltet den Pin selbstständig ein und aus – ohne dass die CPU
 *    dauernd eingreifen muss. Das ist effizienter als Software-PWM.
 *
 *  Was sind OC1A, OC1B, OC2A, OC2B?
 *    "OC" steht für "Output Compare" – der Ausgang, den der Timer
 *    steuert wenn sein Zähler den Vergleichswert (OCR) erreicht.
 *    Die Zahl (1 oder 2) ist die Timer-Nummer, der Buchstabe (A oder B)
 *    ist der Kanal. Timer 1 hat zwei Kanäle (A und B), Timer 2 auch.
 *
 *  Warum sind PWM-Pins festgelegt?
 *    Hardware-PWM funktioniert NUR an speziellen Pins, die direkt mit
 *    dem Timer-Modul verbunden sind. Man kann sie nicht frei wählen.
 *    PB1=OC1A, PB2=OC1B, PB3=OC2A, PD3=OC2B – das sind die einzigen
 *    Optionen für Timer 1 und Timer 2 beim ATmega328P.
 * ──────────────────────────────────────────────────────────────────────── */
#define PWM_FL_PIN PB1 /* Timer 1, Kanal A (OC1A) → steuert FL-Geschwindigkeit */
#define PWM_RL_PIN PB2 /* Timer 1, Kanal B (OC1B) → steuert RL-Geschwindigkeit */
#define PWM_L_DDR DDRB

#define PWM_FR_PIN PB3 /* Timer 2, Kanal A (OC2A) → steuert FR-Geschwindigkeit */
#define PWM_RR_PIN PD3 /* Timer 2, Kanal B (OC2B) → steuert RR-Geschwindigkeit */
#define PWM_FR_DDR DDRB
#define PWM_RR_DDR DDRD

#define MOTOR_SPEED_DEFAULT 255 /* Maximale Geschwindigkeit beim Start */

/* ── Interne Drehrichtungs-Enumeration ───────────────────────────────────
 *
 *  Was ist ein "typedef enum"?
 *    "enum" = Enumeration (Aufzählung). Statt überall die Zahlen 0, 1, 2
 *    zu schreiben, erstellt man benannte Konstanten. Das macht den Code
 *    lesbarer und verhindert Fehler durch falsche Zahlen.
 *    "typedef" erlaubt es, den Typ direkt als "MotorDirection" zu verwenden,
 *    ohne jedes Mal "enum MotorDirection" schreiben zu müssen.
 *
 *  "static" vor typedef:
 *    Hier nicht anwendbar – static gilt für Variablen und Funktionen.
 *    Die Enumeration ist intern, weil sie nur in dieser Datei gebraucht wird.
 * ──────────────────────────────────────────────────────────────────────── */
typedef enum { MOTOR_STOP, MOTOR_BACKWARD, MOTOR_FORWARD } MotorDirection;

/* Was ist "static uint8_t"?
 *   "static" bei einer globalen Variable bedeutet: Diese Variable ist nur
 *   in dieser Datei sichtbar. Andere Dateien können sie nicht direkt lesen
 *   oder verändern – sie ist "privat" für dieses Modul.
 *   "uint8_t" = unsigned 8-bit integer, Wertebereich 0–255. Passend für
 *   PWM-Werte, da OCR-Register ebenfalls 8 Bit groß sind. */
static uint8_t current_speed = MOTOR_SPEED_DEFAULT;

/* ── set_left_motors: Drehrichtung der linken Motoren setzen ─────────────
 *
 *  "static" vor der Funktion: Nur intern in motors.c verwendbar.
 *  Von außen (main.c) ruft man motor_forward(), motor_stop() usw. auf –
 *  diese Hilfsfunktion bleibt verborgen.
 *
 *  Die Bit-Operatoren im Detail:
 *
 *    FL_PORT |= (1 << FL_IN2_PIN)
 *      "|=" = bitweises ODER mit Zuweisung.
 *      (1 << FL_IN2_PIN) erzeugt eine Zahl, bei der nur das FL_IN2-Bit
 *      gesetzt ist. Mit "|=" wird genau dieses Bit auf 1 gesetzt,
 *      ohne die anderen Bits im PORT-Register zu verändern.
 *      → Pin geht auf HIGH.
 *
 *    FL_PORT &= ~(1 << FL_IN1_PIN)
 *      "&=" = bitweises UND mit Zuweisung.
 *      "~" = bitweise Invertierung (0 wird 1, 1 wird 0).
 *      ~(1 << FL_IN1_PIN) ergibt eine Zahl, bei der alle Bits 1 sind
 *      AUSSER dem FL_IN1-Bit. Mit "&=" wird nur dieses eine Bit auf 0
 *      gesetzt, alle anderen bleiben unverändert.
 *      → Pin geht auf LOW.
 * ──────────────────────────────────────────────────────────────────────── */
static void set_left_motors(MotorDirection dir) {
  switch (dir) {
    case MOTOR_STOP:
      FL_PORT &= ~(1 << FL_IN1_PIN); /* IN1 = LOW  */
      FL_PORT &= ~(1 << FL_IN2_PIN); /* IN2 = LOW  → Stop */
      RL_PORT &= ~(1 << RL_IN3_PIN);
      RL_PORT &= ~(1 << RL_IN4_PIN);
      break;
    case MOTOR_FORWARD:
      FL_PORT &= ~(1 << FL_IN1_PIN); /* IN1 = LOW  */
      FL_PORT |= (1 << FL_IN2_PIN);  /* IN2 = HIGH → Vorwärts */
      RL_PORT &= ~(1 << RL_IN3_PIN);
      RL_PORT |= (1 << RL_IN4_PIN);
      break;
    case MOTOR_BACKWARD:
      FL_PORT |= (1 << FL_IN1_PIN);  /* IN1 = HIGH */
      FL_PORT &= ~(1 << FL_IN2_PIN); /* IN2 = LOW  → Rückwärts */
      RL_PORT |= (1 << RL_IN3_PIN);
      RL_PORT &= ~(1 << RL_IN4_PIN);
      break;
  }
}

/* ── set_right_motors: Identisch für die rechte Seite ────────────────────
 *
 *  FR (Port C, Pin 0+1) und RR (Port C, Pin 2+3) – selbe Logik wie links.
 * ──────────────────────────────────────────────────────────────────────── */
static void set_right_motors(MotorDirection dir) {
  switch (dir) {
    case MOTOR_STOP:
      FR_PORT &= ~(1 << FR_IN1_PIN);
      FR_PORT &= ~(1 << FR_IN2_PIN);
      RR_PORT &= ~(1 << RR_IN3_PIN);
      RR_PORT &= ~(1 << RR_IN4_PIN);
      break;
    case MOTOR_FORWARD:
      FR_PORT &= ~(1 << FR_IN1_PIN);
      FR_PORT |= (1 << FR_IN2_PIN);
      RR_PORT &= ~(1 << RR_IN3_PIN);
      RR_PORT |= (1 << RR_IN4_PIN);
      break;
    case MOTOR_BACKWARD:
      FR_PORT |= (1 << FR_IN1_PIN);
      FR_PORT &= ~(1 << FR_IN2_PIN);
      RR_PORT |= (1 << RR_IN3_PIN);
      RR_PORT &= ~(1 << RR_IN4_PIN);
      break;
  }
}

/* ── motor_pwm_init: Hardware-Timer für PWM konfigurieren ────────────────
 *
 *  Was sind TCCRxA und TCCRxB?
 *    TCCR = "Timer/Counter Control Register"
 *    Jeder Timer hat zwei Kontrollregister (A und B), die zusammen
 *    festlegen wie der Timer arbeitet. Die Zahl (1 oder 2) ist die
 *    Timer-Nummer.
 *
 *  ── Timer 1 (steuert FL und RL – die linken Motoren) ────────────────────
 *
 *  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10)
 *
 *    COM1A1 = "Compare Output Mode for Channel A, Bit 1"
 *    COM1B1 = "Compare Output Mode for Channel B, Bit 1"
 *      Diese Bits legen fest, was mit dem PWM-Ausgangspin passiert, wenn
 *      der Timer-Zähler den Vergleichswert (OCR1A/OCR1B) erreicht.
 *      Nur COM1A1=1 (ohne COM1A0=1) bedeutet: "Non-Inverting Mode" –
 *      der Pin geht auf LOW wenn der Zähler OCR erreicht, und auf HIGH
 *      wenn er wieder bei 0 anfängt. Das ergibt ein normales PWM-Signal.
 *
 *    WGM10 = "Waveform Generation Mode, Bit 0"
 *      WGM-Bits bestimmen, wie der Timer zählt. Zusammen mit WGM12
 *      (in TCCR1B) ergibt WGM10=1 den Modus "Phase Correct PWM, 8-Bit":
 *        Der Zähler zählt von 0 bis 255, dann rückwärts von 255 bis 0,
 *        dann wieder vorwärts – ständig hin und her ("Phase Correct").
 *        Das erzeugt ein sehr gleichmäßiges PWM-Signal, ideal für Motoren.
 *        "8-Bit" bedeutet: Auflösung 0–255 (256 Stufen).
 *
 *  TCCR1B = (1 << WGM12) | (1 << CS11)
 *
 *    WGM12 = "Waveform Generation Mode, Bit 2"
 *      In Kombination mit WGM10 aus TCCR1A aktiviert dies den
 *      "Phase Correct PWM 8-Bit" Modus (Modus 5 laut Datenblatt).
 *
 *    CS11 = "Clock Select, Bit 1"
 *      CS = Clock Select. Diese Bits legen den Prescaler fest.
 *      Der Prescaler teilt den Systemtakt, bevor er zum Timer kommt.
 *      CS11=1 (ohne CS10 oder CS12) = Prescaler 8:
 *        Timer-Takt = 16.000.000 / 8 = 2.000.000 Hz
 *        PWM-Frequenz (Phase Correct) = 2.000.000 / (2 × 255) ≈ 3.922 Hz
 *        Das ist hörbar leise – Motoren pfeifen nicht.
 *
 *  ── Timer 2 (steuert FR und RR – die rechten Motoren) ───────────────────
 *
 *  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21)
 *
 *    COM2A1, COM2B1: Identisch zu Timer 1 – Non-Inverting PWM-Ausgabe.
 *
 *    WGM20 und WGM21 zusammen = "Fast PWM, 8-Bit" (Modus 3):
 *      Im Gegensatz zu "Phase Correct" zählt der Timer hier nur vorwärts
 *      von 0 bis 255, springt dann zurück auf 0 ("Fast PWM").
 *      Die Frequenz ist doppelt so hoch wie bei Phase Correct,
 *      die PWM-Qualität etwas schlechter – für diesen Anwendungsfall
 *      aber ausreichend.
 *
 *  TCCR2B = (1 << CS21)
 *
 *    CS21 = "Clock Select Bit 1 für Timer 2" = Prescaler 8
 *      Identische Wirkung wie CS11 bei Timer 1.
 *
 *  PWM_L_DDR |= (1 << PWM_FL_PIN) | (1 << PWM_RL_PIN):
 *    PB1 und PB2 müssen als Ausgänge konfiguriert sein, damit der
 *    Timer das PWM-Signal tatsächlich auf den Pin ausgeben kann.
 *    Ohne diese Zeile erzeugt der Timer intern PWM, aber nichts kommt
 *    am Pin raus.
 * ──────────────────────────────────────────────────────────────────────── */
static void motor_pwm_init(void) {
  /* Timer 1 – linke Motoren (PB1=OC1A=FL, PB2=OC1B=RL) */
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  PWM_L_DDR |= (1 << PWM_FL_PIN) | (1 << PWM_RL_PIN);

  /* Timer 2 – rechte Motoren (PB3=OC2A=FR, PD3=OC2B=RR) */
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS21);
  PWM_FR_DDR |= (1 << PWM_FR_PIN);
  PWM_RR_DDR |= (1 << PWM_RR_PIN);
}

/* ── motor_init: Alles beim Programmstart vorbereiten ────────────────────
 *
 *  Einmalige Initialisierung aller Motor-Hardware.
 *  Reihenfolge ist wichtig: erst Pins konfigurieren, dann PWM, dann stoppen.
 * ──────────────────────────────────────────────────────────────────────── */
void motor_init(void) {
  FL_DDR |= (1 << FL_IN1_PIN) | (1 << FL_IN2_PIN); /* Vorne links → Ausgänge */
  RL_DDR |= (1 << RL_IN3_PIN) | (1 << RL_IN4_PIN); /* Hinten links → Ausgänge */
  FR_DDR |= (1 << FR_IN1_PIN) | (1 << FR_IN2_PIN); /* Vorne rechts → Ausgänge */
  RR_DDR |= (1 << RR_IN3_PIN) | (1 << RR_IN4_PIN); /* Hinten rechts → Ausgänge */
  _delay_ms(100);                                  /* Hardware stabil werden lassen */
  motor_pwm_init();                                /* Timer konfigurieren */
  motor_stop();                                    /* Sicherer Startzustand */
  motor_set_speed(MOTOR_SPEED_DEFAULT);
}

/* ── Fahrbewegungen ───────────────────────────────────────────────────────
 *
 *  Was sind OCR1A, OCR1B, OCR2A, OCR2B?
 *    OCR = "Output Compare Register"
 *    In dieses Register schreibt man den gewünschten PWM-Wert (0–255).
 *    Der Timer vergleicht seinen aktuellen Zählerstand ständig mit diesem
 *    Wert und schaltet den zugehörigen Pin automatisch um.
 *    OCR1A → steuert OC1A → Pin PB1 → Motor FL (vorne links)
 *    OCR1B → steuert OC1B → Pin PB2 → Motor RL (hinten links)
 *    OCR2A → steuert OC2A → Pin PB3 → Motor FR (vorne rechts)
 *    OCR2B → steuert OC2B → Pin PD3 → Motor RR (hinten rechts)
 * ──────────────────────────────────────────────────────────────────────── */

void motor_forward(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = current_speed; /* FL: volle Geschwindigkeit */
  OCR1B = current_speed; /* RL: volle Geschwindigkeit */
  OCR2A = current_speed; /* FR: volle Geschwindigkeit */
  OCR2B = current_speed; /* RR: volle Geschwindigkeit */
}

void motor_backward(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = current_speed;
  OCR1B = current_speed;
  OCR2A = current_speed;
  OCR2B = current_speed;
}

/* Auf der Stelle links drehen: linke Seite rückwärts, rechte vorwärts */
void motor_turn_left(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = current_speed;
  OCR1B = current_speed;
  OCR2A = current_speed;
  OCR2B = current_speed;
}

/* Auf der Stelle rechts drehen: linke Seite vorwärts, rechte rückwärts */
void motor_turn_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = current_speed;
  OCR1B = current_speed;
  OCR2A = current_speed;
  OCR2B = current_speed;
}

/* Alle Motoren stoppen: Richtungspins LOW, PWM-Wert 0 (kein Signal) */
void motor_stop(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_STOP);
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 0;
  OCR2B = 0;
}

/* Vorwärts-Kurve links: alle Räder vorwärts, linke Seite nur 25% Tempo.
 * "current_speed / 4" = ganzzahlige Division, ergibt ca. 25% des Wertes. */
void motor_curve_left(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = current_speed / 4; /* FL: 25% – zieht nach links */
  OCR1B = current_speed / 4; /* RL: 25% */
  OCR2A = current_speed;     /* FR: 100% */
  OCR2B = current_speed;     /* RR: 100% */
}

/* Vorwärts-Kurve rechts: linke Seite schnell, rechte Seite 25% Tempo */
void motor_curve_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = current_speed;     /* FL: 100% */
  OCR1B = current_speed;     /* RL: 100% */
  OCR2A = current_speed / 4; /* FR: 25% – zieht nach rechts */
  OCR2B = current_speed / 4; /* RR: 25% */
}

/* Rückwärtsbogen nach rechts: nur linke Motoren rückwärts, rechte stehen */
void motor_backward_curve_right(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_STOP);
  OCR1A = current_speed;
  OCR1B = current_speed;
  OCR2A = 0;
  OCR2B = 0;
}

/* Rückwärtsbogen nach links: nur rechte Motoren rückwärts, linke stehen */
void motor_backward_curve_left(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = current_speed;
  OCR2B = current_speed;
}

/* ── motor_set_speed: Globale Geschwindigkeit für alle Motoren ───────────
 *
 *  Speichert den Wert in current_speed (für zukünftige Fahrbefehle)
 *  und schreibt ihn sofort in alle OCR-Register (für laufende Motoren).
 *  Wertebereich: 0 (Stop) bis 255 (Vollgas).
 * ──────────────────────────────────────────────────────────────────────── */
void motor_set_speed(uint8_t speed) {
  current_speed = speed;
  OCR1A         = speed;
  OCR1B         = speed;
  OCR2A         = speed;
  OCR2B         = speed;
}

/* Nur die linke Motorseite auf eine bestimmte Geschwindigkeit setzen */
void motor_set_left_speed(uint8_t speed) {
  OCR1A = speed;
  OCR1B = speed;
}

/* Nur die rechte Motorseite auf eine bestimmte Geschwindigkeit setzen */
void motor_set_right_speed(uint8_t speed) {
  OCR2A = speed;
  OCR2B = speed;
}