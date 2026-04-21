/*
 * motors.c – Motorsteuerung für den 4WD-Roboter
 * ===============================================
 *
 * Steuert 4 DC-Motoren über zwei L298N H-Brücken-Treiber.
 *
 * Was ist eine H-Brücke (L298N)?
 *   Ein Mikrocontroller-Pin liefert nur wenige Milliampere – zu wenig für einen Motor.
 *   Der L298N ist ein Leistungsbaustein der zwischen Chip und Motor sitzt: er schaltet
 *   den starken Motorstrom durch, gesteuert von schwachen Signalen des ATmega.
 *   "H-Brücke" = die Schaltungsform erlaubt Strom in beide Richtungen → vor- und rückwärts.
 *
 * Wie wird ein Motor gesteuert?
 *   Zwei Richtungspins (IN1/IN2) bestimmen die Drehrichtung:
 *     IN1=LOW,  IN2=HIGH → vorwärts
 *     IN1=HIGH, IN2=LOW  → rückwärts
 *     IN1=LOW,  IN2=LOW  → stopp
 *   Der Enable-Pin (ENA/ENB) steuert per PWM die Geschwindigkeit.
 *
 * Was ist PWM (Pulsweitenmodulation)?
 *   Der Enable-Pin wird sehr schnell ein/aus geschaltet.
 *   Je länger er pro Zyklus HIGH ist, desto mehr Spannung "fühlt" der Motor.
 *   OCR-Wert 255 = 100% HIGH = Vollgas, 128 = 50% = halbe Kraft, 0 = stopp.
 *   Hardware-PWM läuft automatisch durch den Timer – kein manuelles Schalten nötig.
 *
 * Motorpositionen:
 *   FL = Front Left  (vorne links)   → L298N #1 ENA  → PWM: OC1A (PB1)
 *   RL = Rear Left   (hinten links)  → L298N #1 ENB  → PWM: OC1B (PB2)
 *   FR = Front Right (vorne rechts)  → L298N #2 ENA  → PWM: OC2A (PB3)
 *   RR = Rear Right  (hinten rechts) → L298N #2 ENB  → PWM: OC2B (PD3)
 */

#include "motors.h"
#include <avr/io.h>
#include <util/delay.h>

/* ── Richtungspins ────────────────────────────────────────────────────────────
 * DDR  = Data Direction Register: Bit=1 → Pin ist Ausgang
 * PORT = Ausgabe-Register: Pin auf HIGH oder LOW setzen
 * PD4 = Port D, Pin 4 | PC0 = Port C, Pin 0  usw. */
#define FL_IN1_PIN PD4
#define FL_IN2_PIN PD5
#define FL_DDR DDRD
#define FL_PORT PORTD

#define RL_IN3_PIN PD6
#define RL_IN4_PIN PD7
#define RL_DDR DDRD
#define RL_PORT PORTD

#define FR_IN1_PIN PC0
#define FR_IN2_PIN PC1
#define FR_DDR DDRC
#define FR_PORT PORTC

#define RR_IN3_PIN PC2
#define RR_IN4_PIN PC3
#define RR_DDR DDRC
#define RR_PORT PORTC

/* ── PWM-Pins ─────────────────────────────────────────────────────────────────
 * Hardware-PWM funktioniert nur an Pins die direkt mit dem Timer verbunden sind.
 * Diese Pins heißen OC (Output Compare).
 * Timer 1 → linke Motoren (OC1A=PB1, OC1B=PB2)
 * Timer 2 → rechte Motoren (OC2A=PB3, OC2B=PD3)
 * OCR1A/OCR1B/OCR2A/OCR2B = die Register die den PWM-Wert (0–255) speichern. */
#define PWM_FL_PIN PB1
#define PWM_RL_PIN PB2
#define PWM_L_DDR DDRB

#define PWM_FR_PIN PB3
#define PWM_RR_PIN PD3
#define PWM_FR_DDR DDRB
#define PWM_RR_DDR DDRD

#define MOTOR_SPEED_DEFAULT 255

/* MotorDirection – interner Datentyp für Fahrtrichtung.
 * typedef enum: eigener Typ mit festen erlaubten Werten (lesbarer als 0/1/2).
 * static: nur in dieser Datei sichtbar. */
typedef enum { MOTOR_STOP, MOTOR_BACKWARD, MOTOR_FORWARD } MotorDirection;

static uint8_t current_speed = MOTOR_SPEED_DEFAULT; /* aktuelle Geschwindigkeit (0–255) */

/*
 * set_left_motors / set_right_motors – Richtungspins der Motorseite setzen
 * (static = interne Hilfsfunktionen, von außen nicht aufrufbar)
 *
 * PORT |= (1 << PIN)   → Pin HIGH  (bitweises ODER: setzt genau dieses Bit)
 * PORT &= ~(1 << PIN)  → Pin LOW   (~ invertiert, &= löscht genau dieses Bit)
 */
static void set_left_motors(MotorDirection dir) {
  switch (dir) {
    case MOTOR_STOP:
      FL_PORT &= ~(1 << FL_IN1_PIN);
      FL_PORT &= ~(1 << FL_IN2_PIN);
      RL_PORT &= ~(1 << RL_IN3_PIN);
      RL_PORT &= ~(1 << RL_IN4_PIN);
      break;
    case MOTOR_FORWARD:
      FL_PORT &= ~(1 << FL_IN1_PIN);
      FL_PORT |= (1 << FL_IN2_PIN); /* LOW+HIGH → vorwärts */
      RL_PORT &= ~(1 << RL_IN3_PIN);
      RL_PORT |= (1 << RL_IN4_PIN);
      break;
    case MOTOR_BACKWARD:
      FL_PORT |= (1 << FL_IN1_PIN);
      FL_PORT &= ~(1 << FL_IN2_PIN); /* HIGH+LOW → rückwärts */
      RL_PORT |= (1 << RL_IN3_PIN);
      RL_PORT &= ~(1 << RL_IN4_PIN);
      break;
  }
}

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

/*
 * motor_pwm_init – Timer 1 und Timer 2 für Hardware-PWM konfigurieren
 *
 * Timer 1 (linke Motoren, PB1 + PB2):
 *   TCCR1A: COM1A1+COM1B1 = Non-Inverting PWM-Ausgang (Pin HIGH wenn Zähler < OCR-Wert)
 *           WGM10 = Phase Correct PWM 8-bit (Teil 1): Timer zählt hoch und wieder runter
 *   TCCR1B: WGM12 = Phase Correct PWM (Teil 2) | CS11 = Prescaler 8 (16MHz/8 = 2MHz Takt)
 *
 * Timer 2 (rechte Motoren, PB3 + PD3):
 *   TCCR2A: COM2A1+COM2B1 = PWM-Ausgang | WGM20+WGM21 = Fast PWM 8-bit
 *   TCCR2B: CS21 = Prescaler 8
 *
 * DDR |= PIN: Pin als Ausgang setzen, sonst kann der Timer kein Signal ausgeben.
 */
static void motor_pwm_init(void) {
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  PWM_L_DDR |= (1 << PWM_FL_PIN) | (1 << PWM_RL_PIN);

  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS21);
  PWM_FR_DDR |= (1 << PWM_FR_PIN);
  PWM_RR_DDR |= (1 << PWM_RR_PIN);
}

/* motor_init – Einmalige Hardware-Initialisierung beim Programmstart */
void motor_init(void) {
  FL_DDR |= (1 << FL_IN1_PIN) | (1 << FL_IN2_PIN);
  RL_DDR |= (1 << RL_IN3_PIN) | (1 << RL_IN4_PIN);
  FR_DDR |= (1 << FR_IN1_PIN) | (1 << FR_IN2_PIN);
  RR_DDR |= (1 << RR_IN3_PIN) | (1 << RR_IN4_PIN);
  _delay_ms(100);
  motor_pwm_init();
  motor_stop();
  motor_set_speed(MOTOR_SPEED_DEFAULT);
}

/* ── Fahrbefehle ─────────────────────────────────────────────────────────── */

/* Alle 4 Motoren vorwärts mit gleicher Geschwindigkeit */
void motor_forward(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

/* Alle 4 Motoren rückwärts */
void motor_backward(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

/* Auf der Stelle links drehen: links rückwärts, rechts vorwärts */
void motor_turn_left(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

/* Auf der Stelle rechts drehen: links vorwärts, rechts rückwärts */
void motor_turn_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

/* Alle Motoren stoppen: Richtungspins LOW + PWM auf 0 */
void motor_stop(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_STOP);
  OCR1A = OCR1B = OCR2A = OCR2B = 0;
}

/* Vorwärts-Linkskurve: links 25%, rechts 100% → zieht nach links */
void motor_curve_left(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = current_speed / 4; /* 25% */
  OCR2A = OCR2B = current_speed;     /* 100% */
}

/* Vorwärts-Rechtskurve: links 100%, rechts 25% → zieht nach rechts */
void motor_curve_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = current_speed;     /* 100% */
  OCR2A = OCR2B = current_speed / 4; /* 25% */
}

/* Rückwärtsbogen rechts: links rückwärts, rechts stoppt */
void motor_backward_curve_right(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_STOP);
  OCR1A = OCR1B = current_speed;
  OCR2A = OCR2B = 0;
}

/* Rückwärtsbogen links: rechts rückwärts, links stoppt */
void motor_backward_curve_left(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = 0;
  OCR2A = OCR2B = current_speed;
}

/* Geschwindigkeit für alle Motoren setzen und sofort anwenden (0–255) */
void motor_set_speed(uint8_t speed) {
  current_speed = speed;
  OCR1A = OCR1B = OCR2A = OCR2B = speed;
}

void motor_set_left_speed(uint8_t speed) {
  OCR1A = OCR1B = speed;
}
void motor_set_right_speed(uint8_t speed) {
  OCR2A = OCR2B = speed;
}