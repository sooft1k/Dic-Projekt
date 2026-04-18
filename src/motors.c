#include "motors.h"
#include <avr/io.h>
#include <util/delay.h>

/* ── Vorne links ─────────────────────────────────────────── */
#define FL_IN1_PIN PD4  // Richtungspin 1 — vorne links
#define FL_IN2_PIN PD5  // Richtungspin 2 — vorne links
#define FL_DDR DDRD     // Port D als Ausgang setzen
#define FL_PORT PORTD   // Port D Ausgabe

/* ── Hinten links ────────────────────────────────────────── */
#define RL_IN3_PIN PD6  // Richtungspin 1 — hinten links
#define RL_IN4_PIN PD7  // Richtungspin 2 — hinten links
#define RL_DDR DDRD
#define RL_PORT PORTD

/* ── Vorne rechts ────────────────────────────────────────── */
#define FR_IN1_PIN PC0  // Richtungspin 1 — vorne rechts
#define FR_IN2_PIN PC1  // Richtungspin 2 — vorne rechts
#define FR_DDR DDRC     // Port C als Ausgang setzen
#define FR_PORT PORTC

/* ── Hinten rechts ───────────────────────────────────────── */
#define RR_IN3_PIN PC2  // Richtungspin 1 — hinten rechts
#define RR_IN4_PIN PC3  // Richtungspin 2 — hinten rechts
#define RR_DDR DDRC
#define RR_PORT PORTC

/* ── PWM-Pins ────────────────────────────────────────────── */
#define PWM_FL_PIN PB1  // Pin 9  → ENA L298N #1 (linke Motoren Geschwindigkeit)
#define PWM_RL_PIN PB2  // Pin 10 → ENB L298N #1
#define PWM_L_DDR DDRB

#define PWM_FR_PIN PB3  // Pin 11 → ENA L298N #2 (rechte Motoren Geschwindigkeit)
#define PWM_RR_PIN PD3  // Pin 3  → ENB L298N #2
#define PWM_FR_DDR DDRB
#define PWM_RR_DDR DDRD

#define MOTOR_SPEED_DEFAULT 255  // Maximale Geschwindigkeit beim Start

typedef enum {
  MOTOR_STOP,
  MOTOR_BACKWARD,
  MOTOR_FORWARD
} MotorDirection;  // Drei mögliche Richtungen

static uint8_t current_speed = MOTOR_SPEED_DEFAULT;  // Aktuelle Geschwindigkeit (0-255)

/* Setzt die Fahrtrichtung der linken zwei Motoren (FL + RL) */
static void set_left_motors(MotorDirection dir) {
  switch (dir) {
    case MOTOR_STOP:  // Beide Pins LOW → Motor stoppt
      FL_PORT &= ~(1 << FL_IN1_PIN);
      FL_PORT &= ~(1 << FL_IN2_PIN);
      RL_PORT &= ~(1 << RL_IN3_PIN);
      RL_PORT &= ~(1 << RL_IN4_PIN);
      break;
    case MOTOR_FORWARD:  // IN1=LOW, IN2=HIGH → Motor vorwärts
      FL_PORT &= ~(1 << FL_IN1_PIN);
      FL_PORT |= (1 << FL_IN2_PIN);
      RL_PORT &= ~(1 << RL_IN3_PIN);
      RL_PORT |= (1 << RL_IN4_PIN);
      break;
    case MOTOR_BACKWARD:  // IN1=HIGH, IN2=LOW → Motor rückwärts
      FL_PORT |= (1 << FL_IN1_PIN);
      FL_PORT &= ~(1 << FL_IN2_PIN);
      RL_PORT |= (1 << RL_IN3_PIN);
      RL_PORT &= ~(1 << RL_IN4_PIN);
      break;
  }
}

/* Setzt die Fahrtrichtung der rechten zwei Motoren (FR + RR) */
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

/* Richtet PWM (Geschwindigkeitssteuerung) für beide Timer ein */
static void motor_pwm_init(void) {
  /* Timer1: steuert linke Motoren (Pin 9 + Pin 10)
     Fast PWM 8-bit → Wert 0-255 bestimmt Einschaltdauer */
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);                 // Prescaler 8
  PWM_L_DDR |= (1 << PWM_FL_PIN) | (1 << PWM_RL_PIN);  // Pins als Ausgang

  /* Timer2: steuert rechte Motoren (Pin 11 + Pin 3)
     Gleicher Modus wie Timer1 */
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS21);  // Prescaler 8
  PWM_FR_DDR |= (1 << PWM_FR_PIN);
  PWM_RR_DDR |= (1 << PWM_RR_PIN);
}

void motor_init(void) {
  FL_DDR |= (1 << FL_IN1_PIN) | (1 << FL_IN2_PIN);  // FL Pins als Ausgang
  RL_DDR |= (1 << RL_IN3_PIN) | (1 << RL_IN4_PIN);  // RL Pins als Ausgang
  FR_DDR |= (1 << FR_IN1_PIN) | (1 << FR_IN2_PIN);  // FR Pins als Ausgang
  RR_DDR |= (1 << RR_IN3_PIN) | (1 << RR_IN4_PIN);  // RR Pins als Ausgang
  _delay_ms(100);                                   // Kurz warten bis Pins stabil sind
  motor_pwm_init();                                 // PWM starten
  motor_stop();                                     // Alle Motoren stoppen
  motor_set_speed(MOTOR_SPEED_DEFAULT);             // Startgeschwindigkeit setzen
}

void motor_forward(void) {
  set_left_motors(MOTOR_FORWARD);   // Linke Motoren vorwärts
  set_right_motors(MOTOR_FORWARD);  // Rechte Motoren vorwärts
  OCR1A = current_speed;
  OCR1B = current_speed;  // Geschwindigkeit links
  OCR2A = current_speed;
  OCR2B = current_speed;  // Geschwindigkeit rechts
}

void motor_backward(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = current_speed;
  OCR1B = current_speed;
  OCR2A = current_speed;
  OCR2B = current_speed;
}

void motor_turn_left(void) {
  set_left_motors(MOTOR_BACKWARD);  // Links rückwärts
  set_right_motors(MOTOR_FORWARD);  // Rechts vorwärts → dreht auf der Stelle links
  OCR1A = current_speed;
  OCR1B = current_speed;
  OCR2A = current_speed;
  OCR2B = current_speed;
}

void motor_turn_right(void) {
  set_left_motors(MOTOR_FORWARD);    // Links vorwärts
  set_right_motors(MOTOR_BACKWARD);  // Rechts rückwärts → dreht auf der Stelle rechts
  OCR1A = current_speed;
  OCR1B = current_speed;
  OCR2A = current_speed;
  OCR2B = current_speed;
}

void motor_stop(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_STOP);
  OCR1A = 0;
  OCR1B = 0;  // PWM auf 0 → kein Strom
  OCR2A = 0;
  OCR2B = 0;
}

void motor_curve_left(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = current_speed / 4;  // Links auf 25% → enge Linkskurve
  OCR1B = current_speed / 4;
  OCR2A = current_speed;  // Rechts voll
  OCR2B = current_speed;
}

void motor_curve_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = current_speed;  // Links voll
  OCR1B = current_speed;
  OCR2A = current_speed / 4;  // Rechts auf 25% → enge Rechtskurve
  OCR2B = current_speed / 4;
}

void motor_backward_curve_right(void) {
  set_left_motors(MOTOR_BACKWARD);  // Links rückwärts
  set_right_motors(MOTOR_STOP);     // Rechts stoppt → Kurve nach rechts rückwärts
  OCR1A = current_speed;
  OCR1B = current_speed;
  OCR2A = 0;
  OCR2B = 0;
}

void motor_backward_curve_left(void) {
  set_left_motors(MOTOR_STOP);       // Links stoppt
  set_right_motors(MOTOR_BACKWARD);  // Rechts rückwärts → Kurve nach links rückwärts
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = current_speed;
  OCR2B = current_speed;
}

void motor_set_speed(uint8_t speed) {
  current_speed = speed;  // Geschwindigkeit speichern
  OCR1A         = speed;
  OCR1B         = speed;  // Sofort anwenden links
  OCR2A         = speed;
  OCR2B         = speed;  // Sofort anwenden rechts
}

void motor_set_left_speed(uint8_t speed) {
  OCR1A = speed;  // Nur linke Motoren anpassen
  OCR1B = speed;
}

void motor_set_right_speed(uint8_t speed) {
  OCR2A = speed;  // Nur rechte Motoren anpassen
  OCR2B = speed;
}