// motors.c – Motorsteuerung für 4 DC-Motoren über zwei L298N Treiber
// Steuert Richtung (vorwärts/rückwärts/stop) über digitale Pins
// und Geschwindigkeit über PWM-Signale (Timer 1 und Timer 2)

#include "motors.h"
#include <avr/io.h>
#include <util/delay.h>

// --- Pin-Definitionen ---
// FL = Front Left (vorne links), RL = Rear Left (hinten links)
// FR = Front Right (vorne rechts), RR = Rear Right (hinten rechts)

#define FL_IN1_PIN PD4  // Richtungspin 1 – Motor vorne links
#define FL_IN2_PIN PD5  // Richtungspin 2 – Motor vorne links
#define FL_DDR DDRD
#define FL_PORT PORTD

#define RL_IN3_PIN PD6  // Richtungspin 1 – Motor hinten links
#define RL_IN4_PIN PD7  // Richtungspin 2 – Motor hinten links
#define RL_DDR DDRD
#define RL_PORT PORTD

#define FR_IN1_PIN PC0  // Richtungspin 1 – Motor vorne rechts
#define FR_IN2_PIN PC1  // Richtungspin 2 – Motor vorne rechts
#define FR_DDR DDRC
#define FR_PORT PORTC

#define RR_IN3_PIN PC2  // Richtungspin 1 – Motor hinten rechts
#define RR_IN4_PIN PC3  // Richtungspin 2 – Motor hinten rechts
#define RR_DDR DDRC
#define RR_PORT PORTC

// PWM-Pins – diese steuern die Geschwindigkeit per Timer
#define PWM_FL_PIN PB1  // PWM für vorne links  (Timer 1, OC1A)
#define PWM_RL_PIN PB2  // PWM für hinten links (Timer 1, OC1B)
#define PWM_L_DDR DDRB

#define PWM_FR_PIN PB3  // PWM für vorne rechts  (Timer 2, OC2A)
#define PWM_RR_PIN PD3  // PWM für hinten rechts (Timer 2, OC2B)
#define PWM_FR_DDR DDRB
#define PWM_RR_DDR DDRD

#define MOTOR_SPEED_DEFAULT 255  // Maximale Geschwindigkeit (0–255)

// Enum für die drei möglichen Motorzustände
typedef enum { MOTOR_STOP, MOTOR_BACKWARD, MOTOR_FORWARD } MotorDirection;

// Aktuelle Geschwindigkeit (global für alle Motoren)
static uint8_t current_speed = MOTOR_SPEED_DEFAULT;

// --- Interne Hilfsfunktionen ---

// Setzt die Richtung der LINKEN beiden Motoren (FL + RL)
// Der L298N braucht für jede Richtung eine bestimmte IN1/IN2-Kombination:
//   Vorwärts:  IN1=0, IN2=1
//   Rückwärts: IN1=1, IN2=0
//   Stop:      IN1=0, IN2=0
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
      FL_PORT |= (1 << FL_IN2_PIN);
      RL_PORT &= ~(1 << RL_IN3_PIN);
      RL_PORT |= (1 << RL_IN4_PIN);
      break;
    case MOTOR_BACKWARD:
      FL_PORT |= (1 << FL_IN1_PIN);
      FL_PORT &= ~(1 << FL_IN2_PIN);
      RL_PORT |= (1 << RL_IN3_PIN);
      RL_PORT &= ~(1 << RL_IN4_PIN);
      break;
  }
}

// Setzt die Richtung der RECHTEN beiden Motoren (FR + RR)
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

// Initialisiert die PWM-Timer für die Geschwindigkeitsregelung
// Timer 1 → linke Motoren (OC1A = FL, OC1B = RL)
// Timer 2 → rechte Motoren (OC2A = FR, OC2B = RR)
// PWM-Modus: Phase Correct, 8-Bit (0–255)
static void motor_pwm_init(void) {
  // Timer 1: Phase Correct PWM, Prescaler 8
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  PWM_L_DDR |= (1 << PWM_FL_PIN) | (1 << PWM_RL_PIN);  // PWM-Pins als Ausgang

  // Timer 2: Fast PWM, Prescaler 8
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS21);
  PWM_FR_DDR |= (1 << PWM_FR_PIN);  // PWM-Pin vorne rechts als Ausgang
  PWM_RR_DDR |= (1 << PWM_RR_PIN);  // PWM-Pin hinten rechts als Ausgang
}

// --- Öffentliche Funktionen ---

// Initialisiert alle Motor-Pins und startet PWM
void motor_init(void) {
  // Alle Richtungspins als Ausgang setzen
  FL_DDR |= (1 << FL_IN1_PIN) | (1 << FL_IN2_PIN);
  RL_DDR |= (1 << RL_IN3_PIN) | (1 << RL_IN4_PIN);
  FR_DDR |= (1 << FR_IN1_PIN) | (1 << FR_IN2_PIN);
  RR_DDR |= (1 << RR_IN3_PIN) | (1 << RR_IN4_PIN);

  _delay_ms(100);                        // Kurz warten bis Treiber bereit ist
  motor_pwm_init();                      // PWM starten
  motor_stop();                          // Alle Motoren stoppen
  motor_set_speed(MOTOR_SPEED_DEFAULT);  // Standardgeschwindigkeit setzen
}

// Alle 4 Motoren vorwärts mit aktueller Geschwindigkeit
void motor_forward(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;  // PWM-Wert setzen
}

// Alle 4 Motoren rückwärts mit aktueller Geschwindigkeit
void motor_backward(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

// Auf der Stelle links drehen: linke Motoren rückwärts, rechte vorwärts
void motor_turn_left(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

// Auf der Stelle rechts drehen: linke Motoren vorwärts, rechte rückwärts
void motor_turn_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

// Alle Motoren stoppen (PWM = 0, Richtungspins = LOW)
void motor_stop(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_STOP);
  OCR1A = OCR1B = OCR2A = OCR2B = 0;
}

// Kurve nach links: beide Seiten vorwärts, aber links langsamer (1/4 Speed)
void motor_curve_left(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = current_speed / 4;  // Links langsam
  OCR2A = OCR2B = current_speed;      // Rechts schnell
}

// Kurve nach rechts: beide Seiten vorwärts, aber rechts langsamer (1/4 Speed)
void motor_curve_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = current_speed;      // Links schnell
  OCR2A = OCR2B = current_speed / 4;  // Rechts langsam
}

// Rückwärts-Kurve nach rechts: nur linke Motoren rückwärts, rechte stoppen
void motor_backward_curve_right(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_STOP);
  OCR1A = OCR1B = current_speed;
  OCR2A = OCR2B = 0;
}

// Rückwärts-Kurve nach links: nur rechte Motoren rückwärts, linke stoppen
void motor_backward_curve_left(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = 0;
  OCR2A = OCR2B = current_speed;
}

// Setzt die Geschwindigkeit aller Motoren gleichzeitig (0 = stop, 255 = max)
void motor_set_speed(uint8_t speed) {
  current_speed = speed;
  OCR1A = OCR1B = OCR2A = OCR2B = speed;
}

// Setzt nur die Geschwindigkeit der linken Motoren
void motor_set_left_speed(uint8_t speed) {
  OCR1A = OCR1B = speed;
}

// Setzt nur die Geschwindigkeit der rechten Motoren
void motor_set_right_speed(uint8_t speed) {
  OCR2A = OCR2B = speed;
}