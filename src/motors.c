#include "motors.h"
#include <avr/io.h>
#include <util/delay.h>

// Vorne links - 2 Richtungspins
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

// PWM funktioniert nur auf bestimmten Pins die mit speziellen
// Timer-Ausgängen verbunden sind. Diese sind fest vorgegeben:
//   OC1A = Pin 9  (gehört zu Timer 1, Kanal A)
//   OC1B = Pin 10 (gehört zu Timer 1, Kanal B)
//   OC2A = Pin 11 (gehört zu Timer 2, Kanal A)
//   OC2B = Pin 3  (gehört zu Timer 2, Kanal B)
#define PWM_FL_PIN PB1  // Timer 1, Kanal A → Pin 9
#define PWM_RL_PIN PB2  // Timer 1, Kanal B → Pin 10
#define PWM_L_DDR DDRB

#define PWM_FR_PIN PB3  // Timer 2, Kanal A → Pin 11
#define PWM_RR_PIN PD3  // Timer 2, Kanal B → Pin 3
#define PWM_FR_DDR DDRB
#define PWM_RR_DDR DDRD

#define MOTOR_SPEED_DEFAULT 255

typedef enum {
  MOTOR_STOP,
  MOTOR_BACKWARD,
  MOTOR_FORWARD
} MotorDirection;  // 3 Mögliche Zustände des Motors

// static = nur in dieser Datei sichtbar
static uint8_t current_speed = MOTOR_SPEED_DEFAULT;

static void set_left_motors(MotorDirection dir) {
  switch (dir) {
    case MOTOR_STOP:
      FL_PORT &= ~(1 << FL_IN1_PIN);
      FL_PORT &= ~(1 << FL_IN2_PIN);
      RL_PORT &= ~(1 << RL_IN3_PIN);
      RL_PORT &= ~(1 << RL_IN4_PIN);
      break;
    case MOTOR_FORWARD:
      FL_PORT &= ~(1 << FL_IN1_PIN);  // IN1 = LOW
      FL_PORT |= (1 << FL_IN2_PIN);   // IN2 = HIGH → vorwärts
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

// PWM = Pulsweitenmodulation. Der Pin wird sehr schnell ein- und ausgeschaltet.
// Der Motor reagiert auf den Durchnschnitswert
// TCCR1A, TCCR1B = "Timer Counter Control Register" für Timer 1
// COM1A1, COM1B1 = Compare Output Mode - aktiviert das PWM-Signal am Pin
// WGM10, WGM12 = Waveform Generation Mode - wählt PWM-Modus
// CS11 = Clock Select - Prescaler (Vorteiler) für Timer
static void motor_pwm_init(void) {
  // Timer 1 für linke Motoren
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  PWM_L_DDR |= (1 << PWM_FL_PIN) | (1 << PWM_RL_PIN);  // Pins als Ausgang

  // Timer 2 für rechte Motoren
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS21);
  PWM_FR_DDR |= (1 << PWM_FR_PIN);
  PWM_RR_DDR |= (1 << PWM_RR_PIN);
}

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

void motor_forward(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

void motor_backward(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

// Auf der Stelle drehen
void motor_turn_left(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

void motor_turn_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed;
}

void motor_stop(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_STOP);
  OCR1A = OCR1B = OCR2A = OCR2B = 0;
}

void motor_curve_left(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = current_speed / 4;
  OCR2A = OCR2B = current_speed;
}

void motor_curve_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = current_speed;
  OCR2A = OCR2B = current_speed / 4;
}

void motor_backward_curve_right(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_STOP);
  OCR1A = OCR1B = current_speed;
  OCR2A = OCR2B = 0;
}

void motor_backward_curve_left(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = 0;
  OCR2A = OCR2B = current_speed;
}

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