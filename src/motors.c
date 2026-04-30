#include "motors.h"
#include <avr/io.h>
#include <util/delay.h>

/* ── Richtungspins ──────────────────────────────────────────────────────────
 * Jeder Motor braucht 2 Pins am L298N:
 *   IN1=LOW,  IN2=HIGH → Motor dreht vorwärts
 *   IN1=HIGH, IN2=LOW  → Motor dreht rückwärts
 *   IN1=LOW,  IN2=LOW  → Motor stoppt
 */
#define FL_IN1_PIN PD4 /* Vorne links,  Richtungspin 1 – Port D, Pin 4 */
#define FL_IN2_PIN PD5 /* Vorne links,  Richtungspin 2 – Port D, Pin 5 */
#define FL_DDR DDRD    /* DDR = legt Pin als Ausgang fest */
#define FL_PORT PORTD

#define RL_IN3_PIN PD6 /* Hinten links, Richtungspin 1 – Port D, Pin 6 */
#define RL_IN4_PIN PD7 /* Hinten links, Richtungspin 2 – Port D, Pin 7 */
#define RL_DDR DDRD
#define RL_PORT PORTD

#define FR_IN1_PIN PC0 /* Vorne rechts, Richtungspin 1 – Port C, Pin 0 (A0) */
#define FR_IN2_PIN PC1 /* Vorne rechts, Richtungspin 2 – Port C, Pin 1 (A1) */
#define FR_DDR DDRC
#define FR_PORT PORTC

#define RR_IN3_PIN PC2 /* Hinten rechts, Richtungspin 1 – Port C, Pin 2 (A2) */
#define RR_IN4_PIN PC3 /* Hinten rechts, Richtungspin 2 – Port C, Pin 3 (A3) */
#define RR_DDR DDRC
#define RR_PORT PORTC

/* ── PWM-Pins ───────────────────────────────────────────────────────────────
PWM-Funktion funktioniert nur auf bestimmten Pins,
die mit OC1A, OC1B, OC2A oder OC2B bezeichnet sind.
Diese Pins sind fest vorgegeben.
 */
#define PWM_FL_PIN PB1 /* OC1A = Pin 9  → ENA L298N #1 (linke Motoren) */
#define PWM_RL_PIN PB2 /* OC1B = Pin 10 → ENB L298N #1 */
#define PWM_L_DDR DDRB

#define PWM_FR_PIN PB3 /* OC2A = Pin 11 → ENA L298N #2 (rechte Motoren) */
#define PWM_RR_PIN PD3 /* OC2B = Pin 3  → ENB L298N #2 */
#define PWM_FR_DDR DDRB
#define PWM_RR_DDR DDRD

#define MOTOR_SPEED_DEFAULT 255 /* Startgeschwindigkeit (Maximum = 255) */

typedef enum { MOTOR_STOP, MOTOR_BACKWARD, MOTOR_FORWARD } MotorDirection;

/* Aktuelle Geschwindigkeit für alle Motoren (0–255)
 * static = nur in dieser Datei sichtbar */
static uint8_t current_speed = MOTOR_SPEED_DEFAULT;

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

static void motor_pwm_init(void) {
  /* Timer 1: linke Motoren (Pin 9 = OC1A = FL, Pin 10 = OC1B = RL) */
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); /* PWM aktivieren */
  TCCR1B = (1 << WGM12) | (1 << CS11);                   /* Prescaler 8 */
  PWM_L_DDR |= (1 << PWM_FL_PIN) | (1 << PWM_RL_PIN);    /* Pins als Ausgang */

  /* Timer 2: rechte Motoren (Pin 11 = OC2A = FR, Pin 3 = OC2B = RR) */
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS21);
  PWM_FR_DDR |= (1 << PWM_FR_PIN); /* Pin 11 als Ausgang */
  PWM_RR_DDR |= (1 << PWM_RR_PIN); /* Pin 3  als Ausgang */
}

void motor_init(void) {
  FL_DDR |= (1 << FL_IN1_PIN) | (1 << FL_IN2_PIN); /* FL-Pins → Ausgänge */
  RL_DDR |= (1 << RL_IN3_PIN) | (1 << RL_IN4_PIN); /* RL-Pins → Ausgänge */
  FR_DDR |= (1 << FR_IN1_PIN) | (1 << FR_IN2_PIN); /* FR-Pins → Ausgänge */
  RR_DDR |= (1 << RR_IN3_PIN) | (1 << RR_IN4_PIN); /* RR-Pins → Ausgänge */
  _delay_ms(100);                                  /* Hardware stabilisieren */
  motor_pwm_init();                                /* Timer starten */
  motor_stop();                                    /* Sicherer Startzustand */
  motor_set_speed(MOTOR_SPEED_DEFAULT);            /* Startgeschwindigkeit */
}

/* Alle 4 Motoren vorwärts mit aktueller Geschwindigkeit */
void motor_forward(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = OCR2A = OCR2B = current_speed; /* Alle OCR auf gleichen Wert */
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

/* Alle Motoren stoppen: Richtungspins LOW, PWM auf 0 */
void motor_stop(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_STOP);
  OCR1A = OCR1B = OCR2A = OCR2B = 0; /* PWM auf 0 → kein Strom */
}

/* Vorwärts-Linkskurve: links 25% Tempo, rechts 100% → zieht nach links
 * current_speed / 4 = ganzzahlige Division → ca. 25% */
void motor_curve_left(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = current_speed / 4; /* Links: 25% */
  OCR2A = OCR2B = current_speed;     /* Rechts: 100% */
}

/* Vorwärts-Rechtskurve: links 100%, rechts 25% → zieht nach rechts */
void motor_curve_right(void) {
  set_left_motors(MOTOR_FORWARD);
  set_right_motors(MOTOR_FORWARD);
  OCR1A = OCR1B = current_speed;     /* Links: 100% */
  OCR2A = OCR2B = current_speed / 4; /* Rechts: 25% */
}

/* Rückwärtsbogen nach rechts: links rückwärts, rechts stoppt */
void motor_backward_curve_right(void) {
  set_left_motors(MOTOR_BACKWARD);
  set_right_motors(MOTOR_STOP);
  OCR1A = OCR1B = current_speed;
  OCR2A = OCR2B = 0;
}

/* Rückwärtsbogen nach links: links stoppt, rechts rückwärts */
void motor_backward_curve_left(void) {
  set_left_motors(MOTOR_STOP);
  set_right_motors(MOTOR_BACKWARD);
  OCR1A = OCR1B = 0;
  OCR2A = OCR2B = current_speed;
}

void motor_set_speed(uint8_t speed) {
  current_speed = speed;                 /* Für zukünftige Fahrbefehle speichern */
  OCR1A = OCR1B = OCR2A = OCR2B = speed; /* Sofort auf laufende Motoren anwenden */
}

/* Nur linke Motoren auf bestimmte Geschwindigkeit setzen */
void motor_set_left_speed(uint8_t speed) {
  OCR1A = OCR1B = speed;
}

/* Nur rechte Motoren auf bestimmte Geschwindigkeit setzen */
void motor_set_right_speed(uint8_t speed) {
  OCR2A = OCR2B = speed;
}