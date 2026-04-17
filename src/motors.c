#include "motors.h"
#include <avr/io.h>
#include <util/delay.h>

/* Vorne links */
#define FL_IN1_PIN      PD4
#define FL_IN2_PIN      PD5
#define FL_DDR          DDRD
#define FL_PORT         PORTD

/* Hinten links */
#define RL_IN3_PIN      PD6
#define RL_IN4_PIN      PD7
#define RL_DDR          DDRD
#define RL_PORT         PORTD

/* Vorne rechts */
#define FR_IN1_PIN      PC0
#define FR_IN2_PIN      PC1
#define FR_DDR          DDRC
#define FR_PORT         PORTC

/* Hinten rechts */
#define RR_IN3_PIN      PC2
#define RR_IN4_PIN      PC3
#define RR_DDR          DDRC
#define RR_PORT         PORTC

/* PWM */
#define PWM_LEFT_PIN    PB1   
#define PWM_RIGHT_PIN   PB2   
#define PWM_DDR         DDRB

#define MOTOR_SPEED_DEFAULT 255 // Standardgeschwindigkeit (0-255)

typedef enum { MOTOR_STOP, MOTOR_BACKWARD, MOTOR_FORWARD } MotorDirection; // Motorrichtungsdefinition

static uint8_t current_speed = MOTOR_SPEED_DEFAULT; // Aktuelle Geschwindigkeit der Motoren (0-255)

static void set_left_motors(MotorDirection dir) { // Steuert die Richtung der linken Motoren
    switch (dir) {
        case MOTOR_STOP:
            FL_PORT &= ~(1 << FL_IN1_PIN);
            FL_PORT &= ~(1 << FL_IN2_PIN);
            RL_PORT &= ~(1 << RL_IN3_PIN);
            RL_PORT &= ~(1 << RL_IN4_PIN);
            break;
        case MOTOR_FORWARD:
            FL_PORT &= ~(1 << FL_IN1_PIN);
            FL_PORT |=  (1 << FL_IN2_PIN);
            RL_PORT &= ~(1 << RL_IN3_PIN);
            RL_PORT |=  (1 << RL_IN4_PIN);
            break;
        case MOTOR_BACKWARD:
            FL_PORT |=  (1 << FL_IN1_PIN);
            FL_PORT &= ~(1 << FL_IN2_PIN);
            RL_PORT |=  (1 << RL_IN3_PIN);
            RL_PORT &= ~(1 << RL_IN4_PIN);
            break;
    }
}

static void set_right_motors(MotorDirection dir) { // Steuert die Richtung der rechten Motoren
    switch (dir) {
        case MOTOR_STOP:
            FR_PORT &= ~(1 << FR_IN1_PIN);
            FR_PORT &= ~(1 << FR_IN2_PIN);
            RR_PORT &= ~(1 << RR_IN3_PIN);
            RR_PORT &= ~(1 << RR_IN4_PIN);
            break;
        case MOTOR_FORWARD:
            FR_PORT &= ~(1 << FR_IN1_PIN);
            FR_PORT |=  (1 << FR_IN2_PIN);
            RR_PORT &= ~(1 << RR_IN3_PIN);
            RR_PORT |=  (1 << RR_IN4_PIN);
            break;
        case MOTOR_BACKWARD:
            FR_PORT |=  (1 << FR_IN1_PIN); 
            FR_PORT &= ~(1 << FR_IN2_PIN); 
            RR_PORT |=  (1 << RR_IN3_PIN); 
            RR_PORT &= ~(1 << RR_IN4_PIN); 
            break;
    }
}

static void motor_pwm_init(void) { 
    TCCR1A = (1 << COM1A1) |
             (1 << COM1B1) |
             (1 << WGM10);

    TCCR1B = (1 << WGM12) |
             (1 << CS11);

    PWM_DDR |= (1 << PWM_LEFT_PIN) | (1 << PWM_RIGHT_PIN);
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
    OCR1A = current_speed;
    OCR1B = current_speed;
}

void motor_backward(void) {
    set_left_motors(MOTOR_BACKWARD);
    set_right_motors(MOTOR_BACKWARD);
    OCR1A = current_speed;
    OCR1B = current_speed;
}

void motor_turn_left(void) {
    set_left_motors(MOTOR_BACKWARD);
    set_right_motors(MOTOR_FORWARD);
    OCR1A = current_speed;
    OCR1B = current_speed;
}

void motor_turn_right(void) {
    set_left_motors(MOTOR_FORWARD);
    set_right_motors(MOTOR_BACKWARD);
    OCR1A = current_speed;
    OCR1B = current_speed;
}

void motor_stop(void) {
    set_left_motors(MOTOR_STOP);
    set_right_motors(MOTOR_STOP);
}


void motor_curve_left(void) {
    set_left_motors(MOTOR_FORWARD);
    set_right_motors(MOTOR_FORWARD);
    OCR1A = current_speed / 2;   /* links langsamer */
    OCR1B = current_speed;
}

void motor_curve_right(void) {
    set_left_motors(MOTOR_FORWARD);
    set_right_motors(MOTOR_FORWARD);
    OCR1A = current_speed;
    OCR1B = current_speed / 2;   /* rechts langsamer */
}

void motor_backward_curve_right(void) {
    set_left_motors(MOTOR_BACKWARD);
    set_right_motors(MOTOR_STOP);
    OCR1A = current_speed;
    OCR1B = 0;
}

void motor_backward_curve_left(void) {
    set_left_motors(MOTOR_STOP);
    set_right_motors(MOTOR_BACKWARD);
    OCR1A = 0;
    OCR1B = current_speed;
}

void motor_set_speed(uint8_t speed) {
    current_speed = speed;
    OCR1A = speed;
    OCR1B = speed;
}

void motor_set_left_speed(uint8_t speed) {
    OCR1A = speed;
}

void motor_set_right_speed(uint8_t speed) {
    OCR1B = speed;
}