#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

void motor_init(void);
void motor_forward(void);
void motor_backward(void);
void motor_turn_left(void);
void motor_turn_right(void);
void motor_backward_curve_right(void);
void motor_backward_curve_left(void);
void motor_curve_left(void);
void motor_curve_right(void);
void motor_stop(void);
void motor_set_speed(uint8_t speed);
void motor_set_left_speed(uint8_t speed);
void motor_set_right_speed(uint8_t speed);

#endif