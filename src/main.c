#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>
#include "motors.h"
#include "sensors.h"
#include "bt.h"

#define OBSTACLE_DISTANCE_CM    20
#define REMOTE_START_BYTE       0xFF
#define REMOTE_TIMEOUT_MS       100
#define REMOTE_THRESHOLD        100

typedef enum { MODE_AUTONOMOUS, MODE_MANUEL } RobotMode;
typedef enum { AUTO_FORWARD, AUTO_REVERSE, AUTO_TURN, AUTO_PAUSE } AutoState;

volatile RobotMode current_mode = MODE_AUTONOMOUS;
static AutoState auto_state = AUTO_FORWARD;
volatile uint16_t state_timer_ms = 0;

void timer0_init(void) {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
    if (state_timer_ms > 0) state_timer_ms--;
}

void toggle_mode(void) {
    if (current_mode == MODE_AUTONOMOUS) {
        current_mode = MODE_MANUEL;
        motor_set_speed(200);
    } else {
        current_mode = MODE_AUTONOMOUS;
        auto_state   = AUTO_FORWARD;
        motor_set_speed(200);
        motor_stop();
    }
}

void check_mode_switch(void) {
    if (!bt_data_available()) return;
    uint8_t b = bt_receive();
    if (b == 0xFE) toggle_mode();
}

static uint8_t remote_receive_timeout(uint8_t *timeout) {
    uint16_t ms = 0;
    while (!bt_data_available()) {
        _delay_ms(1);
        if (++ms >= REMOTE_TIMEOUT_MS) { *timeout = 1; return 0; }
    }
    return bt_receive();
}

void process_remote(void) {
    uint8_t timeout = 0;
    uint16_t sync = 0;

    uint8_t first = remote_receive_timeout(&timeout);
    if (timeout) { motor_stop(); return; }

    if (first == 0xFE) {
        toggle_mode();
        return;
    }

    if (first != REMOTE_START_BYTE) {
        if (++sync > 3) { motor_stop(); return; }
        return;
    }

    uint8_t data[4];
    for (uint8_t i = 0; i < 4; i++) {
        data[i] = remote_receive_timeout(&timeout);
        if (timeout) { motor_stop(); return; }
    }

    int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);

    if (y > REMOTE_THRESHOLD) {
        if      (x >  REMOTE_THRESHOLD) motor_curve_right();
        else if (x < -REMOTE_THRESHOLD) motor_curve_left();
        else                            motor_forward();
    } else if (y < -REMOTE_THRESHOLD) {
        if      (x >  REMOTE_THRESHOLD) motor_backward_curve_right();
        else if (x < -REMOTE_THRESHOLD) motor_backward_curve_left();
        else                            motor_backward();
    } else if (x >  REMOTE_THRESHOLD)  motor_turn_right();
    else if   (x < -REMOTE_THRESHOLD)  motor_turn_left();
    else                               motor_stop();
}

void autonomous_mode(void) {
    check_mode_switch();
    
    uint16_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = state_timer_ms; }
    if (t > 0) return;

    switch (auto_state) {
        case AUTO_FORWARD: {
    int d = read_distance();
    if (d > 0 && d < OBSTACLE_DISTANCE_CM) {
        motor_stop();
        auto_state = AUTO_REVERSE;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { state_timer_ms = 500; }
    } else {
        motor_forward();
    }
    break;
}
        case AUTO_REVERSE:
            motor_backward();
            auto_state = AUTO_TURN;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { state_timer_ms = 2000; }
            break;

        case AUTO_TURN:
            motor_turn_right();
            auto_state = AUTO_PAUSE;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { state_timer_ms = 800; }
            break;

        case AUTO_PAUSE:
            motor_stop();
            auto_state = AUTO_FORWARD;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { state_timer_ms = 200; }
            break;
    }
}

int main(void) {
    motor_init();
    bt_init();
    sensor_init();
    timer0_init();

    sei();
    _delay_ms(1000);
    motor_set_speed(200);

    while (1) {
        check_mode_switch();

        if (current_mode == MODE_AUTONOMOUS)
            autonomous_mode();
        else
            process_remote();
    }
}   