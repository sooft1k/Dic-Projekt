#include "sensors.h"
#include <avr/io.h>
#include <util/delay.h>

#define TRIG_PIN        PB3
#define TRIG_DDR        DDRB
#define TRIG_PORT       PORTB

#define ECHO_PIN        PB4
#define ECHO_DDR        DDRB
#define ECHO_PORT       PORTB
#define ECHO_PIN_REG    PINB

#define TIMEOUT_US      5800
#define MAX_DISTANCE_CM 100
#define MIN_DISTANCE_CM 2
#define NUM_MEASUREMENTS 1

static uint8_t wait_for_echo_high(uint16_t timeout_us) {
    while (!(ECHO_PIN_REG & (1 << ECHO_PIN))) {
        if (timeout_us == 0) return 0;
        _delay_us(1);
        timeout_us--;
    }
    return 1;
}

void sensor_init(void) {
    TRIG_DDR  |=  (1 << TRIG_PIN);
    TRIG_PORT &= ~(1 << TRIG_PIN);
    ECHO_DDR  &= ~(1 << ECHO_PIN);
    ECHO_PORT &= ~(1 << ECHO_PIN);
    _delay_ms(50);
}

uint16_t ultrasonic_measure_us(void) {
    uint16_t duration_us = 0;

    TRIG_PORT &= ~(1 << TRIG_PIN);
    _delay_us(2);
    TRIG_PORT |=  (1 << TRIG_PIN);
    _delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_PIN);

    if (!wait_for_echo_high(TIMEOUT_US)) return 0;

    while (ECHO_PIN_REG & (1 << ECHO_PIN)) {
        _delay_us(1);
        duration_us++;
        if (duration_us >= TIMEOUT_US) return 0;
    }

    return duration_us;
}

int ultrasonic_distance_cm(void) {
    uint16_t duration_us = ultrasonic_measure_us();
    if (duration_us == 0) return -1;
    int distance_cm = duration_us / 58;
    if (distance_cm < MIN_DISTANCE_CM || distance_cm > MAX_DISTANCE_CM) return -1;
    return distance_cm;
}

int read_distance(void) {
    int valid_count = 0;
    int sum = 0;
    for (uint8_t i = 0; i < NUM_MEASUREMENTS; i++) {
        int dist = ultrasonic_distance_cm();
        if (dist > 0) {
            sum += dist;
            valid_count++;
        }
        _delay_ms(2);
    }
    if (valid_count == 0) return -1;
    return sum / valid_count;
}