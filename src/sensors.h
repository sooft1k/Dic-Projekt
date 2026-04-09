#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

void     sensor_init(void);
uint16_t ultrasonic_measure_us(void);
int      ultrasonic_distance_cm(void);
int      read_distance(void);

#endif