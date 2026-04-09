#ifndef BT_H
#define BT_H

#include <stdint.h>

void    bt_init(void);
uint8_t bt_data_available(void);
uint8_t bt_receive(void);

#endif