#include "bt.h"
#include <avr/io.h>

void bt_init(void) {
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

uint8_t bt_data_available(void) {
    return (UCSR0A & (1 << RXC0));
}

uint8_t bt_receive(void) {
    while (!(UCSR0A & (1 << RXC0)));
    if (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
        (void)UDR0;
        return 0;
    }
    return UDR0;
}