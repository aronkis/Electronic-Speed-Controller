//#include "./include/serial.h"
#include <avr/io.h>
#include <util/delay.h>
#include "./include/functions.h"

int main(void)
{
    DDRB |= (1 << DDB5);
    while (1) {
        PORTB |=  (1 << PB5);   // LED on
        _delay_ms(500);
        PORTB &= ~(1 << PB5);   // LED off
        _delay_ms(500);
    }
    return 0;
}
