#include <avr/interrupt.h>
#include "./include/serial.h"

int count = 0;

int main(void)
{
	uart_init(9600,0);

    while (1)
    {
        debug_print(count, "Count == ");
        count++;
    }
}
