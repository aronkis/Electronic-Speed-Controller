#include "./include/functions.h"
#include "./include/interrupts.h"
#include "./include/serial.h"
#include <util/delay.h>

int main(void)
{
    uart_init(57600);
    uart_send_string("STARTING\n\r");
    initPorts();
    uart_send_string("INIT_PORTS_DONE\n\r");
    initTimers();
    uart_send_string("INIT_TIMERS_DONE\n\r");
    initComparator();
    uart_send_string("INIT_COMPARATOR_DONE\n\r");
    generateTables();
    startMotor();

    while (1)
    {
        SET_TIMER(PWM_START_VALUE);
    }

    return 0;
}
