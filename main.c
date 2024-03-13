//#define DEBUG_RUN
#define NORMAL_RUN
#ifdef NORMAL_RUN
    #include "./include/interrupts.h"
#endif

#ifdef DEBUG_RUN
    #include <avr/io.h>
#endif

#include "./include/functions.h"
#include "./include/serial.h"



int main(void)
{
    uart_init(57600);
    uart_send_string("STARTING\n\r");
    #ifdef NORMAL_RUN
        initPorts();
        //uart_send_string("INIT_PORTS_DONE\n\r");
        initTimers();
        //uart_send_string("INIT_TIMERS_DONE\n\r");
        initComparator();
        //uart_send_string("INIT_COMPARATOR_DONE\n\r");
        generateTables();
        //uart_send_string("TABELES_GENERATED\n\r");
        startupDelay(1000);
        startMotor();
        uart_send_string("MOTOR_STARTED\n\r");
        sei();
        while (1)
        {
            if (updateSpeed)
            {
                SET_TIMER(PWM_START_VALUE);
            }
        }
    #endif

    #ifdef DEBUG_RUN
        DDRB = 0;
        DDRB = (1 << PB5);
        PORTB = 0x00;
        TCCR1B = SET_BIT(CS11);
        while (1)
        {
            uart_send_string("TEST\n\r");
            startupDelay(25000);
            PORTB |= SET_BIT(PB5);
            debug_print(PORTB, "PORTB = ");
            startupDelay(25000);
            PORTB = 0;
            debug_print(PORTB, "PORTB = ");
        }
    #endif

    return 0;
}
