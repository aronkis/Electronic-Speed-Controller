//#include "./include/serial.h"
#include "./include/functions.h"
#include <avr/io.h> 
//#include "./include/interrupts.h"

int main(void)
{
  //  initPorts();
    //initTimers();
    //initComparator();
    //generateTables();
	TCCR1B = SET_BIT(CS11);
    
    DDRB |= (1 << DDB5);
    while (1) {
        PORTB |=  (1 << PB5);   // LED on
        startupDelay(START_UP_DELAY);
        PORTB &= ~(1 << PB5);   // LED off
        startupDelay(START_UP_DELAY);
    }
    // while (1)
    // {
    //     runMotor();
    // }
    return 0;
}
