#include "./include/functions.h"
#include "./include/interrupts.h"
#include "./include/serial.h"
#include <util/delay.h>

int main(void)
{
    sei();
    uart_init(57600);
    uart_send_string("STARTING\n\r");
    initPorts();
    uart_send_string("INIT_PORTS_DONE\n\r");
    initTimers();
    uart_send_string("INIT_TIMERS_DONE\n\r");
    //initComparator();
    uart_send_string("INIT_COMPARATOR_DONE\n\r");
    generateTables();
    uart_send_string("TABELES_GENERATED\n\r");
    startMotor();
    uart_send_string("MOTOR_STARTED\n\r");
    while (1)
    {
        // if (updateSpeed)
        // {
        //     SET_TIMER(PWM_START_VALUE);
        // }
        debug_print(nextPhase, "CURRENTLY_IN_PHASE: ");
		DRIVE_PORT = nextStep;
		startupDelay(startupDelays[nextPhase % 6]);

		//bemfSensing(ComparatorPinTable[nextPhase], ComparatorEdgeTable[nextPhase]);
		
		CHECK_ZERO_CROSS_POLARITY;

		nextPhase++;
		if (nextPhase >= 6)
		{
			nextPhase = 0;
		}
		nextStep = driveTable[nextPhase];
    }

    return 0;
}
