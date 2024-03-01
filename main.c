//#include "./include/serial.h"
#include "./include/functions.h"
#include "./include/interrupts.h"

int main(void)
{
    initPorts();
    initTimers();
    initComparator();
    generateTables();

    while (1)
    {
        runMotor();
    }
    return 0;
}
