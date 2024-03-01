#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#include <avr/interrupt.h>

#define PWM_IN_MIN 1000
#define PWM_IN_MAX 2000
#define PWM_MIN_VALUE 35
#define PWM_MAX_VALUE 254
#define PWM_SAMPLES 50

extern volatile uint8_t motorState;

ISR(ANALOG_COMP_vect);
ISR(PCINT0_vect);
ISR(TIMER2_OVF_vect);
ISR(TIMER1_COMPA_vect);
ISR(TIMER1_OVF_vect);


#endif