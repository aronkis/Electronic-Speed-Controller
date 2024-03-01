#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#include <avr/interrupt.h>

#define PWM_IN_MIN 1000
#define PWM_IN_MAX 2000
#define PWM_MIN_VALUE 35
#define PWM_MAX_VALUE 254
#define PWM_SAMPLES 50

extern volatile uint8_t motorState;

ISR(ANALOG_COMP_vect);  // Zero Crossing 
ISR(PCINT0_vect);       // Measure PWM
ISR(TIMER2_OVF_vect);   // PWM time measuring
ISR(TIMER0_COMPB_vect); // Toggle PWM pin
ISR(TIMER0_OVF_vect);   // Check Zero Crossing
ISR(TIMER1_COMPA_vect); // Commutate
ISR(TIMER1_COMPB_vect); // ZC detection enable
ISR(TIMER1_OVF_vect);


#endif