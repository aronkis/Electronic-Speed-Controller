#include "./include/includes.h"

#define PWM_IN_MIN 1000
#define PWM_IN_MAX 2000
#define PWM_MIN_VALUE 35
#define PWM_MAX_VALUE 250

volatile byte timer_value;

void setup()
{
	Serial.begin(115200);
	Serial.println("START");
	// Configure pins 2, 3 and 4 as outputs
	DDRD  |= (1 << PD2) | (1 << PD3) | (1 << PD4); 
	PORTD &= 0x00;								  
	// Configure pins 1, 2 and 3 as outputs
	DDRB  |= (1 << PB1) | (1 << PB2) | (1 << PB3); 
	PORTB &= 0x00;								  

	// Timer1 is used to toggle the high side pins
	TCCR1A |= (1 << WGM10);					// PWM, Phase correct, 8-bit
	TCCR1B |= (1 << CS10);					// Set clock source to clkI/O / 1 (no prescaling)
	TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A); // Enable timer1 interrupt overflow and compare match A interrupt
	TIFR1  |= (1 << OCF1A);

	// Timer2 is used to measure the incoming PWM signal
	TCCR2A  = 0;
	TCCR2B  = 0;
	TIMSK2 |= (1 << TOIE2); // Enable timer2 interrupt overflow

	// Analog comparator setting
	ACSR   |= (1 << ACI); // Clear the analog comparator interrupt

	// Enabling pin change interrupt on PB0
	PCICR  |= (1 << PCIE0);	 // Enable interrupts on PCINT[0:7]
	PCMSK0 |= (1 << PCINT0); 
} 

void loop()
{
	//TODO : Implement a soft start
	ACSR |= (1 << ACIE); // Enable the analog comparator interrupt
	while (1)
	{
		PWM_INPUT   = constrain(PWM_INPUT, PWM_IN_MIN, PWM_IN_MAX);
		timer_value = map(PWM_INPUT, PWM_IN_MIN, PWM_IN_MAX, \
		                             PWM_MIN_VALUE, PWM_MAX_VALUE);
		set_timer(timer_value);
	}
}