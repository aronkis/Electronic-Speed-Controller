#include "./include/includes.h"

#define PWM_IN_MIN 1000
#define PWM_IN_MAX 2000
#define PWM_MIN_VALUE 35
#define PWM_MAX_VALUE 250

byte timer_value,
     motor_state = 0;
int motor_off_counter = 0,
    soft_starter_countdown = 0;
	
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
	//TODO : Measure the PWM starting point
	if (pwm_input > (PWM_IN_MIN + 115))
	{
		motor_off_counter = 0;
		motor_state = 1;
	}	

	if (motor_state)
	{
		soft_starter_countdown = 2000;
		timer_value = PWM_MIN_VALUE;
		set_timer(timer_value);
		
		while (soft_starter_countdown > 500)
		{
			delayMicroseconds(soft_starter_countdown);
			set_next_step();
			current_phase++;
			current_phase %= 6;
			soft_starter_countdown -= 10;
		}
		
		ACSR |= (1 << ACIE); // Enable the analog comparator interrupt
		
		while (motor_state)
		{
			pwm_input   = constrain(pwm_input, PWM_IN_MIN, PWM_IN_MAX);
			timer_value = map(pwm_input, PWM_IN_MIN, PWM_IN_MAX, \
										 PWM_MIN_VALUE, PWM_MAX_VALUE);
			set_timer(timer_value);

			if (pwm_input < (PWM_IN_MIN + 30))
			{
				if (motor_off_counter > 10000)
				{
					motor_state = 0;
					motor_off_counter = 0;
					PORTD &= ~PORTD;
					TCCR1A = 0;
				}
				motor_off_counter++;
			}
		}
	}
}
