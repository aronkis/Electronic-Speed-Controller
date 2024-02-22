#include "./include/includes.h"

int motor_off_counter = 0;
int soft_starter_countdown = 0;
int motor_state = 0;

void setup()
{
	Serial.begin(115200);
	Serial.println("START");
		
	set_up_ports();

	// // Timer1 is used to toggle the high side pins
	set_up_timer1();

	// // Analog comparator setting
	set_up_comparator();

	// Timer2 is used to measure the incoming PWM signal
	set_up_timer2();	
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
		
		set_up_comparator();
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
					ACSR &= ~(1 << ACIE);
				}
				motor_off_counter++;
			}
		}
	}

}
