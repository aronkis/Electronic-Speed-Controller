#include "./include/includes.h"

static volatile int motor_off_counter = 0;
static volatile int soft_starter_countdown = 0;
static volatile int motor_state = 0;

void setup()
{
	Serial.begin(115200);
	Serial.println("START");
		
	set_up_ports();

	// // Timer1 is used to toggle the high side pins
	set_up_timer1();

	// // Analog comparator setting
	ACSR = (1 << ACI);

	// Timer2 is used to measure the incoming PWM signal
	set_up_timer2();	
} 

void loop()
{
	
	//TODO : Measure the PWM starting point
	if (pwm_average > (PWM_IN_MIN + 115))
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
			soft_start_next_step();
			current_phase++;
			current_phase %= 6;
			soft_starter_countdown -= 10;
		}
		
		set_up_comparator();

		while (motor_state)
		{
			pwm_average = constrain(pwm_average, PWM_IN_MIN, PWM_IN_MAX);
			timer_value = map(pwm_average, PWM_IN_MIN, PWM_IN_MAX, \
										   PWM_MIN_VALUE, PWM_MAX_VALUE);
			set_timer(timer_value);
		}
	}

}
