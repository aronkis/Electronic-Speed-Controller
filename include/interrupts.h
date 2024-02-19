#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

byte last_PWM_state   	  = 0, // 0 PWM low, 1 PWM high
     last_timer_state     = 0; // 0 PIN ON, 1 PIN OFF
static byte current_phase = 0;
static int pwm_input = 1250;
       int timer_overflow_counter = 0;

void set_next_step();

// Used to check the zero crossing (10 times eliminate noise)
ISR(ANALOG_COMP_vect)
{
	// byte count = 0;
	// while (count < 5)
	// {
	// 	if (current_phase & 1)
	// 	{
	// 		if (((ACSR >> ACO) & 1))    // On falling edge the AIN0 > AINx, this means ACO = 1
	// 			count++;
	// 	}
	// 	else
	// 	{
	// 		if (!((ACSR >> ACO) & 1)) // On rising edge the AIN0 < AINx, this means ACO = 0
	// 			count++;
	// 	} 
	// }

	byte count = 0;
	while (count < 5)
	{
		if (current_phase & 1)
		{
			// On falling edge the AIN0 > AINx, this means ACO = 1
			if (!((ACSR >> ACO) & 1))
				count--;
		}
		else 
		{
			// On rising edge the AIN0 < AINx, this means ACO = 0
			if (((ACSR >> ACO) & 1))
				count--;
		}
		count++;
	}

	set_next_step();
	current_phase++;
	current_phase %= 6;
}

// TODO: Filter the PWM signal length
// Used to measure the PWM signal
ISR(PCINT0_vect)
{
	if (last_PWM_state == 0)
	{
		last_PWM_state = 1;
		timer_overflow_counter = 0;
		TCCR2B |= (1 << CS20); // no prescaling
		TCNT2 = 0;
	}
	else if (last_PWM_state == 1)
	{
		pwm_input = TCNT2 + (timer_overflow_counter * 255 / 16);
		TCCR2B = 0;
		last_PWM_state = 0;
	}
}

// Used to count the number of overflows to count more then 255.
ISR(TIMER2_OVF_vect)
{
	timer_overflow_counter++;
}

// Used to toggle the output pins on the high side
ISR(TIMER1_COMPA_vect)
{
	if (last_timer_state)
	{
		PORTB &= ~PORTB;
		last_timer_state = 0;
	}
	else
	{
		PORTB |= (1 << current_highside);
		last_timer_state = 1;
	}
}

// Used to assure that we are in the right period
ISR(TIMER1_OVF_vect)
{
	last_timer_state = 1;
}

#endif // _INTERRUPTS_H_