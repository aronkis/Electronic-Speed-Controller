#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

volatile byte last_PWM_state = 0;
volatile byte last_timer_state = 0; // 0 PIN ON, 1 PIN OFF
				 byte sequence_step = 0;
volatile int timer_overflow_counter = 0;
volatile unsigned long PWM_INPUT = 1250;

int pos = 0;


ISR (ANALOG_COMP_vect)
{
	byte count = 0;
	while (count < 10)
	{
		if (sequence_step & 1)
			if (ACSR & (1 << ACO))   //ACO = 1 - On falling edge the AIN0 > AINx, this means ACO = 1
				count++;
		else
			if (!(ACSR & (1 << ACO))) //ACO = 0 - On rising edge the AIN0 < AINx, this means ACO = 0
				count++;
	}

	set_next_step();
	sequence_step++;
	sequence_step %= 6;
}

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
		PWM_INPUT = TCNT2 + (timer_overflow_counter * 255 / 16);
		TCCR2B = 0;
		last_PWM_state = 0;
	}
}

ISR(TIMER2_OVF_vect)
{
	timer_overflow_counter++;
}
// TODO: Turning PINs ON and OFF
ISR(TIMER1_COMPA_vect)
{
	if (last_timer_state)
	{
		PORTB &= ~PORTB;
    last_timer_state = 0;
	}
	else
	{
    PORTB |= current_highside;
		last_timer_state = 1;
	}
}

// It is used to assure that we are in the right period
ISR(TIMER1_OVF_vect)
{
	last_timer_state = 1;
}

#endif