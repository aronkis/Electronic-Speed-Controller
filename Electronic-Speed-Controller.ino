#define RISING  ((1 << ACIS0) | (1 << ACIS1))
#define FALLING  (1 << ACIS1)
#define PWM_IN_MIN 1000
#define PWM_IN_MAX 2000
#define PWM_MIN_VALUE 35
#define PWM_MAX_VALUE 250


static byte current_highside = PB1;
byte last_PWM_state   	  = 0, // 0 PWM low, 1 PWM high
     last_timer_state     = 0; // 0 PIN ON, 1 PIN OFF
static byte current_phase = 0;
static int pwm_input = 1250;
       int timer_overflow_counter = 0;
byte timer_value;


void set_up_comparator()
{
	ADCSRA = (0 << ADEN); // Disable the ADC module
	ADCSRB = (1 << ACME); // Enable MUX select for negative input of comparator
}

void bemf_sensing(byte adc_pin, byte bemf_direction)
{
	ADMUX = adc_pin;
	ACSR &= ~(1 << ACIE);
	ACSR &= ~((1 << ACIS0) | (1 << ACIS1));
	ACSR |= bemf_direction;
	ACSR |= (1 << ACIE);
}

void set_timer(byte pwm_value)
{
	OCR1A = pwm_value;
}

void mostfet_state (byte high_side, byte low_side)
{
	PORTD &= ~PORTD;
	PORTB &= ~PORTB;
	PORTD |= low_side;
	current_highside = high_side; // Used tell which mosfet is switching
}

	
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

	// // Timer1 is used to toggle the high side pins
	TCCR1A |= (1 << WGM10);					// PWM, Phase correct, 8-bit
	TCCR1B |= (1 << CS10);					// Set clock source to clkI/O / 1 (no prescaling)
	TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A); // Enable timer1 interrupt overflow and compare match A interrupt
	TIFR1  |= (1 << OCF1A);

	// // Analog comparator setting
	ACSR |= (1 << ACI);  // Clear the analog comparator interrupt
	ACSR |= (1 << ACIE); // Enable the analog comparator interrupt
	set_up_comparator();

	// Timer2 is used to measure the incoming PWM signal
	TCCR2A  = 0;
	TCCR2B  = 0;
	TIMSK2 |= (1 << TOIE2); // Enable timer2 interrupt overflow

	// Enabling pin change interrupt on PB0
	PCICR  |= (1 << PCIE0);	 // Enable interrupts on PCINT[0:7]
	PCMSK0 |= (1 << PCINT0); 
} 

void loop()
{
/*
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
*/
	pwm_input   = constrain(pwm_input, PWM_IN_MIN, PWM_IN_MAX);
	timer_value = map(pwm_input, PWM_IN_MIN, PWM_IN_MAX, \
								PWM_MIN_VALUE, PWM_MAX_VALUE);
	set_timer(timer_value);
	// Serial.print("PWM_INPUT = ");
	// Serial.println(pwm_input);
	// Serial.print("OCR1A = ");
	// Serial.println(OCR1A);
}

void set_next_step()
{
	switch (current_phase)
	{
	case 0:
		mostfet_state(PB1, PB3);
		bemf_sensing(0, RISING);
		break;
	case 1:
		mostfet_state(PB2, PB3);
		bemf_sensing(0, FALLING);
		break;
	}
}

ISR(ANALOG_COMP_vect)
{
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
	if (current_phase == 2)
		current_phase = 0;
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
		pwm_input = TCNT2 + (timer_overflow_counter * 255 / 16);
		TCCR2B = 0;
		last_PWM_state = 0;
	}
}

ISR(TIMER2_OVF_vect)
{
	timer_overflow_counter++;
}

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

ISR(TIMER1_OVF_vect)
{
	last_timer_state = 1;
}