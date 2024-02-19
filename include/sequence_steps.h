#ifndef _SEQUENCE_STEPS_H_
#define _SEQUENCE_STEPS_H_

#define RISING  ((1 << ACIS0) | (1 << ACIS1))
#define FALLING  (1 << ACIS1)

#define ADC_PIN_B 0
#define ADC_PIN_C 1
#define ADC_PIN_A 2

#define A_HIGH_PIN PB3
#define B_HIGH_PIN PB2
#define C_HIGH_PIN PB1

#define A_LOW_PIN PD4
#define B_LOW_PIN PD3
#define C_LOW_PIN PD2

static byte current_highside = 0;

void set_up_comparator()
{
	ADCSRA = (0 << ADEN); // Disable the ADC module
	ADCSRB = (1 << ACME); // Enable MUX select for negative input of comparator
}

void mostfet_state (byte high_side, byte low_side)
{
	PORTD &= ~PORTD;
	PORTB &= ~PORTB;
	PORTD |= low_side;
	current_highside = high_side; // Used tell which mosfet is switching
}

void bemf_sensing(byte adc_pin, byte bemf_direction)
{
	ADMUX = adc_pin;
	ACSR &= ~(1 << ACIE);
	ACSR &= ~((1 << ACIS0) | (1 << ACIS1));
	ACSR |= bemf_direction;
	ACSR |= (1 << ACIE);
}

#endif // _SEQUENCE_STEP_H_