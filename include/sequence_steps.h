#ifndef _SEQUENCE_STEP_H_
#define _SEQUENCE_STEP_H_

#define RISING  ((1 << ACIS0) | (1 << ACIS1))
#define FALLING (~(1 << ACIS0))

#define ADC_PIN_B 0
#define ADC_PIN_C 1
#define ADC_PIN_A 2

#define A_HIGH_PIN PB3
#define B_HIGH_PIN PB2
#define C_HIGH_PIN PB1

#define A_LOW_PIN PD4
#define B_LOW_PIN PD3
#define C_LOW_PIN PD2

volatile byte current_highside = 0;

// TODO: PWM control on High Side
void AH_BL()
{
	PORTD &= ~PORTD;
	PORTD |= B_LOW_PIN;
	current_highside = A_HIGH_PIN;
}

void AH_CL();
void BH_CL();
void BH_AL();
void CH_AL();
void CH_BL();

void set_up_comparator()
{
	ADCSRA = (0 << ADEN); // Disable the ADC module
	ADCSRB = (1 << ACME); // Enable MUX select for negative input of comparator
}

void BEMF_RISING(byte adc_pin)
{
	set_up_comparator();
	ADMUX = adc_pin;
	ACSR |= RISING; // Set up interrupt on rising edge
}

void BEMF_FALLING(byte adc_pin)
{
	set_up_comparator();
	ADMUX = adc_pin;
	ACSR &= FALLING; // Set interrupt on falling edge
}

#endif // _SEQUENCE_STEP_H_