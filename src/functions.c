#include "../include/serial.h"
#include "../include/functions.h"
#include <avr/io.h>

volatile uint8_t currentHighside = 0;
volatile uint8_t nextPhase = 0;
volatile uint8_t motorState = 0;
volatile uint8_t zeroCrossPolarity = 0; // Q: move to register
volatile uint16_t motorTurnOffCounter = 0;
volatile uint16_t filteredTimeSinceCommutation = 0;

void initPorts(void)
{
    DDRD = SET_BIT(AL) | SET_BIT(BL) | SET_BIT(CL);
	PORTD = 0x00;
    DDRB = SET_BIT(AH) | SET_BIT(BH) | SET_BIT(CH);
	PORTB = 0x00;
}

void initTimers(void)
{
	// Timer 0 for PWM generation
	TCCR0A = SET_BIT(WGM00);
	TCCR0B = SET_BIT(WGM02) | SET_BIT(CS01);
	OCR0A = PWM_TOP_VALUE;
	CLEAR_INTERRUPT_FLAGS(TIFR0);
	TIMSK0 = SET_BIT(OCIE0B);
	TIFR0 = SET_BIT(OCF0B);

    // Timer1 for commutation timing
	TCCR1B = SET_BIT(CS11);

    // Timer2 for PWM measuring
    // TCCR2A = 0;
    // TCCR2B = 0;
    // TIMSK2 = SET_BIT(TOIE2);

    // // Enable interrupt on pin change
    // PCICR  = SET_BIT(PCIE0);
    // PCMSK0 = SET_BIT(PCINT0);

}

void initComparator(void)
{
    ADCSRA &= CLEAR_BIT(ADEN);
    ADCSRB |= SET_BIT(ACME);
    //ACSR   |= SET_BIT(ACIE) | SET_BIT(ACI) | SET_BIT(ACIS1) | SET_BIT(ACIS0);
	CLEAR_ANALOG_COMPARATOR_INTERRUPT;
}

void mosfetState(uint8_t highSide, uint8_t lowSide)
{
    PORTD &= CLEAR_REGISTER(PORTD);
    PORTB &= CLEAR_REGISTER(PORTB);
    PORTD = SET_BIT(lowSide);
    currentHighside = highSide;
}

void bemfSensing(uint8_t adcPin, uint8_t bemfDirection)
{
    ADMUX = adcPin;
    ACSR &= CLEAR_BITS(ACIS0, ACIS1);
    ACSR |= bemfDirection;
}

void setNextStep(void)
{
    switch (nextPhase)
	{
	case AH_BL:
		mosfetState(AH, BL);
		bemfSensing(ADC_PIN_C, RISING);
		break;
	case AH_CL:
		mosfetState(AH, CL);
		bemfSensing(ADC_PIN_B, FALLING);
		break;
	case BH_CL:
		mosfetState(BH, CL);
		bemfSensing(ADC_PIN_A, RISING);
		break;
	case BH_AL:
		mosfetState(BH, AL);
		bemfSensing(ADC_PIN_C, FALLING);
		break;
	case CH_AL:
		mosfetState(CH, AL);
		bemfSensing(ADC_PIN_B, RISING);
		break;
	case CH_BL:
		mosfetState(CH, BL);
		bemfSensing(ADC_PIN_A, FALLING);
		break;
	}
}

// TODO: fine tuning
void startupDelay(uint16_t time)
{
	CLEAR_INTERRUPT_FLAGS(TIFR1);
	do
	{
		TCNT1 = UINT16_MAX - DELAY_MULTIPLIER;
		// Wait for timer to overflow.
		while (!(TIFR1 & (1 << TOV1)));
		CLEAR_INTERRUPT_FLAGS(TIFR1);
		time--;
	} while (time);
}

void startMotor()
{
    uart_send_string("STARTING_MOTOR\n\r");
	
	uint8_t i;

	SET_TIMER(PWM_START_VALUE);
	nextPhase = 0;
	setNextStep();
	debug_print(nextPhase, "Currently in phase: ");  
	startupDelay(1000);
	nextPhase++;

	for (i = 0; i < START_UP_COMMS; i++)
	{
		setNextStep();
		debug_print(nextPhase, "Currently in phase: ");  
		startupDelay(startupDelays[i]);

		CHECK_ZERO_CROSS_POLARITY;

		nextPhase++;
		if (nextPhase >= 6)
			nextPhase = 0;
	}

	// Soft start done.
	TCNT1 = 0;
	SET_TIMER1_COMMUTATE_INT;

	filteredTimeSinceCommutation = startupDelays[START_UP_COMMS - 1] * (START_UP_DELAY / 2);
    uart_send_string("MOTOR_STARTED\n\r");

}

void generateTables(void)
{
	startupDelays[0] = 200;
	startupDelays[1] = 150;
	startupDelays[2] = 100;
	startupDelays[3] = 80;
	startupDelays[4] = 70;
	startupDelays[5] = 65;
	startupDelays[6] = 60;
	startupDelays[7] = 55;
}

void runMotor(void)
{
	if(motorState)
	{
		motorTurnOffCounter = 0;
		startMotor();
		while (motorState)
		{
			//TODO: motor turn off steps
		}
	}
}