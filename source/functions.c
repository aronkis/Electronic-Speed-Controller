#include "../include/functions.h"
#include <avr/io.h>

volatile uint8_t currentHighside = 0;
volatile uint8_t currentPhase = 0;
volatile uint8_t motorState = 0;
volatile uint16_t motorTurnOffCounter = 0;

void initPorts(void)
{
    DDRD = SET_BIT(AL) | SET_BIT(BL) | SET_BIT(CL);
    DDRB = SET_BIT(AH) | SET_BIT(BH) | SET_BIT(CH);
}

void initTimers(void)
{
    // Timer1 for commutation timing
    TCCR1A = SET_BIT(WGM10);
    TCCR1B = SET_BIT(CS10);
    TIMSK1 = SET_BIT(TOIE1) | SET_BIT(OCIE1A);
    TIFR1  = SET_BIT(OCF1A);

    // Timer2 for PWM measuring
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = SET_BIT(TOIE2);

    // Enable interrupt on pin change
    PCICR  = SET_BIT(PCIE0);
    PCMSK0 = SET_BIT(PCINT0);

    //TODO: TIMER0 for after commutation delay.
}

void initComparator(void)
{
    ADCSRA &= CLEAR_BIT(ADEN);
    ADCSRB |= SET_BIT(ACME);
    ACSR   |= SET_BIT(ACIE) | SET_BIT(ACI);
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
    switch (currentPhase)
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
  TIFR1 = TIFR1; // clear interrupts
  do
  {
    TCNT1 = 0xffff - 100;
    // Wait for timer to overflow.
    while (!(TIFR1 & (1 << TOV1)))
    {

    }
    TIFR1 = TIFR1;
    time--;
  } while (time);
}

void startMotor()
{
	unsigned char i;

	SET_TIMER(PWM_START_VALUE);
	currentPhase = 0;

	setNextStep();

	startupDelay(1000);

	currentPhase++;

	for (i = 0; i < START_UP_COMMS; i++)
	{
		setNextStep();
		startupDelay(startupDelays[i]);

		currentPhase++;
		if (currentPhase >= 6)
			currentPhase = 0;
	}
	initComparator();
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
