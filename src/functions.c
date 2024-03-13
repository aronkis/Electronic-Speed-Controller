#include "../include/serial.h"
#include "../include/functions.h"
#include <avr/io.h>

#include <avr/interrupt.h>

volatile uint8_t currentHighside = 0;
volatile uint8_t nextStep  = 0;
volatile uint8_t nextPhase = 0;
volatile uint8_t motorState = 0;
volatile uint8_t zeroCrossPolarity = 0; // Q: move to register
volatile uint16_t motorTurnOffCounter = 0;
volatile uint16_t filteredTimeSinceCommutation = 0;

void initPorts(void)
{
    DDRB = SET_BIT(AL) | SET_BIT(BL) | SET_BIT(CL) 
		 | SET_BIT(AH) | SET_BIT(BH) | SET_BIT(CH);
	CLEAR_REGISTER(PORTB);

	DDRD |= SET_BIT(PWM_PIN);
}

void initTimers(void)
{
	// Timer 0 for PWM generation
	TCCR0A = SET_BIT(COM0B1) | SET_BIT(WGM00);
	TCCR0B = SET_BIT(WGM02) | SET_BIT(CS00);
	OCR0A  = PWM_TOP_VALUE;
	CLEAR_INTERRUPT_FLAGS(TIFR0);
	TIMSK0 = (0 << TOIE0);

    // Timer1 for commutation timing
	TCCR1B = SET_BIT(CS11) | (0 << CS10);

    // Timer2 for PWM measuring
    // TCCR2A = 0;
    // TCCR2B = 0;
    // TIMSK2 = SET_BIT(TOIE2);

    // // Enable interrupt on pin change
    // PCICR  = SET_BIT(PCIEx);
    // PCMSK0 = SET_BIT(PCINTx);
}

// TODO: Make it usefull only above 8k RPM
void initComparator(void)
{
    ADCSRA &= CLEAR_BIT(ADEN);
    ADCSRB  = SET_BIT(ACME);
    ACSR    = (0 << ACBG) | SET_BIT(ACIE) | SET_BIT(ACI) | SET_BIT(ACIS1) | SET_BIT(ACIS0);
}

void bemfSensing(uint8_t adcPin, uint8_t bemfDirection)
{
    ADMUX = adcPin;
    ACSR &= CLEAR_BITS(ACIS0, ACIS1);
    ACSR |= bemfDirection;
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
	uint8_t i;

	SET_TIMER(PWM_START_VALUE);

	nextPhase = 0;
	debug_print(nextPhase, "CURRENTLY_IN_PHASE: ");
	DRIVE_PORT = driveTable[nextPhase];
	startupDelay(START_UP_DELAY * 2);

	nextPhase = 1;
	nextStep = driveTable[nextPhase];

	for (i = 0; i < START_UP_COMMS; i++)
	{
		debug_print(nextPhase, "CURRENTLY_IN_PHASE: ");
		DRIVE_PORT = nextStep;
		startupDelay(startupDelays[i] * 2);

	    bemfSensing(ComparatorPinTable[nextPhase], ComparatorEdgeTable[nextPhase]);

		CHECK_ZERO_CROSS_POLARITY;

		nextPhase++;
		if (nextPhase >= 6)
		{
			nextPhase = 0;
		}
		nextStep = driveTable[nextPhase];
	}


	// Soft start done.
	TCNT1 = 0;
	SET_TIMER1_COMMUTATE_INT;
	// CLEAR_INTERRUPT_FLAGS(TIFR1);
    // OCR1B = ZC_DETECTION_HOLDOFF_TIME;
    // SET_TIMER1_HOLDOFF_INT;
	filteredTimeSinceCommutation = startupDelays[START_UP_COMMS - 1] * (START_UP_DELAY / 2) * 2; // too fast?
}

void generateTables(void)
{
	// Mosfet sequence
	driveTable[0] = AH_BL;
	driveTable[1] = AH_CL;
	driveTable[2] = BH_CL;
	driveTable[3] = BH_AL;
	driveTable[4] = CH_AL;
	driveTable[5] = CH_BL; 

	ComparatorEdgeTable[0] = RISING;
	ComparatorEdgeTable[1] = FALLING;
	ComparatorEdgeTable[2] = RISING;
	ComparatorEdgeTable[3] = FALLING;
	ComparatorEdgeTable[4] = RISING;
	ComparatorEdgeTable[5] = FALLING;

/*
	ComparatorPinTable[0] = ADC_PIN_C;
	ComparatorPinTable[1] = ADC_PIN_B;
	ComparatorPinTable[2] = ADC_PIN_A;
	ComparatorPinTable[3] = ADC_PIN_C;
	ComparatorPinTable[4] = ADC_PIN_B;
	ComparatorPinTable[5] = ADC_PIN_A;
*/
	// For startup
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