#include "../include/serial.h"
#include "../include/functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/wdt.h>

volatile uint8_t currentHighside = 0;
volatile uint8_t nextStep  = 0;
volatile uint8_t nextPhase = 0;
volatile uint8_t motorState = 0;
volatile uint8_t zeroCrossPolarity = 0; // Q: move to register
volatile uint16_t motorTurnOffCounter = 0;
volatile uint16_t filteredTimeSinceCommutation = 0;

void initPorts(void)
{
    DRIVE_REG = SET_BIT(AL) | SET_BIT(BL) | SET_BIT(CL) 
			  | SET_BIT(AH) | SET_BIT(BH) | SET_BIT(CH);
	CLEAR_REGISTER(PORTB);

	DDRD |= SET_BIT(PWM_PIN);

	DIDR0 = SET_BIT(ADC0D) | SET_BIT(ADC1D) | SET_BIT(ADC2D)
		  | SET_BIT(ADC3D) | SET_BIT(ADC4D) | SET_BIT(ADC5D); 
}

void initTimers(void)
{
	// Timer 0 for PWM generation; Phase correct, TOP=OCR0A;
	TCCR0A = SET_BIT(COM0B1) | SET_BIT(WGM00);
	TCCR0B = SET_BIT(WGM02) | SET_BIT(CS10); // Prescaler 8 for 20 kHz PWM
	OCR0A  = PWM_TOP_VALUE;
	CLEAR_INTERRUPT_FLAGS(TIFR0);
	TIMSK0 = (0 << TOIE0);

    // Timer1 for commutation timing
	TCCR1B = (1 << CS11) | (0 << CS10); // Prescaler 8, 2 MHz

    // Timer2 for PWM measuring
    // TCCR2A = 0;
    // TCCR2B = 0;
    // TIMSK2 = SET_BIT(TOIE2);

    // // Enable interrupt on pin change
    // PCICR  = SET_BIT(PCIEx);
    // PCMSK0 = SET_BIT(PCINTx);
}

void initADC(void)
{
	ADCSRA = SET_BIT(ADEN) | SET_BIT(ADATE) | SET_BIT(ADIF) | ADC_PRESCALER_16;
	ADCSRB = ADC_TRIGGER_SOURCE;
}

// TODO: Make it useful only above 8k RPM
void initComparator(void)
{
    //ADCSRA &= CLEAR_BIT(ADEN);
    //ADCSRB  = SET_BIT(ACME);
#ifdef COMPARATOR_MEASURE
    ACSR    = (0 << ACBG) | SET_BIT(ACIE) | SET_BIT(ACI) | SET_BIT(ACIS1) | SET_BIT(ACIS0);
#endif
}

// void enableWatchdogTimer(void)
// {
// 	cli();
// 	wdt_reset();
// 	WDTCSR |= SET_BIT(WDCE) | SET_BIT(WDE);
// 	WDTCSR  = SET_BIT(WDIF) | SET_BIT(WDIE) | SET_BIT(WDE) | SET_BIT(WDP2);
// 	sei();
// }

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
	uint8_t j;
	SET_COMPB_TRIGGER_VALUE(PWM_START_VALUE);

	nextPhase = 0;
	//debug_print(nextPhase, "CURRENTLY_IN_PHASE: ");
	DRIVE_PORT = driveTable[nextPhase];
	startupDelay(START_UP_DELAY);

	nextPhase++;
	nextStep = driveTable[nextPhase];

	for (i = 0; i < START_UP_COMMS; i++)
	{
		//debug_print(nextPhase, "CURRENTLY_IN_PHASE: ");
		DRIVE_PORT = nextStep;
		startupDelay(startupDelays[i]);

		ADMUX = ADMUXTable[nextPhase];
		CHECK_ZERO_CROSS_POLARITY;

		nextPhase++;
		if (nextPhase >= 6)
		{
			nextPhase = 0;
		}
		nextStep = driveTable[nextPhase];
		debug_print(i, "i = ");
		if (i == 25)
		{
			i = 15;
			j++;
			if (j == 100){
				SET_COMPB_TRIGGER_VALUE(32);
				j = 0;
			}
		}
	}

	// Soft start done.
	TCNT1 = 0;
	SET_TIMER1_COMMUTATE_INT;
	filteredTimeSinceCommutation = startupDelays[START_UP_COMMS - 1] * (START_UP_DELAY / 2) / 2; //time related
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

	ADMUXTable[0] = ADC_PIN_C;
	ADMUXTable[1] = ADC_PIN_B;
	ADMUXTable[2] = ADC_PIN_A;
	ADMUXTable[3] = ADC_PIN_C;
	ADMUXTable[4] = ADC_PIN_B;
	ADMUXTable[5] = ADC_PIN_A;

	// For startup
	startupDelays[0] = 200;
  	startupDelays[1] = 150;
  	startupDelays[2] = 100;
  	startupDelays[3] = 80;
  	startupDelays[4] = 70;
  	startupDelays[5] = 65;
  	startupDelays[6] = 60;
  	startupDelays[7] = 55;
  	startupDelays[8] = 50;
  	startupDelays[9] = 45;
  	startupDelays[10] = 40;
  	startupDelays[11] = 35;
  	startupDelays[12] = 30;
  	startupDelays[13] = 30;
  	startupDelays[14] = 30;
  	startupDelays[15] = 1;
  	startupDelays[16] = 1;
  	startupDelays[17] = 1;
  	startupDelays[18] = 1;
  	startupDelays[19] = 1;
  	startupDelays[20] = 1;
  	startupDelays[21] = 1;
  	startupDelays[22] = 1;
  	startupDelays[23] = 1;
  	startupDelays[24] = 1;
  	startupDelays[25] = 1;
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