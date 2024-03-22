#include "../include/interrupts.h"
#include "../include/functions.h"
#include "../include/serial.h"

//#include <avr/wdt.h>

volatile uint8_t lastPWMPinState = 0;
volatile uint8_t lastTimerState = 0;

volatile uint8_t timerOverflowCounter = 0;
volatile uint8_t PWMAverageCount = 0;
volatile uint8_t updateSpeed = FALSE;
volatile uint8_t timerValue = 0;
volatile uint32_t PWMInput = 0;

// ISR(PCINTx_vect)
// {
//     if (!lastPWMPinState)
//     {
//         lastPWMPinState = 1;
//         timerOverflowCounter = 0;
//         TCCR2B |= SET_BIT(CS20);
//         TCNT2   = 0;
//     }
//     else if (lastPWMPinState)
//     {
//         PWMInput += (TCNT2 + ((timerOverflowCounter * UINT8_MAX) >> 4));
//         PWMAverageCount++;
//         if (PWMAverageCount == PWM_SAMPLES)
//         {
//             PWMInput /= PWM_SAMPLES;
//             PWMInput = constrain(PWMInput, PWM_IN_MIN, PWM_IN_MAX);
//             motorState = (PWMInput > PWM_IN_MIN + 115) ? 1 : 0;
//             timerValue = map(PWMInput, PWM_IN_MIN, PWM_IN_MAX, \
//                                        PWM_MIN_VALUE, PWM_MAX_VALUE);
//             SET_TIMER(timerValue);
//             PWMAverageCount = 0;
//             PWMInput = 0;
//         }
//         TCCR2B = 0;
//         lastPWMPinState = 0;
//     }
// }

// ISR(TIMER2_OVF_vect)
// {
//     CLEAR_INTERRUPT_FLAGS(TIFR2);
//     timerOverflowCounter++;
// }

ISR (TIMER0_OVF_vect) // ZC detection
{
    uart_init(57600);
    uint8_t adcValue;

    ADCSRA &= ~((1 << ADATE) | (1 << ADIE));
    debug_print(zeroCrossPolarity, "BEFORE WHILE\n\r");

    while (!(ADCSRA & (1 << ADIF)));
    debug_print(zeroCrossPolarity, "AFTER WHILE\n\r");


    adcValue = ADCH;

    if (((zeroCrossPolarity == RISING) && (adcValue > ZC_DETECTION_THRESHOLD)) || 
        ((zeroCrossPolarity == FALLING) && (adcValue < ZC_DETECTION_THRESHOLD)))
    {
        //debug_print(zeroCrossPolarity, "IN ADC\n\r");
        uint16_t timeSinceCommutation = TCNT1;
        TCNT1 = COMMUTATION_CORRECTION; //time related page125 of datesheet!!

        filteredTimeSinceCommutation = ((COMMUTATION_TIMING_IIR_COEFF_A * timeSinceCommutation +
                                        COMMUTATION_TIMING_IIR_COEFF_B * filteredTimeSinceCommutation) /
                                        (COMMUTATION_TIMING_IIR_COEFF_A + COMMUTATION_TIMING_IIR_COEFF_B)) / 2; // time related
        OCR1A = filteredTimeSinceCommutation; 

        updateSpeed = TRUE; 
        SET_COMPB_TRIGGER_VALUE(PWM_START_VALUE);

        SET_TIMER1_COMMUTATE_INT;
        CLEAR_INTERRUPT_FLAGS(TIFR1);
        DISABLE_ALL_TIMER0_INTS;
    }
}

ISR(TIMER1_COMPA_vect) // Commutate
{
    uart_init(57600);

    uart_send_string("TIMER1_COMPA\n\r");
    DRIVE_PORT = nextStep;
    TCNT1 = 0;

    CHECK_ZERO_CROSS_POLARITY;

    CLEAR_INTERRUPT_FLAGS(TIFR1);
    OCR1B = ZC_DETECTION_HOLDOFF_TIME;
    SET_TIMER1_HOLDOFF_INT;

    //wdt_reset();
}

ISR(TIMER1_COMPB_vect) // Enable ZC Detection
{
    uart_init(57600);
    uart_send_string("TIMER1_COMPB\n\r");
    CLEAR_INTERRUPT_FLAGS(TIFR0);
    CLEAR_INTERRUPT_FLAGS(TIFR1);
    SET_TIMER0_ZC_DETECTION_INT;
    DISABLE_ALL_TIMER1_INTS;

    ADMUX = ADMUXTable[nextPhase];
    uart_send_string("BEFORE WHILE CB\n\r");

    while (!(ADCSRA & (1 << ADIF)));
    uart_send_string("AFTER WHILE CB\n\r");

    ADCSRA &= CLEAR_BIT(ADIE);
    ADCSRA |= SET_BIT(ADSC) | SET_BIT(ADATE);

    nextPhase++;
    if (nextPhase >= 6)
    {
        nextPhase = 0;
    }
    nextStep = driveTable[nextPhase];
}

// ISR(WDT_vect)
// {
//     CLEAR_REGISTER(DRIVE_PORT);
//     while(1);
// }