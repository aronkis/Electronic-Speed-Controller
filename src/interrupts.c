#include "../include/interrupts.h"
#include "../include/functions.h"

volatile uint8_t lastPWMPinState = 0;
volatile uint8_t lastTimerState = 0;

volatile uint8_t timerOverflowCounter = 0;
volatile uint8_t PWMAverageCount = 0;
volatile uint8_t timerValue = 0;
volatile uint32_t PWMInput = 0;

// ISR(PCINT0_vect)
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

ISR (ANALOG_COMP_vect) // ZC detection
{
    // Q: Filtering
    uint16_t timeSinceCommutation = TCNT1;
    TCNT1 = COMMUTATION_CORRECTION;
    filteredTimeSinceCommutation = (COMMUTATION_TIMING_IIR_COEFF_A * timeSinceCommutation +
                                    COMMUTATION_TIMING_IIR_COEFF_B * filteredTimeSinceCommutation) /
                                    (COMMUTATION_TIMING_IIR_COEFF_A + COMMUTATION_TIMING_IIR_COEFF_B);
    OCR1A = filteredTimeSinceCommutation;    
    
    TIMSK1 = SET_BIT(OCIE1A);
    CLEAR_INTERRUPT_FLAGS(TIFR1);
    DISABLE_ANALOG_COMPARATOR;
}

//TODO: Verify interrupts
ISR(TIMER1_COMPA_vect) // Commutate
{
    setNextStep();~
    TCNT1 = 0;
    CHECK_ZERO_CROSS_POLARITY;

    CLEAR_INTERRUPT_FLAGS(TIFR1);
    OCR1B = ZC_DETECTION_HOLDOFF_TIME;
    SET_TIMER1_HOLDOFF_INT;
}

ISR(TIMER1_COMPB_vect) // Enable ZC Detection
{
    CLEAR_INTERRUPT_FLAGS(TIFR0);
    CLEAR_INTERRUPT_FLAGS(TIFR1);
    ENABLE_ANALOG_COMPARATOR;
    TIMSK1 = 0;

    nextPhase++;
    if (nextPhase >= 6)
    {
        nextPhase = 0;
    }
}

// TODO: TIMER0 to PWM generator --- Check if TCNT0 > OCR0B?
ISR(TIMER0_COMPB_vect)
{
    if (lastTimerState)
    {
        PORTB &= CLEAR_REGISTER(PORTB);
        lastTimerState = 0;
    }
    else
    {
        PORTB |= SET_BIT(currentHighside);
        lastTimerState = 1;
    }
}

ISR(TIMER0_OVF_vect)
{
    lastTimerState = 1;
}