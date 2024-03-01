#include "../include/interrupts.h"
#include "../include/functions.h"

volatile uint8_t lastPWMPinState = 0;
volatile uint8_t lastTimerState = 0;

volatile uint8_t timerOverflowCounter = 0;
volatile uint8_t PWMAverageCount = 0;
volatile uint8_t timerValue = 0;
volatile uint32_t PWMInput = 0;


ISR (ANALOG_COMP_vect)
{
    setNextStep();
    currentPhase++;
    if (currentPhase >= 6)
    {
        currentPhase = 0;
    }
}

ISR(PCINT0_vect)
{
    if (!lastPWMPinState)
    {
        lastPWMPinState = 1;
        timerOverflowCounter = 0;
        TCCR2B |= SET_BIT(CS20);
        TCNT2   = 0;
    }
    else if (lastPWMPinState)
    {
        PWMInput += (TCNT2 + ((timerOverflowCounter * UINT8_MAX) >> 4));
        PWMAverageCount++;
        if (PWMAverageCount == PWM_SAMPLES)
        {
            PWMInput /= PWM_SAMPLES;
            PWMInput = constrain(PWMInput, PWM_IN_MIN, PWM_IN_MAX);
            motorState = (PWMInput > PWM_IN_MIN + 115) ? 1 : 0;
            timerValue = map(PWMInput, PWM_IN_MIN, PWM_IN_MAX, \
                                       PWM_MIN_VALUE, PWM_MAX_VALUE);
            SET_TIMER(timerValue);

            PWMAverageCount = 0;
            PWMInput = 0;
        }
        TCCR2B = 0;
        lastPWMPinState = 0;
    }
}

ISR(TIMER2_OVF_vect)
{
    timerOverflowCounter++;
}

ISR(TIMER1_COMPA_vect)
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

ISR (TIMER1_OVF_vect)
{
    lastTimerState = 1;
}