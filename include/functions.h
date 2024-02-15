#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include "sequence_steps.h"
#include "interrupts.h"

void set_timer(byte pwm_value)
{
  OCR1A = pwm_value;
}

void set_next_step()
{
  switch (sequence_step)
  {
  case 0:
    AH_BL();
    BEMF_RISING(ADC_PIN_C);
    break;
  case 1:
    AH_CL();
    BEMF_FALLING(ADC_PIN_B);
    break;
  case 2:
    BH_CL();
    BEMF_RISING(ADC_PIN_A);
    break;
  case 3:
    BH_AL();
    BEMF_FALLING(ADC_PIN_C);
    break;
  case 4:
    CH_AL();
    BEMF_RISING(ADC_PIN_B);
    break;
  case 5:
    CH_BL();
    BEMF_FALLING(ADC_PIN_A);
    break;
  }
}

#endif