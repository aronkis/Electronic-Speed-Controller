#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

void set_timer(byte pwm_value)
{
	OCR1A = pwm_value;
}

void set_next_step()
{
	switch (sequence_step)
	{
	case 0:
		mostfet_state(A_HIGH_PIN, B_LOW_PIN);
		bemf_rising(ADC_PIN_C);
		break;
	case 1:
		mostfet_state(A_HIGH_PIN, C_LOW_PIN);
		bemf_falling(ADC_PIN_B);
		break;
	case 2:
		mostfet_state(B_HIGH_PIN, C_LOW_PIN);
		bemf_rising(ADC_PIN_A);
		break;
	case 3:
		mostfet_state(B_HIGH_PIN, A_LOW_PIN);
		bemf_falling(ADC_PIN_C);
		break;
	case 4:
		mostfet_state(C_HIGH_PIN, A_LOW_PIN);
		bemf_rising(ADC_PIN_B);
		break;
	case 5:
		mostfet_state(C_HIGH_PIN, B_LOW_PIN);
		bemf_falling(ADC_PIN_A);
		break;
	}
}

#endif // _FUNCTIONS_H_