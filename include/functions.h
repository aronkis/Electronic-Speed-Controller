#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#define AH_BL 0
#define AH_CL 1
#define BH_CL 2
#define BH_AL 3
#define CH_AL 4
#define CH_BL 5

void set_timer(byte pwm_value)
{
	OCR1A = pwm_value;
}

void set_next_step()
{
	switch (current_phase)
	{
	case AH_BL:
		mostfet_state(A_HIGH_PIN, B_LOW_PIN);
		bemf_sensing(ADC_PIN_C, RISING);
		break;
	case AH_CL:
		mostfet_state(A_HIGH_PIN, C_LOW_PIN);
		bemf_sensing(ADC_PIN_B, FALLING);
		break;
	case BH_CL:
		mostfet_state(B_HIGH_PIN, C_LOW_PIN);
		bemf_sensing(ADC_PIN_A, RISING);
		break;
	case BH_AL:
		mostfet_state(B_HIGH_PIN, A_LOW_PIN);
		bemf_sensing(ADC_PIN_C, FALLING);
		break;
	case CH_AL:
		mostfet_state(C_HIGH_PIN, A_LOW_PIN);
		bemf_sensing(ADC_PIN_B, RISING);
		break;
	case CH_BL:
		mostfet_state(C_HIGH_PIN, B_LOW_PIN);
		bemf_sensing(ADC_PIN_A, FALLING);
		break;
	}
}

#endif // _FUNCTIONS_H_