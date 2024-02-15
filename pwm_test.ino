#include "./include/functions.h"

#define PWM_IN_MIN 1000
#define PWM_IN_MAX 2000
#define PWM_MIN_VALUE 35
#define PWM_MAX_VALUE 250

volatile byte timer_value;

void setup()
{
	Serial.begin(115200);
	Serial.println("START");
	DDRD  |= (1 << PD2) | (1 << PD3) | (1 << PD4); // Configure pins 2, 3 and 4 as outputs
	PORTD &= 0x00;								  // pins 0 to 7 set to LOW
	DDRB  |= (1 << PB1) | (1 << PB2) | (1 << PB3); // Configure pins 1, 2 and 3 as outputs
	PORTB &= 0x00;								  // B00110001    D9, D10 and D11 to LOW

	// // Timer1 is used to toggle the high side pins
	TCCR1A |= (1 << WGM10);					// PWM, Phase correct, 8-bit
	TCCR1B |= (1 << CS10);					// Set clock source to clkI/O / 1 (no prescaling)
	TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A); // Enable timer1 interrupt overflow
	TIFR1  |= (1 << OCF1A);

	// Timer2 is used to measure the incoming PWM signal
	TCCR2A  = 0;
	TCCR2B  = 0;
	TIMSK2 |= (1 << TOIE2); // Enable timer2 interrupt overflow

	// Analog comparator setting
	ACSR   |= (1 << ACI); // Disable and clear (flag bit) analog comparator interrupt
	// Set D8 (PWM in) to trigger interrupt
	PCICR  |= (1 << PCIE0);	 // enable PCMSK0 scan
	PCMSK0 |= (1 << PCINT0); // Set pin D8 trigger an interrupt on state change.

} // End of setup loop
// steps : AH-BL -> AH-CL -> BH-CL -> BH-AL -> CH-AL -> CH-BL
void loop()
{
	PWM_INPUT   = constrain(PWM_INPUT, PWM_IN_MIN, PWM_IN_MAX);
	timer_value = map(PWM_INPUT, PWM_IN_MIN, PWM_IN_MAX, PWM_MIN_VALUE, PWM_MAX_VALUE);
	set_timer(timer_value);
}

// void setup()
// {
// 	pinMode(LED_BUILTIN, OUTPUT);
// }

// void loop()
// {
// 	digitalWrite (LED_BUILTIN, HIGH);
// 	delay(500);
// 	digitalWrite (LED_BUILTIN, LOW);
// 	delay(500);
// }