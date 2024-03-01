#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <stdint.h>

#define RISING  ((1 << ACIS0) | (1 << ACIS1))
#define FALLING  (1 << ACIS1)

#define ADC_PIN_B 0
#define ADC_PIN_C 1
#define ADC_PIN_A 2

#define AH PB3
#define BH PB2
#define CH PB1

#define AL PD4
#define BL PD3
#define CL PD2

#define AH_BL 0
#define AH_CL 1
#define BH_CL 2
#define BH_AL 3
#define CH_AL 4
#define CH_BL 5

#define START_UP_COMMS 8
#define PWM_START_VALUE 35

#define SET_TIMER(timerValue) (OCR1A = timerValue)
#define SET_BIT(bitPos) (1 << bitPos)
#define CLEAR_BIT(bitPos) (~(1 << bitPos))
#define CLEAR_BITS(bitPos1, bitPos2) (~(SET_BIT(bitPos1) | SET_BIT(bitPos2)))
#define CLEAR_REGISTER(reg) (~reg)
#define constrain(value, min, max) (value < min ? min : \
                                    value > max ? max : value)
#define map(input, in_min, in_max, out_min, out_max) ( (input - in_min) * (out_max - out_min) \
                                                      /(in_max - in_min) + out_min)

uint8_t startupDelays[START_UP_COMMS];
extern volatile uint8_t currentHighside;
extern volatile uint8_t currentPhase;
extern volatile uint8_t motorState;

void initPorts(void);
void initTimers(void);
void initComparator(void);
void startupDelay(uint16_t time);
void mosfetState(uint8_t highSide, uint8_t lowSide);
void bemfSensing(uint8_t adcPin, uint8_t bemfDirection);
void setNextStep(void);
void startMotor(void);
void generateTables(void);
void runMotor(void);

#endif