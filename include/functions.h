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
#define START_UP_DELAY 10000
#define PWM_START_VALUE 35
#define PWM_TOP_VALUE 200
#define DELAY_MULTIPLIER 200
#define ZC_DETECTION_HOLDOFF_TIME (filteredTimeSinceCommutation / 2)


#define CLEAR_INTERRUPT_FLAGS(reg) (reg = reg)
#define SET_TIMER(timerValue) (OCR1A = timerValue)
#define SET_BIT(bitPos) (1 << bitPos)
#define CLEAR_BIT(bitPos) (~(1 << bitPos))
#define CLEAR_BITS(bitPos1, bitPos2) (~(SET_BIT(bitPos1) | SET_BIT(bitPos2)))
#define CLEAR_REGISTER(reg) (~reg)
#define CHECK_ZERO_CROSS_POLARITY (zeroCrossPolarity = nextPhase & 0x01)
#define SET_TIMER1_HOLDOFF_INT (TIMSK1 = SET_BIT(OCIE1B))
#define SET_TIMER1_COMMUTATE_INT (TIMSK1 = SET_BIT(OCIE1A))
#define DISABLE_ANALOG_COMPARATOR (ACSR &= CLEAR_BIT(ACIE))
#define ENABLE_ANALOG_COMPARATOR (ACSR |= SET_BIT(ACIE))
#define constrain(value, min, max) (value < min ? min : \
                                    value > max ? max : value)
#define map(input, in_min, in_max, out_min, out_max) ( (input - in_min) * (out_max - out_min) \
                                                      /(in_max - in_min) + out_min)

uint8_t startupDelays[START_UP_COMMS];
extern volatile uint8_t currentHighside;
extern volatile uint8_t nextPhase;
extern volatile uint8_t motorState;
extern volatile uint8_t zeroCrossPolarity;
extern volatile uint16_t filteredTimeSinceCommutation;

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