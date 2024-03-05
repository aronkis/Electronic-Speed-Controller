#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <stdint.h>

#define FALSE 0
#define TRUE  (!FALSE)

#define RISING  ((1 << ACIS0) | (1 << ACIS1))
#define FALLING  (1 << ACIS1)

#define ADC_PIN_B 0
#define ADC_PIN_C 1
#define ADC_PIN_A 2

#define AH PB5
#define BH PB4
#define CH PB3
#define AL PB2
#define BL PB1
#define CL PB0

#define PWM_PIN PB5

#define AH_BL ((SET_BIT(AH)) | SET_BIT(BL))
#define AH_CL ((SET_BIT(AH)) | SET_BIT(CL))
#define BH_CL ((SET_BIT(BH)) | SET_BIT(CL))
#define BH_AL ((SET_BIT(BH)) | SET_BIT(AL))
#define CH_AL ((SET_BIT(CH)) | SET_BIT(AL))
#define CH_BL ((SET_BIT(CH)) | SET_BIT(BL))

#define START_UP_COMMS 8
#define NUMBER_OF_STEPS 6
#define START_UP_DELAY 10000
#define PWM_START_VALUE 100
#define PWM_TOP_VALUE 200 // TODO: Test different values... How to calculate this correctly?
#define DELAY_MULTIPLIER 200
#define ZC_DETECTION_HOLDOFF_TIME (filteredTimeSinceCommutation / 2)

#define DRIVE_PORT PORTB
#define DRIVE_REG  DDRB

#define CLEAR_INTERRUPT_FLAGS(reg) (reg = reg)
#define SET_TIMER(timerValue) (OCR0B = timerValue)
#define SET_BIT(bitPos) (1 << bitPos)
#define CLEAR_BIT(bitPos) (~(1 << bitPos))
#define CLEAR_BITS(bitPos1, bitPos2) (~(SET_BIT(bitPos1) | SET_BIT(bitPos2)))
#define CLEAR_REGISTER(reg) (~reg)
#define CHECK_ZERO_CROSS_POLARITY (zeroCrossPolarity = nextPhase & 0x01)
#define SET_TIMER1_HOLDOFF_INT (TIMSK1 = SET_BIT(OCIE1B))
#define SET_TIMER1_COMMUTATE_INT (TIMSK1 = SET_BIT(OCIE1A))
#define DISABLE_ANALOG_COMPARATOR (ACSR &= CLEAR_BIT(ACIE))
#define ENABLE_ANALOG_COMPARATOR (ACSR |= SET_BIT(ACIE))
#define CLEAR_ANALOG_COMPARATOR_INTERRUPT (ACSR |= SET_BIT(ACI))
#define constrain(value, min, max) (value < min ? min : \
                                    value > max ? max : value)
#define map(input, in_min, in_max, out_min, out_max) ( (input - in_min) * (out_max - out_min) \
                                                      /(in_max - in_min) + out_min)

uint8_t startupDelays[START_UP_COMMS];
uint8_t driveTable[NUMBER_OF_STEPS];
uint8_t ComparatorPinTable[NUMBER_OF_STEPS];
uint8_t ComparatorEdgeTable[NUMBER_OF_STEPS];
extern volatile uint8_t updateSpeed;
extern volatile uint8_t currentHighside;
extern volatile uint8_t nextStep;
extern volatile uint8_t nextPhase;
extern volatile uint8_t motorState;
extern volatile uint8_t zeroCrossPolarity;
extern volatile uint16_t filteredTimeSinceCommutation;

void initPorts(void);
void initTimers(void);
void initComparator(void);
void startupDelay(uint16_t time);
void bemfSensing(uint8_t adcPin, uint8_t bemfDirection);
void startMotor(void);
void generateTables(void);
void runMotor(void);

#endif