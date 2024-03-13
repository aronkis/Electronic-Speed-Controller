#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <stdint.h>

#define FALSE 0
#define TRUE  (!FALSE)

//~#define base_PWM

#define RISING  ((1 << ACIS0) | (1 << ACIS1))
#define FALLING  (1 << ACIS1)

#define ADC_PIN_B 0x00
#define ADC_PIN_C 0x01
#define ADC_PIN_A 0x02

#define AH PB5
#define BH PB4
#define CH PB3
#define AL PB2
#define BL PB1
#define CL PB0

#define PWM_PIN PD5

#define AH_BL ((SET_BIT(AH)) | SET_BIT(BL))
#define AH_CL ((SET_BIT(AH)) | SET_BIT(CL))
#define BH_CL ((SET_BIT(BH)) | SET_BIT(CL))
#define BH_AL ((SET_BIT(BH)) | SET_BIT(AL))
#define CH_AL ((SET_BIT(CH)) | SET_BIT(AL))
#define CH_BL ((SET_BIT(CH)) | SET_BIT(BL))

#define START_UP_COMMS 8
#define NUMBER_OF_STEPS 6
#define START_UP_DELAY 5000
#define PWM_START_VALUE 40
#define PWM_TOP_VALUE 225 // TODO: Test different values... How to calculate this correctly?
#define DELAY_MULTIPLIER 200
#define ZC_DETECTION_HOLDOFF_TIME (filteredTimeSinceCommutation / 2)

/*  ZC_DETECTION_THRESHOLD = VIN * 255 / VREF; 
    VIN  = VBUS* / 2 = 1395, *  where VBUS is downscaled by VD/LPF
    VREF = VCC*      = 3000; * where VCC is downslaced by a VD/LPF
*/
#define ZC_DETECTION_THRESHOLD 119 
#define ADC_REF_SELECTION ((0 << REFS1) | (0 << REFS0))
#define ADC_RES_ADJUST (1 << ADLAR)
#define ADC_PRESCALER_16 ((1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0)) // limit the ADC clock to 1 MHz
#define ADC_TRIGGER_SOURCE ((1 << ADTS2) | (0 ADTS1) | (0 << ADTS0))
#define ADMUX_A (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_PIN_A)
#define ADMUX_B (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_PIN_B)
#define ADMUX_C (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_PIN_C)

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

uint16_t startupDelays[START_UP_COMMS];
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