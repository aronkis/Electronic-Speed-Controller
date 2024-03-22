#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_
#define ADC_MEASURE

#include <stdint.h>

#define SYSTEM_CLOCK_FREQUENCY 2000000 // 16MHz prescaled by 8 so 2MHz
#define BASE_PWM_FREQUENCY 20000
#define PWM_TOP_VALUE (SYSTEM_CLOCK_FREQUENCY / BASE_PWM_FREQUENCY / 2) // 50
#define PWM_MIN_VALUE   22
#define PWM_START_VALUE 32
#define PWM_MAX_VALUE   50
#define DELAY_MULTIPLIER 100

#define FALSE 0
#define TRUE  (!FALSE)

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

// ADC settings
/*  ZC_DETECTION_THRESHOLD = VIN * 255 / VREF; 
    VIN  = VBUS* / 2 = 1395, *  where VBUS is downscaled by VD/LPF
    VIN  = VBUS* / 2 = 698, *  where VBUS is downscaled by VD/LPF
    VREF = VCC*      = 3000; * where VCC is downslaced by a VD/LPF
if VREF = 5000;  ZC_DETECTION_THRESHOLD = VIN * 255 / VREF;
    ZC_DETECTION_THRESHOLD = 71
*/
// TODO: Connect the VREF before testing!
#define ZC_DETECTION_THRESHOLD 54
#define ADC_PIN_B 0x00
#define ADC_PIN_C 0x01
#define ADC_PIN_A 0x02
#define ADC_REF_SELECTION ((0 << REFS1) | (0 << REFS0))
#define ADC_RES_ADJUST (1 << ADLAR)
#define ADC_PRESCALER_16 ((1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0)) // limit the ADC clock to 1 MHz
#define ADC_TRIGGER_SOURCE ((1 << ADTS2) | (0 << ADTS1) | (0 << ADTS0))
#define ADMUX_A (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_PIN_A)
#define ADMUX_B (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_PIN_B)
#define ADMUX_C (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_PIN_C)
#define CHECK_ZERO_CROSS_POLARITY (zeroCrossPolarity = nextPhase & 0x01)

// Comparator settings
#ifdef COMPARATOR_MEASURE
#define RISING  ((1 << ACIS0) | (1 << ACIS1))
#define FALLING  (1 << ACIS1)
#endif

#ifdef ADC_MEASURE
#define RISING 0
#define FALLING 1
#endif 

#define DRIVE_PORT PORTB
#define DRIVE_REG  DDRB

#define START_UP_COMMS 26
#define NUMBER_OF_STEPS 6
#define START_UP_DELAY 10000
#define ZC_DETECTION_HOLDOFF_TIME (filteredTimeSinceCommutation / 2) // time related
#define SET_COMPB_TRIGGER_VALUE(timerValue) (OCR0B = timerValue)


#define CLEAR_INTERRUPT_FLAGS(reg) (reg = reg)
#define SET_BIT(bitPos) (1 << bitPos)
#define CLEAR_BIT(bitPos) (~(1 << bitPos))
#define CLEAR_BITS(bitPos1, bitPos2) (~(SET_BIT(bitPos1) | SET_BIT(bitPos2)))
#define CLEAR_REGISTER(reg) (~reg)
#define SET_TIMER1_HOLDOFF_INT (TIMSK1 = SET_BIT(OCIE1B))
#define SET_TIMER1_COMMUTATE_INT (TIMSK1 = SET_BIT(OCIE1A))
#define DISABLE_ALL_TIMER1_INTS (TIMSK1 = 0)
#define DISABLE_ALL_TIMER0_INTS (TIMSK0 = 0)
#define SET_TIMER0_ZC_DETECTION_INT (TIMSK0 = SET_BIT(TOIE0))
#define constrain(value, min, max) (value < min ? min : \
                                    value > max ? max : value)
#define map(input, in_min, in_max, \
                   out_min, out_max) ( (input - in_min) * (out_max - out_min) \
                                      /(in_max - in_min) + out_min)

uint8_t driveTable[NUMBER_OF_STEPS];
uint8_t ADMUXTable[NUMBER_OF_STEPS];
uint16_t startupDelays[START_UP_COMMS];
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
void initADC(void);
void enableWatchdogTimer(void);
void startupDelay(uint16_t time);
void bemfSensing(uint8_t adcPin, uint8_t bemfDirection);
void startMotor(void);
void generateTables(void);
void runMotor(void);
void writeTimer1(uint16_t value);
uint16_t readTimer1(void);

#endif