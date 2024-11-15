/*
 * pwm.h
 *
 *  Created on: Apr 24, 2024
 *      Author: liangnie
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_
#include "main.h"

struct individualTracker {
	int32_t frequency;
	int32_t subPeriod;
	int32_t period;
};

/*
struct {
	int On;
	int SwitchedOn;
};
*/

typedef enum {
	MS,
	FR,
} msOrFullRange;

typedef enum {
	Motor,
	LED,
	Buzzer,
} TypesThatUsePWM_t;

void PWMInitialize(TypesThatUsePWM_t Type, msOrFullRange microsecondOrFullrange, int8_t position, float val);


// void PWMInit (TIM_HandleTypeDef htim1, TIM_HandleTypeDef htim4, TIM_HandleTypeDef htim5, TIM_HandleTypeDef htim8);

void PWMInit (TIM_HandleTypeDef *t1, TIM_HandleTypeDef *t4, TIM_HandleTypeDef *t5, TIM_HandleTypeDef *t8);

void initializePeriod (TypesThatUsePWM_t Type, int8_t Position, uint32_t desiredFrequency);

uint32_t calculateOutputPeriodValue (TypesThatUsePWM_t Type, msOrFullRange microsecondOrFullrange, int8_t position, float val);

uint32_t safeOutputPeriodValueCalculator(int32_t maxVal, float ratioVal);

void PWMOutput(TypesThatUsePWM_t Type, int8_t Position, uint32_t desiredFrequency);

void PWMTimerStarter();

void mainPrint();

void PWMOff(TypesThatUsePWM_t Type, int8_t Position);
void PWMOn(TypesThatUsePWM_t Type, int8_t Position);

/*
typedef enum {
	PWM1 = "TIM_CHANNEL_1",
	PWM2 = "TIM_CHANNEL_2",
	PWM3 = "TIM_CHANNEL_3",
	PWM4 = "TIM_CHANNEL_4",
	PWM5 = "TIM_CHANNEL_1",
	PWM6 = "TIM_CHANNEL_2",
	PWM7 = "TIM_CHANNEL_3",
} PWMTimerNames;

typedef enum {
	PWM1 = "TIM_CHANNEL_1",
	PWM2 = "TIM_CHANNEL_2",
	PWM3 = "TIM_CHANNEL_3",
} LEDTimerNames;

typedef enum {
	PWM1 = "TIM_CHANNEL_3",
} BuzzerTimerNames;
*/
/*
typedef struct {
	float zeroToOne;

} pwmImportantValues;
*/


#endif /* INC_PWM_H_ */
