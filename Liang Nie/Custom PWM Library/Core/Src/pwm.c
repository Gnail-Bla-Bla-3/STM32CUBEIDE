/*
 * pwm.c
 *
 *  Created on: Apr 24, 2024
 *      Author: liangnie
 */
#include "pwm.h"
#include "main.h"
// #include "stm32f4xx_hal_tim.h"

TIM_HandleTypeDef *tim1 = NULL;
TIM_HandleTypeDef *tim4 = NULL;
TIM_HandleTypeDef *tim5 = NULL;
TIM_HandleTypeDef *tim8 = NULL;

const float PWMPre = 0.000002;
const int8_t PWMµsPre = 2;
const float LEDPre = 0.000001;
const int8_t LEDµsPre = 1;
const float buzzerPre = 0.000001;
const int8_t buzzerµsPre = 1;


// (0-6 = PWM; 7-9 = LED; 10 = Buzzer)
int8_t whichPWMisOn[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int32_t frequency[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t subPeriod[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t period[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// struct individualTracker PWMS[11];

// Initializes the variables in this library :<
void PWMInit (TIM_HandleTypeDef htim1, TIM_HandleTypeDef htim4, TIM_HandleTypeDef htim5, TIM_HandleTypeDef htim8) {
	tim1 = &htim1;
	tim4 = &htim4;
	tim5 = &htim5;
	tim8 = &htim8;
	/*
	for (int i = 0; i < 11; i++) {
		PWMS[i].frequency = 0;
		PWMS[i].period = 0;
		PWMS[i].subPeriod = 0;
	}
	*/
}

int32_t calculateOutputPeriodToGetFrequency (TypesThatUsePWM_t Type, int16_t desiredFrequency) {
	int32_t finalVal = 0;
	switch (Type) {
	case 0:
		finalVal = (1/(desiredFrequency))/PWMPre;
	case 1:
		finalVal = (1/(desiredFrequency))/LEDPre;
	case 2:
		finalVal = (1/(desiredFrequency))/buzzerPre;
	default:
	}
	return finalVal;
}

uint32_t safeOutputPeriodValueCalculator(int32_t maxVal, float ratioVal) {
	// int32_t val = (int32_t)((maxVal)*ratioVal);
	uint32_t val = 500;

	if (val > maxVal) {
		val = maxVal;
	} else if (val < 1) {
		val = 1;
	}
	// val--;
	return val;
}

uint32_t calculateOutputPeriodValue (TypesThatUsePWM_t Type, msOrFullRange microsecondOrFullrange, int8_t position, float val) {
	uint32_t returnVal = 0;
	switch (Type) {
		case 0:
			if (microsecondOrFullrange == MS) {
				returnVal = (uint32_t)(val)/PWMµsPre;
			} else {
				returnVal = safeOutputPeriodValueCalculator(period[position-1], val);
			}
			// PWMS[position-1].period = returnVal;
			break;
		case 1:
			if (microsecondOrFullrange == MS) {
				returnVal = (uint32_t)(val)/LEDµsPre;
			} else {
				returnVal = safeOutputPeriodValueCalculator(period[position+6], val);
			}
			// (int32_t)((float)(period[position+6]) * val)
			// PWMS[position+6].period = returnVal;
			break;
		case 2:
			if (microsecondOrFullrange == MS) {
				returnVal = (uint32_t)(val)/buzzerµsPre;
			} else {
				returnVal = safeOutputPeriodValueCalculator(period[10], val);
			}
			// PWMS[10].period = returnVal;
			break;
		default:
	}
	return returnVal;
}

void PWMInitialize(TypesThatUsePWM_t Type, msOrFullRange microsecondOrFullrange, int8_t position, float val) {
	uint32_t value = calculateOutputPeriodValue (Type, microsecondOrFullrange, position, val);
	switch (Type) {
	case 0:
		switch (position) {
		case 1:
			(*tim1).Instance->CCR1=value;
			break;
		case 2:
			(*tim1).Instance->CCR2=value;
			break;
		case 3:
			(*tim1).Instance->CCR3=value;
			break;
		case 4:
			(*tim1).Instance->CCR4=value;
			break;
		case 5:
			(*tim8).Instance->CCR1=value;
			break;
		case 6:
			(*tim8).Instance->CCR2=value;
			break;
		case 7:
			(*tim8).Instance->CCR3=value;
			break;
		default:
			break;
		}
		subPeriod[position-1] = value;
		break;
	case 1:
		switch (position) {
		case 1:
			(*tim5).Instance->CCR1=value;
			break;
		case 2:
			(*tim5).Instance->CCR2=value;
			break;
		case 3:
			(*tim5).Instance->CCR3=value;
			break;
		default:
			break;
		}
		subPeriod[position+6] = value;
		break;
	case 2:
		(*tim4).Instance->CCR3=value;
		subPeriod[10] = value;
		break;
	default:
	}
}

// htim1.Init.Period
void initializePeriod (TypesThatUsePWM_t Type, int8_t Position, int16_t desiredFrequency) {
	int32_t calculatedPeriod = calculateOutputPeriodToGetFrequency(Type, desiredFrequency);
	switch (Type) {
	case 0:
		if (Position < 5 && Position > 0) {
			(*tim1).Init.Period = calculatedPeriod-1;
		} else {
			(*tim8).Init.Period = calculatedPeriod-1;
		}
		period[Position-1] = calculatedPeriod;
	case 1:
		(*tim5).Init.Period = calculatedPeriod-1;
		period[Position+6] = calculatedPeriod;
	case 2:
		(*tim4).Init.Period = calculatedPeriod-1;
		period[10] = calculatedPeriod;
	default:
	}
}

// (Type, Position, ms or fullrange, val)
void PWMOutput(TypesThatUsePWM_t Type, int8_t Position, int16_t desiredFrequency) {

	initializePeriod(Type, Position, desiredFrequency);


	switch (Type) {
	case 0:

		whichPWMisOn[Position-1] = 1;
	case 1:

		whichPWMisOn[Position+6] = 1;
	case 2:

		whichPWMisOn[10] = 1;
	default:
		break;
	}
	// whichPWMisOn[7]= 1;
}



void PWMTimerStarter() {
	for (int i = 0; i < 11; i++) {
		switch (i) {
		case 0:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_1);
			} else {
				HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_1);
			}
			break;
		case 1:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_2);
			} else {
				HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_2);
			}
			break;
		case 2:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_3);
			} else {
				HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_3);
			}
			break;
		case 3:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_4);
			} else {
				HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_4);
			}
			break;
		case 4:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim8, TIM_CHANNEL_1);
			} else {
				HAL_TIM_PWM_Stop(tim8, TIM_CHANNEL_1);
			}
			break;
		case 5:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim8, TIM_CHANNEL_2);
			} else {
				HAL_TIM_PWM_Stop(tim8, TIM_CHANNEL_2);
			}
			break;
		case 6:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim8, TIM_CHANNEL_3);
			} else {
				HAL_TIM_PWM_Stop(tim8, TIM_CHANNEL_3);
			}
			break;
		case 7:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim5, TIM_CHANNEL_1);
			} else {
				HAL_TIM_PWM_Stop(tim5, TIM_CHANNEL_1);
			}
			break;
		case 8:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim5, TIM_CHANNEL_2);
			} else {
				HAL_TIM_PWM_Stop(tim5, TIM_CHANNEL_2);
			}
			break;
		case 9:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim5, TIM_CHANNEL_3);
			} else {
				HAL_TIM_PWM_Stop(tim5, TIM_CHANNEL_3);
			}
			break;
		case 10:
			if (whichPWMisOn[i] == 1) {
				HAL_TIM_PWM_Start(tim4, TIM_CHANNEL_3);
			} else {
				HAL_TIM_PWM_Stop(tim4, TIM_CHANNEL_3);
			}
			break;
		}
	}
}

