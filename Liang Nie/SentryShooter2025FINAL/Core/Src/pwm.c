/*
 * pwm.c
 *
 *  Created on: Apr 24, 2024
 *      Author: liangnie
 */
#include "pwm.h"
#include "main.h"
// #include "UART.h"
#include <math.h>
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

uint32_t frequency[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t subPeriod[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t period[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// struct individualTracker PWMS[11];

// Initializes the variables in this library :<
void PWMInit (TIM_HandleTypeDef *t1, TIM_HandleTypeDef *t4, TIM_HandleTypeDef *t5, TIM_HandleTypeDef *t8) {
	tim1 = t1;
	tim4 = t4;
	tim5 = t5;
	tim8 = t8;
	HAL_TIM_Base_Start(tim1);
	HAL_TIM_Base_Start(tim4);
	HAL_TIM_Base_Start(tim5);
	HAL_TIM_Base_Start(tim8);
}

uint32_t calculateOutputPeriodToGetFrequency (TypesThatUsePWM_t Type, uint32_t desiredFrequency) {
	uint32_t finalVal = 0;
	switch (Type) {
	case 0:
		finalVal = 1/(PWMPre*desiredFrequency);
		// usart_printf("%d %d\r\n", desiredFrequency, finalVal);
		break;
	case 1:
		finalVal = 1/(LEDPre*desiredFrequency);
		// usart_printf("%d %d\r\n", desiredFrequency, finalVal);
		break;
	case 2:
		finalVal = 1/(buzzerPre*desiredFrequency);
		// usart_printf("%d %d\r\n", desiredFrequency, finalVal);
		break;
	}

	return finalVal;
}

uint32_t safeOutputPeriodValueCalculator(int32_t maxVal, float ratioVal) {
	// float val = (maxVal);
	// uint32_t val = maxVal;
	// uint32_t val = 500;

	uint32_t val1 = maxVal * ratioVal;// (uint32_t)val;

	if (val1 > (uint32_t)maxVal) {
		val1 = (uint32_t)(maxVal);
	} else if (val1 < 1) {
		val1 = 1;
	}

	// val1 = val1-1;
	return val1;

}

uint32_t calculateOutputPeriodValue (TypesThatUsePWM_t Type, msOrFullRange microsecondOrFullrange, int8_t position, float val) {
	uint32_t returnVal = 0;
	// usart_printf("beanis1 %d", position);
	switch (Type) {
		case 0:
			if (microsecondOrFullrange == MS) {
				returnVal = (uint32_t)(val)/PWMµsPre;
			} else {
				returnVal = safeOutputPeriodValueCalculator(period[position-1], val);
			}
			// usart_printf("beanis %d %d\r\n", returnVal, period[position-1]);
			// PWMS[position-1].period = returnVal;
			break;
		case 1:
			if (microsecondOrFullrange == MS) {
				returnVal = (uint32_t)(val)/LEDµsPre;
			} else {
				returnVal = safeOutputPeriodValueCalculator(period[position+6], val);
			}
			// usart_printf("%d %d\r\n", returnVal, period[position+6]);
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
			// usart_printf("%d %d\r\n", returnVal, period[10]);
			break;
		default:
	}
	return returnVal;
}

void PWMInitialize(TypesThatUsePWM_t Type, msOrFullRange microsecondOrFullrange, int8_t position, float val) {

	// usart_printf("beanis2 %d\r\n", position);

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
	}
	return;
}

void MotorPositionForInitializingPeriod(int8_t Position, uint32_t calculatedPeriod) {
	if (Position < 5) {

		(*tim1).Instance->ARR = calculatedPeriod;
		// (*tim1).Init.Period = calculatedPeriod;
		period[Position-1] = calculatedPeriod;

	} else {
		(*tim8).Instance->ARR = calculatedPeriod;
		period[Position-1] = calculatedPeriod;
	}

}

// htim1.Init.Period
void initializePeriod (TypesThatUsePWM_t Type, int8_t Position, uint32_t desiredFrequency) {
	uint32_t calculatedPeriod = calculateOutputPeriodToGetFrequency(Type, desiredFrequency);
	// usart_printf("%d %d\r\n", Position ,calculatedPeriod);
	switch (Type) {
	case 0:
		MotorPositionForInitializingPeriod(Position, calculatedPeriod);

		break;
	case 1:
		(*tim5).Instance->ARR = calculatedPeriod;

		period[Position+6] = calculatedPeriod;
		// usart_printf("%d %d\r\n", Position ,calculatedPeriod);
		break;
	case 2:
		// (*tim4).Init.Period = calculatedPeriod;
		(*tim4).Instance->ARR = calculatedPeriod;
		period[10] = calculatedPeriod;
		break;
	}
	return;
}

// (Type, Position, ms or fullrange, val)
void PWMOutput(TypesThatUsePWM_t Type, int8_t Position, uint32_t desiredFrequency) {

	initializePeriod(Type, Position, desiredFrequency);

	// usart_printf("beanis3 %d\r\n", Position);
	switch (Type) {
	case 0:
		whichPWMisOn[Position-1] = 1;
		// usart_printf("PWM %d\r\n", Position);
		return;
	case 1:
		whichPWMisOn[Position+6] = 1;
		// usart_printf("LED %d\r\n", Position);
		return;
	case 2:
		whichPWMisOn[10] = 1;
		// usart_printf("Buzzer %d\r\n", Position);
		return;
	}
	// whichPWMisOn[7]= 1;
	// usart_printf("beanis7 %d %d\r\n", Position, whichPWMisOn[Position-1]);
	// return;
}

void PWMOn(TypesThatUsePWM_t Type, int8_t Position) {
	switch (Type) {
	case 0:
		whichPWMisOn[Position-1] = 1;
		// usart_printf("PWM %d\r\n", Position);
		return;
	case 1:
		whichPWMisOn[Position+6] = 1;
		// usart_printf("LED %d\r\n", Position);
		return;
	case 2:
		whichPWMisOn[10] = 1;
		// usart_printf("Buzzer %d\r\n", Position);
		return;
	}
}

void PWMOff(TypesThatUsePWM_t Type, int8_t Position) {
	switch (Type) {
	case 0:
		whichPWMisOn[Position-1] = 0;
		// usart_printf("PWM %d\r\n", Position);
		return;
	case 1:
		whichPWMisOn[Position+6] = 0;
		// usart_printf("LED %d\r\n", Position);
		return;
	case 2:
		whichPWMisOn[10] = 0;
		// usart_printf("Buzzer %d\r\n", Position);
		return;
	}
}

/*
int8_t whichPWMisOn[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int32_t frequency[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t subPeriod[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t period[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
*/

void mainPrint() {

	// usart_printf("|  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |\r\n", whichPWMisOn[0], whichPWMisOn[1], whichPWMisOn[2], whichPWMisOn[3], whichPWMisOn[4], whichPWMisOn[5], whichPWMisOn[6], whichPWMisOn[7], whichPWMisOn[8], whichPWMisOn[9], whichPWMisOn[10]);
	/*
	usart_printf("-----------------------------MAIN PWM INFORMATION-------------------------------\r\n");
	usart_printf("          |                  Motors                 |       LED       | Buzzer |\r\n");
	usart_printf("          |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  1  |  2  |  3  |   1    |\r\n");
	*/
	// usart_printf("Is it on? |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |  ‰d  |\r\n", whichPWMisOn[0], whichPWMisOn[1], whichPWMisOn[2], whichPWMisOn[3], whichPWMisOn[4], whichPWMisOn[5], whichPWMisOn[6], whichPWMisOn[7], whichPWMisOn[8], whichPWMisOn[9], whichPWMisOn[10]);

	// usart_printf("||| %d | %d | %d \r\n", whichPWMisOn[10], period[10], subPeriod[10]);
}

void PWMTimerStarter() {
	for (int i = 0; i < 11; i++) {
		switch (i) {
		case 0:
			if (whichPWMisOn[0] == 1) {
				HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_1);
			} else {
				HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_1);
			}
			break;
		case 1:
			if (whichPWMisOn[1] == 1) {
				HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_2);
			} else {
				HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_2);
			}
			break;
		case 2:
			if (whichPWMisOn[2] == 1) {
				HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_3);
			} else {
				HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_3);
			}
			break;
		case 3:
			if (whichPWMisOn[3] == 1) {
				HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_4);
			} else {
				HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_4);
			}
			break;
		case 4:
			if (whichPWMisOn[4] == 1) {
				HAL_TIM_PWM_Start(tim8, TIM_CHANNEL_1);
			} else {
				HAL_TIM_PWM_Stop(tim8, TIM_CHANNEL_1);
			}
			break;
		case 5:
			if (whichPWMisOn[5] == 1) {
				HAL_TIM_PWM_Start(tim8, TIM_CHANNEL_2);
			} else {
				HAL_TIM_PWM_Stop(tim8, TIM_CHANNEL_2);
			}
			break;
		case 6:
			if (whichPWMisOn[6] == 1) {
				HAL_TIM_PWM_Start(tim8, TIM_CHANNEL_3);
			} else {
				HAL_TIM_PWM_Stop(tim8, TIM_CHANNEL_3);
			}
			break;
		case 7:
			if (whichPWMisOn[7] == 1) {
				HAL_TIM_PWM_Start(tim5, TIM_CHANNEL_1);
			} else {
				HAL_TIM_PWM_Stop(tim5, TIM_CHANNEL_1);
			}
			break;
		case 8:
			if (whichPWMisOn[8] == 1) {
				HAL_TIM_PWM_Start(tim5, TIM_CHANNEL_2);

			} else {
				HAL_TIM_PWM_Stop(tim5, TIM_CHANNEL_2);
				// usart_printf("beanis\r\n");
			}
			break;
		case 9:
			if (whichPWMisOn[9] == 1) {
				HAL_TIM_PWM_Start(tim5, TIM_CHANNEL_3);
			} else {
				HAL_TIM_PWM_Stop(tim5, TIM_CHANNEL_3);
			}
			// usart_printf("Returned1\r\n");
			break;
		case 10:
			if (whichPWMisOn[10] == 1) {
				HAL_TIM_PWM_Start(tim4, TIM_CHANNEL_3);
			} else {
				HAL_TIM_PWM_Stop(tim4, TIM_CHANNEL_3);
			}
			// usart_printf("Returned2\r\n");
			break;
		}
	}
}


