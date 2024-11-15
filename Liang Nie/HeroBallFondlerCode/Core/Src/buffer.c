/*
 * buffer.c
 *
 *  Created on: May 22, 2024
 *      Author: liangnie
 */
#include "buffer.h"
#include "CAN.h"
#include "UART.h"
#include "main.h"
#include "string.h"

float maxPowerLimitor = 0;
float powerScaler = 0;
int16_t M3508Current[4] = {0, 0, 0, 0};
int16_t M3508ScaledCurrent[4] = {0, 0, 0, 0};

void findingKIScaler() {
	uint32_t sum = 0;
	for (int i = 0; i < 4; i++) {
		if (M3508Current[i] > 0) {
			sum += M3508Current[i];
		} else {
			sum -= M3508Current[i];
		}
	}
	powerScaler = maxPowerLimitor/sum;
	if (powerScaler > 1) {
		powerScaler = 1;
	} else if (powerScaler < 0) {
		powerScaler = 0;
	}
	// usart_printf("%f/%d = %f\r\n", maxPowerLimitor, sum, powerScaler);
}

void settingMaxCurrentVal (float bufferValFromPMM, uint16_t powerLimit) {

	// uint16_t adjustedPowerLimit = powerLimit - 6;

	float maxPowerForRobot = 0;
	if (powerLimit > 100) {
		maxPowerForRobot = 4;
	} if (powerLimit >= 60 && powerLimit <=100) {
		maxPowerForRobot = (0.05*powerLimit)-1;
	} if (powerLimit < 60) {
		maxPowerForRobot = 2;
	}

	if (bufferValFromPMM > 50) {
		maxPowerLimitor = 8192;
	} if (bufferValFromPMM > 5 && bufferValFromPMM <= 50) {
		maxPowerLimitor = (179.8*bufferValFromPMM) - 798.0f;
	} if (bufferValFromPMM <= 5) {
		maxPowerLimitor = 0.00001;
	}
	maxPowerLimitor = maxPowerLimitor * maxPowerForRobot;
	// maxPowerLimitor = 8192*8;
}

void driveMotorRPM (int16_t RPMtarget[], PID_preset_t preset) {
	for (int i = 0; i < 4; i++) {
		M3508Current[i] = calcRPM_PID(Bus1, M3508, i+1, RPMtarget[i], preset);
	}
	// M3508Current[motorID-1] = calcRPM_PID(Bus2, M3508, motorID, RPMtarget, preset);
}

void ScaleAllCurrentValuesForDriveMotors () {
	findingKIScaler();
	for (int i = 0; i < 4; i++) {
		M3508ScaledCurrent[i] = M3508Current[i] * powerScaler;
	}
}

void createDriveMotorCAN() {
	for (int j = 0; j < 4; j++) {
		CAN_setMotorCtrlVal(Bus1, M3508, j+1, M3508ScaledCurrent[j]);
	}
}

void bufferLimitedDriveMode(int16_t RPMtarget[], PID_preset_t preset) {
	driveMotorRPM(RPMtarget, preset);
	findingKIScaler();
	ScaleAllCurrentValuesForDriveMotors();
	createDriveMotorCAN();
}
