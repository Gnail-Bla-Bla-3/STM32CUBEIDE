/*
 * driving.c
 *
 *  Created on: Jun 6, 2024
 *      Author: liangnie
 */
#include "main.h"
#include "remote_control.h"
#include "UART.h"
#include "buffer.h"
#include <math.h>

const float increaseSpeed = 0.008;
const float decreaseSpeed = 0.02;
const int8_t maxQEVal = 3;
const int16_t rotationSpeedOfChassis = 2000;

int8_t qeRotationWOWS = 0;
int8_t qButtonDown = 0;
int8_t eButtonDown = 0;
int8_t rButtonDown = 0;
float movementCounter[6] = {0, 0, 0, 0, 0, 0};

float graphingFunction (float x) {
	float y = x;
	return y;
}

float graphingFunctionEdgeCase (float x) {
	float y = graphingFunction(x);
	if (x <= 0) {
		y = 0;
	} else if (x > 1) {
		y = graphingFunction(1.0);
	}
	return y;
}

void allCheShit(uint8_t w, uint8_t a, uint8_t s, uint8_t d, uint8_t q, uint8_t e, uint8_t r, int16_t rcRPM[], int8_t chassisVsTurretDrive, float convertedAngle, PID_preset_t chassisPreset) {
	float scaledMovementCounter[6] = {0, 0, 0, 0, 0, 0};
	int16_t xJoystickDirection = 0;
	int16_t yJoystickDirection = 0;
	int16_t rotationOfChassis = 0;
	int16_t chassisTargetRPM[4] = {0, 0, 0, 0};

	if (w == 1) {
		movementCounter[0]+= increaseSpeed;
	} else {
		movementCounter[0]-= decreaseSpeed;
	}
	if (a == 1) {
		movementCounter[1]+= increaseSpeed;
	} else {
		movementCounter[1]-= decreaseSpeed;
	}
	if (s == 1) {
		movementCounter[2]+= increaseSpeed;
	} else {
		movementCounter[2]-= decreaseSpeed;
	}
	if (d == 1) {
		movementCounter[3]+= increaseSpeed;
	} else {
		movementCounter[3]-= decreaseSpeed;
	}
	if (q == 1) {
		movementCounter[4]+= increaseSpeed;
	} else {
		movementCounter[4]-= decreaseSpeed;
	}
	if (e == 1) {
		movementCounter[5]+= increaseSpeed;
	} else {
		movementCounter[5]-= decreaseSpeed;
	}

	usart_printf("1\r\n");

	if (q == 1 && qButtonDown == 0) {
		qButtonDown = 1;
		qeRotationWOWS--;
	} else if (q == 0 && qButtonDown == 1) {
		qButtonDown = 0;
	}
	if (e == 1 && eButtonDown == 0) {
		eButtonDown = 1;
		qeRotationWOWS++;
	} else if (e == 0 && eButtonDown == 1) {
		eButtonDown = 0;
	}
	if (qeRotationWOWS > maxQEVal) {
		qeRotationWOWS = 3;
	} else if (qeRotationWOWS < (-1*maxQEVal)) {
		qeRotationWOWS = -3;
	}
	if (r == 1 && rButtonDown == 0) {
		qeRotationWOWS = -1*qeRotationWOWS;
		rButtonDown = 1;
	} else if (r == 0 && rButtonDown == 1) {
		rButtonDown = 0;
	}
	usart_printf("2\r\n");
	for (int i = 0; i < 6; i++) {
		if (movementCounter[i] > 1) {
			movementCounter[i] = 1;
		} else if (movementCounter[i] < 0) {
			movementCounter[i] = 0;
		}
		scaledMovementCounter[i] = graphingFunctionEdgeCase(movementCounter[i]);
	}

	int16_t rcWS = 9005.7*((scaledMovementCounter[0]-scaledMovementCounter[2]));
	int16_t rcAD = 9005.7*((scaledMovementCounter[3]-scaledMovementCounter[1]));
	int16_t rcQE = 9005.7*((scaledMovementCounter[5]-scaledMovementCounter[4]));

	if (chassisVsTurretDrive == 1) {
		xJoystickDirection = (rcRPM[2]*cos(convertedAngle) - rcRPM[3]*sin(convertedAngle) + (rcAD*cos(convertedAngle) - rcWS*sin(convertedAngle)));
		yJoystickDirection = (rcRPM[2]*sin(convertedAngle) + rcRPM[3]*cos(convertedAngle) + (rcAD*sin(convertedAngle) + rcWS*cos(convertedAngle)));
		rotationOfChassis = rotationSpeedOfChassis * qeRotationWOWS;
	} else {
		xJoystickDirection = (rcRPM[2] + rcWS);
		yJoystickDirection = (rcRPM[3] + rcQE);
		rotationOfChassis = (rcRPM[0] + rcAD);
	}

	usart_printf("3\r\n");
    chassisTargetRPM[0] = yJoystickDirection + rotationOfChassis + xJoystickDirection;
    chassisTargetRPM[1] = yJoystickDirection + rotationOfChassis - xJoystickDirection;
    chassisTargetRPM[2] = -yJoystickDirection + rotationOfChassis - xJoystickDirection;
    chassisTargetRPM[3] = -yJoystickDirection + rotationOfChassis + xJoystickDirection;

    bufferLimitedDriveMode(chassisTargetRPM, chassisPreset);

}
