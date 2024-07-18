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
const int16_t rotationSpeedOfChassis = 7000;

int8_t qeRotationWOWS = 0;
int8_t qButtonDown = 0;
int8_t eButtonDown = 0;
int8_t rButtonDown = 0;
float movementCounter[4] = {0, 0, 0, 0};

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

void allCheShit(uint8_t w, uint8_t a, uint8_t s, uint8_t d, uint8_t rotation, int16_t rcRPM[], int8_t chassisVsTurretDrive, float convertedAngle, PID_preset_t chassisPreset) {
	float scaledMovementCounter[4] = {0, 0, 0, 0};
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

	// usart_printf("1\r\n");

	// usart_printf("2\r\n");
	for (int i = 0; i < 4; i++) {
		if (movementCounter[i] > 1) {
			movementCounter[i] = 1;
		} else if (movementCounter[i] < 0) {
			movementCounter[i] = 0;
		}
		scaledMovementCounter[i] = graphingFunctionEdgeCase(movementCounter[i]);
	}

	int16_t rcWS = 9005.7*((scaledMovementCounter[0]-scaledMovementCounter[2]));
	int16_t rcAD = 9005.7*((scaledMovementCounter[3]-scaledMovementCounter[1]));


	xJoystickDirection = (rcRPM[2]*cos(convertedAngle) - rcRPM[3]*sin(convertedAngle) + (rcAD*cos(convertedAngle) - rcWS*sin(convertedAngle)));
	yJoystickDirection = (rcRPM[2]*sin(convertedAngle) + rcRPM[3]*cos(convertedAngle) + (rcAD*sin(convertedAngle) + rcWS*cos(convertedAngle)));
	rotationOfChassis = (rotationSpeedOfChassis * rotation) + rcRPM[0];

	// usart_printf("3\r\n");
    chassisTargetRPM[0] = yJoystickDirection + rotationOfChassis + xJoystickDirection;
    chassisTargetRPM[1] = yJoystickDirection + rotationOfChassis - xJoystickDirection;
    chassisTargetRPM[2] = -yJoystickDirection + rotationOfChassis - xJoystickDirection;
    chassisTargetRPM[3] = -yJoystickDirection + rotationOfChassis + xJoystickDirection;

    bufferLimitedDriveMode(chassisTargetRPM, chassisPreset);

}
