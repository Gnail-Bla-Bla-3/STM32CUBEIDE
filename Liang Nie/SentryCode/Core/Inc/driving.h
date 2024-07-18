/*
 * driving.h
 *
 *  Created on: Jun 6, 2024
 *      Author: liangnie
 */

#ifndef INC_DRIVING_H_
#define INC_DRIVING_H_

#include "main.h"

void allCheShit(uint8_t w, uint8_t a, uint8_t s, uint8_t d, uint8_t rotation, int16_t rcRPM[], int8_t chassisVsTurretDrive, float convertedAngle, PID_preset_t chassisPreset);

#endif /* INC_DRIVING_H_ */
