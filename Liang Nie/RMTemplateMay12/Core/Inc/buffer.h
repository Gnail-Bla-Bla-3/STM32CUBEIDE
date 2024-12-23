/*
 * buffer.h
 *
 *  Created on: May 22, 2024
 *      Author: liangnie
 */

#ifndef INC_BUFFER_H_
#define INC_BUFFER_H_

#include "main.h"
#include "CAN.h"


void settingMaxCurrentVal(float bufferValFromPMM, uint16_t powerLimit, uint8_t shift);
void bufferLimitedDriveMode(int16_t RPMtarget[], PID_preset_t preset);
/*
void driveMotorRPM (int16_t RPMtarget[], PID_preset_t preset);
void ScaleAllCurrentValuesForDriveMotors ();
void findingKIScaler();
void settingMaxCurrentVal (float bufferValFromPMM);
void createDriveMotorCAN();
*/
#endif /* INC_BUFFER_H_ */
