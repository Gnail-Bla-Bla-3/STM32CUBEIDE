/**
  ****************************something something copyright****************************
  * just use it smh
  ****************************something something copyright****************************
  */

#include "CAN.h"
#include "UART.h"
#include "main.h"
#include "string.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef  B2bTransmitHeader;
static uint8_t              b2b_can_send_data[16];
static CAN_TxHeaderTypeDef  MotorTransmitHeader;
static uint8_t              CAN_MotorSendBuffer[8];

#define get_motor_feedback(ptr, data) \
{ \
    (ptr)->rotor_position = (uint16_t)((data)[0] << 8 | (data)[1]); \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]); \
    (ptr)->torque_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6]; \
}

DJI_MotorDeclaration_t DJI_MotorDeclaration[2] = {{{-1, -1, -1, -1}, {-1, -1, -1, -1}, {-1, -1, -1}}, {{-1, -1, -1, -1}, {-1, -1, -1, -1}, {-1, -1, -1}}};
motorControlBuffer_t motorControlBuffer[2] = {{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}};
motorFeedback_t motorFeedback[2][11];
PID_data_t PID_data[2][11];
int16_t RCVAL[7] = {0, 0, 0, 0, 0, 0, 0};



extern robot_status_t robot_status;
extern power_heat_data_t power_heat_data;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId) {
        case CAN_G1M1_ID:
        case CAN_G1M2_ID:
        case CAN_G1M3_ID:
        case CAN_G1M4_ID:
        case CAN_G2M1_ID:
        case CAN_G2M2_ID:
        case CAN_G2M3_ID:
        case CAN_G2M4_ID:
        case CAN_G3M1_ID:
        case CAN_G3M2_ID:
        case CAN_G3M3_ID: {
            uint8_t i = rx_header.StdId - CAN_G1M1_ID; // get motor id
            if (hcan == &hcan1) {
                //memcpy(&motorFeedback[0][i].rotor_position, &rx_data[1], 1);
                //memcpy((motorFeedback[0][i].rotor_position + 1), &rx_data[0], 1);
                get_motor_feedback(&motorFeedback[0][i], rx_data);
            } else if (hcan == &hcan2) {
                //memcpy(&motorFeedback[1][i], &rx_data[0], 8);
                get_motor_feedback(&motorFeedback[1][i], rx_data);
                //usart_printf("%d \r\n", motorFeedback[1][0].rotor_position);
            }
            break;
        }
        case CAN_POWER_ID: {

            memcpy(&power_heat_data, &rx_data, 8);
            break;
        }
        case CAN_HEAT_ID: {
            memcpy((&power_heat_data.buffer_energy), &rx_data, 8);
            break;
        }

        case CAN_STATUS_1_ID: {
            //memcpy(&robot_status, &rx_data, 8);
            break;
        }
        case CAN_STATUS_2_ID: {
            //memcpy((&robot_status.shooter_barrel_heat_limit), &rx_data, 6);
            break;
        }
        case CAN_b2b_A_RC_Val_ID1: {
        	if (hcan == &hcan2) {
        		// usart_printf("CAN_Recv = %x %x %x %x %x %x %x %x\r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
        		uint16_t Temparary[4] = {0, 0, 0, 0};

        		for (int8_t i = 0; i < 4; i++) {
        			Temparary[i] = Temparary[i] + ((uint16_t)rx_data[2*i] << 0);
        			Temparary[i] = Temparary[i] + ((uint16_t)rx_data[(2*i)+1] << 8);
        			RCVAL[i] = ((int16_t)Temparary[i]) - 660;
        			// usart_printf("beans2\r\n");
        		}

        		// usart_printf("%d %d %d %d\r\n", RCVAL[0], RCVAL[1], RCVAL[2], RCVAL[3]);
        		break;
        	}
        	break;
		}
		case CAN_b2b_A_RC_Val_ID2: {
			if (hcan == &hcan2) {
				uint16_t Temparary2[3] = {0, 0, 0};

				Temparary2[0] = (((uint16_t)rx_data[6] << 0) + ((uint16_t)rx_data[7] << 8));//-660;
				Temparary2[1] = (uint16_t)rx_data[2] << 0;
				Temparary2[2] = (uint16_t)rx_data[4] << 0;

			//for (int8_t i = 0; i < 3; i++) {
			//	RCVAL[i+4] = ((int16_t)Temparary2[i]);
			//}
				RCVAL[4] = ((int16_t)Temparary2[0]) - 660;
				RCVAL[5] = ((int16_t)Temparary2[1]);
				RCVAL[6] = ((int16_t)Temparary2[2]);
				//usart_printf("num = %d \r\n", RCVAL[4]);
				break;
			}
			break;
		}

        default: {
            break;
        }
    }
}

int16_t getRCfakechannel(uint8_t index) {
	return RCVAL[index];
}

// CAN_transmit needs some work, right now it's "CAN1_sendFloats"
void CAN_transmit(CAN_Bus bus, CAN_ID headerID, uint64_t data) {
    uint32_t send_mail_box;
    B2bTransmitHeader.StdId = headerID;
    B2bTransmitHeader.IDE = CAN_ID_STD;
    B2bTransmitHeader.RTR = CAN_RTR_DATA;
    B2bTransmitHeader.DLC = 0x08;
    memcpy(&b2b_can_send_data[0], &data, 8);
    HAL_CAN_AddTxMessage(&CAN_1, &B2bTransmitHeader, b2b_can_send_data, &send_mail_box);
}

void CAN_ResetID(CAN_Bus bus) {             //function to set motor IDs (by accessing the "quick setup" function)
    uint32_t send_mail_box;
    MotorTransmitHeader.StdId = 0x700;
    MotorTransmitHeader.IDE = CAN_ID_STD;
    MotorTransmitHeader.RTR = CAN_RTR_DATA;
    MotorTransmitHeader.DLC = 0x08;
    for (int i=0 ; i<8; i++) {                              // set all bytes to be 0, just in case
    	CAN_MotorSendBuffer[i] = 0;
    }
    if (bus == 1) {
		HAL_CAN_AddTxMessage(&CAN_1, &MotorTransmitHeader, CAN_MotorSendBuffer, &send_mail_box);
	} else if (bus == 2) {
		HAL_CAN_AddTxMessage(&CAN_2, &MotorTransmitHeader, CAN_MotorSendBuffer, &send_mail_box);
	} else {      // do nothing
	}
}

void CAN_DriveMotor(CAN_Bus bus, CAN_ID headerID, int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
    uint32_t send_mail_box;
    MotorTransmitHeader.StdId = headerID;
    MotorTransmitHeader.IDE = CAN_ID_STD;
    MotorTransmitHeader.RTR = CAN_RTR_DATA;
    MotorTransmitHeader.DLC = 0x08;
    //memcpy(&CAN_MotorSendBuffer[0], &m1, 2);
    //memcpy(&CAN_MotorSendBuffer[2], &m2, 2);
    //memcpy(&CAN_MotorSendBuffer[4], &m3, 2);
    //memcpy(&CAN_MotorSendBuffer[6], &m4, 2);     // first 8 bits and last 8 bits of each int16_t is flipped, kms
    CAN_MotorSendBuffer[0] = m1 >> 8;
    CAN_MotorSendBuffer[1] = m1;
    CAN_MotorSendBuffer[2] = m2 >> 8;
    CAN_MotorSendBuffer[3] = m2;
    CAN_MotorSendBuffer[4] = m3 >> 8;
    CAN_MotorSendBuffer[5] = m3;
    CAN_MotorSendBuffer[6] = m4 >> 8;
    CAN_MotorSendBuffer[7] = m4;
    if (bus == 1) {
    	HAL_CAN_AddTxMessage(&CAN_1, &MotorTransmitHeader, CAN_MotorSendBuffer, &send_mail_box);
    } else if (bus == 2) {
    	HAL_CAN_AddTxMessage(&CAN_2, &MotorTransmitHeader, CAN_MotorSendBuffer, &send_mail_box);
    } else {      // do nothing
    }
}

void CAN_defineMotor(CAN_Bus bus, MotorType_ID motorType, int8_t motorID) {
	switch(motorType) {
		case (1) :
		case (2) : {
			switch(motorID) {
				case (1):
				case (2):
				case (3):
				case (4): {
					CAN_setMotorDefinition(&DJI_MotorDeclaration[bus - 1].Group1[motorID - 1], motorType);
					break;
				}
				case (5):
				case (6):
				case (7):
				case (8): {
					CAN_setMotorDefinition(&DJI_MotorDeclaration[bus - 1].Group2[motorID - 1], motorType);
					break;
				}
				default: {
					usart_printf("WARNING - motor ID %d does not match any known motors IDs \r\n", motorID);
					break;
				}
			}
			break;
		}
		case (3): {
			switch(motorID) {
				case (1):
				case (2):
				case (3):
				case (4): {
					CAN_setMotorDefinition(&DJI_MotorDeclaration[bus - 1].Group2[motorID - 1], motorType);
					break;
				}
				case (5):
				case (6):
				case (7): {
					CAN_setMotorDefinition(&DJI_MotorDeclaration[bus - 1].Group3[motorID - 5], motorType);
					break;
				}
				case (8): {               // call the user a dumbass for spawning an ID 8 on 0x2FF
					usart_printf("WARNING - motor ID 8 is not available for GM6020s according to DJI's CAN protocols \r\n", motorID);
					break;
				}
				default: {
					usart_printf("WARNING - motor ID %d does not match any known motors IDs \r\n", motorID);
					break;
				}
			}
			break;
		}
		default: {
			usart_printf("WARNING - motor type %d does not match any registered motors \r\n", motorType);
			break;
		}
	}
}

void CAN_setMotorDefinition(int8_t *Declaration, MotorType_ID motorType) {
	if (*Declaration == -1) {    // ok
		*Declaration = motorType;
	} else {                           // repeat definition, do nothing
		usart_printf("WARNING - motor ID has already been set \r\n");
	}
}

int8_t CAN_getMotorDefinition(CAN_Bus bus, uint8_t group, int8_t groupID) {
	switch(bus) {
		case (1): {
			switch (group) {
				case (1): {
					if (groupID >= 1 && groupID <= 4) {
						return DJI_MotorDeclaration[0].Group1[groupID - 1];
					}
					break;
				}
				case (2): {
					if (groupID >= 1 && groupID <= 4) {
						return DJI_MotorDeclaration[0].Group2[groupID - 1];
					}
					break;
				}
				case (3): {
					if (groupID >= 1 && groupID <= 3) {
						return DJI_MotorDeclaration[0].Group3[groupID - 1];
					}
					break;
				}
			}
			break;
		}
		case (2): {
			switch (group) {
				case (1): {
					if (groupID >= 1 && groupID <= 4) {
						return DJI_MotorDeclaration[1].Group1[groupID - 1];
					}
					break;
				}
				case (2): {
					if (groupID >= 1 && groupID <= 4) {
						return DJI_MotorDeclaration[1].Group2[groupID - 1];
					}
					break;
				}
				case (3): {
					if (groupID >= 1 && groupID <= 3) {
						return DJI_MotorDeclaration[1].Group3[groupID - 1];
					}
					break;
				}
			}
			break;
		}
	}
	return 0;
}

void CAN_setMotorCurrent(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t setCurrent) {
	switch(motorType) {
		case (1):
		case (2): {
			switch(motorID) {
				case (1):
				case (2):
				case (3):
				case (4): {             // 0x200
					motorControlBuffer[bus - 1].Group1[motorID - 1] = setCurrent;
					CAN_DriveMotor(bus, CAN_DJI_GROUP1_ID, motorControlBuffer[bus - 1].Group1[0], motorControlBuffer[bus - 1].Group1[1], motorControlBuffer[bus - 1].Group1[2], motorControlBuffer[bus - 1].Group1[3]);
					break;
				}
				case (5):
				case (6):
				case (7):
				case (8): {             // 0x1FF
					motorControlBuffer[bus - 1].Group2[motorID - 5] = setCurrent;
					CAN_DriveMotor(bus, CAN_DJI_GROUP2_ID, motorControlBuffer[bus - 1].Group2[0], motorControlBuffer[bus - 1].Group2[1], motorControlBuffer[bus - 1].Group2[2], motorControlBuffer[bus - 1].Group2[3]);
					break;
				}
				default: {
					break;
				}
			}
			break;
		}
		case (4): {
			switch(motorID) {
				case (1):
				case (2):
				case (3):
				case (4): {             // 0x1FF
					motorControlBuffer[bus - 1].Group2[motorID - 1] = setCurrent;
					CAN_DriveMotor(bus, CAN_DJI_GROUP2_Current_ID, motorControlBuffer[bus - 1].Group2[0], motorControlBuffer[bus - 1].Group2[1], motorControlBuffer[bus - 1].Group2[2], motorControlBuffer[bus - 1].Group2[3]);
					break;
				}
				case (5):
				case (6):
				case (7): {             // 0x2FF
					motorControlBuffer[bus - 1].Group3[motorID - 5] = setCurrent;
					CAN_DriveMotor(bus, CAN_DJI_GROUP3_Current_ID, motorControlBuffer[bus - 1].Group3[0], motorControlBuffer[bus - 1].Group3[1], motorControlBuffer[bus - 1].Group3[2], motorControlBuffer[bus - 1].Group3[3]);
					break;
				}
				case (8): {
					// shame the user for saying there's an ID 4 in group 3
					break;
				}
				default: {
					break;
				}
			}
			break;
		}
		default: {

		}
	}
}

void CAN_setMotorVoltage(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t setVoltage) {
	switch(motorType) {
		case (1):
		case (2): {
			// error: no voltage controls available for those two motors
		}
		case (3): {
			switch(motorID) {
				case (1):
				case (2):
				case (3):
				case (4): {             // 0x1FE
					motorControlBuffer[bus - 1].Group2[motorID - 1] = setVoltage;
					CAN_DriveMotor(bus, CAN_DJI_GROUP2_ID, motorControlBuffer[bus - 1].Group2[0], motorControlBuffer[bus - 1].Group2[1], motorControlBuffer[bus - 1].Group2[2], motorControlBuffer[bus - 1].Group2[3]);
				}
				case (5):
				case (6):
				case (7): {             // 0x2FE
					motorControlBuffer[bus - 1].Group3[motorID - 5] = setVoltage;
					CAN_DriveMotor(bus, CAN_DJI_GROUP3_ID, motorControlBuffer[bus - 1].Group3[0], motorControlBuffer[bus - 1].Group3[1], motorControlBuffer[bus - 1].Group3[2], motorControlBuffer[bus - 1].Group3[3]);
				}
				case (8): {
					// shame the user for saying there's an ID 4 in group 3
				}
				default: {
					break;
				}
			}
		}
		default: {

		}
	}
}


// Max Power *
void CAN_setMotorCtrlVal(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t ctrlVal) {
	switch(motorType) {
		case (1):
		case (2): {

			switch(motorID) {
				case (1):
				case (2):
				case (3):
				case (4): {             // 0x200
					motorControlBuffer[bus - 1].Group1[motorID - 1] = ctrlVal;
					CAN_DriveMotor(bus, CAN_DJI_GROUP1_ID, motorControlBuffer[bus - 1].Group1[0], motorControlBuffer[bus - 1].Group1[1], motorControlBuffer[bus - 1].Group1[2], motorControlBuffer[bus - 1].Group1[3]);
					break;
				}
				case (5):
				case (6):
				case (7):
				case (8): {             // 0x1FF
					motorControlBuffer[bus - 1].Group2[motorID - 5] = ctrlVal;
					CAN_DriveMotor(bus, CAN_DJI_GROUP2_ID, motorControlBuffer[bus - 1].Group2[0], motorControlBuffer[bus - 1].Group2[1], motorControlBuffer[bus - 1].Group2[2], motorControlBuffer[bus - 1].Group2[3]);
					break;
				}
				default: {
					break;
				}
			}
			break;
		}
		case (3): {
			switch(motorID) {
				case (1):
				case (2):
				case (3):
				case (4): {             // 0x1FF
					motorControlBuffer[bus - 1].Group2[motorID - 1] = ctrlVal;
					CAN_DriveMotor(bus, CAN_DJI_GROUP2_ID, motorControlBuffer[bus - 1].Group2[0], motorControlBuffer[bus - 1].Group2[1], motorControlBuffer[bus - 1].Group2[2], motorControlBuffer[bus - 1].Group2[3]);
					break;
				}
				case (5):
				case (6):
				case (7): {             // 0x2FF
					motorControlBuffer[bus - 1].Group3[motorID - 5] = ctrlVal;
					CAN_DriveMotor(bus, CAN_DJI_GROUP3_ID, motorControlBuffer[bus - 1].Group3[0], motorControlBuffer[bus - 1].Group3[1], motorControlBuffer[bus - 1].Group3[2], motorControlBuffer[bus - 1].Group3[3]);
					// usart_printf("trig \r\n");
					break;
				}
				case (8): {
					// shame the user for saying there's an ID 4 in group 3
					break;
				}
				default: {
					break;
				}
			}
			break;
		}
		case (4): {
			switch(motorID) {
				case (1):
				case (2):
				case (3):
				case (4): {             // 0x1FF
					motorControlBuffer[bus - 1].Group2[motorID - 1] = ctrlVal;
					CAN_DriveMotor(bus, CAN_DJI_GROUP2_Current_ID, motorControlBuffer[bus - 1].Group2[0], motorControlBuffer[bus - 1].Group2[1], motorControlBuffer[bus - 1].Group2[2], motorControlBuffer[bus - 1].Group2[3]);
					break;
				}
				case (5):
				case (6):
				case (7): {             // 0x2FF
					motorControlBuffer[bus - 1].Group3[motorID - 5] = ctrlVal;
					CAN_DriveMotor(bus, CAN_DJI_GROUP3_Current_ID, motorControlBuffer[bus - 1].Group3[0], motorControlBuffer[bus - 1].Group3[1], motorControlBuffer[bus - 1].Group3[2], motorControlBuffer[bus - 1].Group3[3]);
					break;
				}
				case (8): {
					// shame the user for saying there's an ID 4 in group 3
					break;
				}
				default: {
					break;
				}
			}
			break;
		}
	}
}

int32_t calcRPM_PID(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	uint8_t ID;
	int32_t return_val = 0;
	uint8_t busID = bus - 1;
	switch(motorType) {
		case (1):
		case (2): {
			ID = motorID - 1;
			break;
		}
		case (3):
		case (4): {
			ID = motorID + 3;
			break;
		}
		default: {
			ID = motorID - 1;
			break;
		}
	}
	PID_data[busID][ID].pVal = RPMtarget - motorFeedback[busID][ID].speed_rpm;     // update proportional term
	PID_data[busID][ID].iVal += PID_data[busID][ID].pVal;                                                         // add to integral term
	PID_data[busID][ID].dVal = PID_data[busID][ID].lastVal - motorFeedback[busID][ID].speed_rpm;       // update derivative term
	PID_data[busID][ID].lastVal = motorFeedback[busID][ID].speed_rpm;                        // save the current RPM to be used in the next cycle
	return_val = (preset.kP * PID_data[busID][ID].pVal) + (preset.kI * PID_data[busID][ID].iVal) + (preset.kD * PID_data[busID][ID].dVal);     // calculates PID result
	return_val = applyCtrlLimit(motorType, return_val);           // applies min/max limits to the final control value
	// I think this works, but what should the min/max values be for the I term to reset, if it is needed at all?
	return return_val;
}

int32_t calcPosition_PID(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t positionTarget, PID_preset_t preset) {
	uint8_t ID;
	int32_t return_val = 0;
	uint8_t busID = bus - 1;
	int16_t cwPositionDifference = 0;
	int16_t ccwPositionDifference = 0;
	switch(motorType) {
		case (1):
		case (2): {
			ID = motorID - 1;
			break;
		}
		case (3):
		case (4): {
			ID = motorID + 3;
			break;
		}
		default: {
			ID = motorID - 1;
			break;
		}
	}

	if (positionTarget - motorFeedback[busID][ID].rotor_position >= 0) {
		cwPositionDifference = positionTarget - motorFeedback[busID][ID].rotor_position;
	} else {
		cwPositionDifference = 8192 - motorFeedback[busID][ID].rotor_position + positionTarget;
	}

	if (motorFeedback[busID][ID].rotor_position - positionTarget >= 0) {
		ccwPositionDifference =  motorFeedback[busID][ID].rotor_position - positionTarget;
	} else {
		ccwPositionDifference = 8192 + motorFeedback[busID][ID].rotor_position - positionTarget;
	}

	//ccwPositionDifference = 8191 - cwPositionDifference;
	//usart_printf("CW %d | CCW %d \r\n", cwPositionDifference, ccwPositionDifference);

//	if (motorFeedback[busID][ID].rotor_position <= 4095) {
//
//
//
//		positionDifference = positionTarget - motorFeedback[busID][ID].rotor_position;
//		usart_printf("CW %d %d \r\n", positionDifference, motorFeedback[busID][ID].rotor_position - positionTarget);
//	} else {     // ccw better
//		positionDifference = motorFeedback[busID][ID].rotor_position - positionTarget;
//		usart_printf("CCW %d %d \r\n", positionDifference, motorFeedback[busID][ID].rotor_position - positionTarget);
//	}

	PID_data[busID][ID].pVal = positionTarget - motorFeedback[busID][ID].rotor_position;     // update proportional term
	PID_data[busID][ID].iVal += PID_data[busID][ID].pVal;                                                         // add to integral term
	PID_data[busID][ID].dVal = PID_data[busID][ID].lastVal - motorFeedback[busID][ID].rotor_position;       // update derivative term
	PID_data[busID][ID].lastVal = motorFeedback[busID][ID].rotor_position;                        // save the current RPM to be used in the next cycle
	return_val = (preset.kP * PID_data[busID][ID].pVal) + (preset.kI * PID_data[busID][ID].iVal) + (preset.kD * PID_data[busID][ID].dVal);     // calculates PID result
	return_val = applyCtrlLimit(motorType, return_val);           // applies min/max limits to the final control value
	//usart_printf("ch1: %d\r\n", return_val);
	return return_val;
}

int32_t applyCtrlLimit(MotorType_ID motorType, int32_t val) {
	switch (motorType) {
		case (1):
		case (2):
		case (4): {
			if (val > 16384) {
				val = 16384;
			} else if (val < -16384) {
				val = -16384;
			}
			break;
		}
		case (3): {
			if (val > 25000) {
				val = 25000;
			} else if (val < -25000) {
				val = -25000;
			}
			break;
		}
	}
	return val;
}

void setMotorRPM(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	CAN_setMotorCtrlVal(bus, motorType, motorID, calcRPM_PID(bus, motorType, motorID, RPMtarget, preset));
}

void setMotorPosition(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t positionTarget, PID_preset_t preset) {
	CAN_setMotorCtrlVal(bus, motorType, motorID, calcPosition_PID(bus, motorType, motorID, positionTarget, preset));
}

uint16_t getRotorPosition(CAN_Bus bus, MotorType_ID motorType, int8_t motorID) {
	switch (motorType) {
		case (1):
		case (2): {
			if (motorID >= 1 && motorID <= 8) {
				return motorFeedback[bus - 1][motorID - 1].rotor_position;
			}
			break;
		}
		case (3):
		case (4): {
			if (motorID >= 1 && motorID <= 7) {
				return motorFeedback[bus - 1][motorID + 3].rotor_position;
			}
			break;
		}
	}
	return -1;
}

int16_t getMotorRPM(CAN_Bus bus, MotorType_ID motorType, int8_t motorID) {
	switch (motorType) {
		case (1):
		case (2): {
			if (motorID >= 1 && motorID <= 8) {
				return motorFeedback[bus - 1][motorID - 1].speed_rpm;
			}
			break;
		}
		case (3):
		case (4): {
			if (motorID >= 1 && motorID <= 7) {
				return motorFeedback[bus - 1][motorID + 3].speed_rpm;
			}
			break;
		}
	}
	return -1;
}

int16_t getMotorCurrent(CAN_Bus bus, MotorType_ID motorType, int8_t motorID) {
	switch (motorType) {
		case (1):
		case (2): {
			if (motorID >= 1 && motorID <= 8) {
				return motorFeedback[bus - 1][motorID - 1].torque_current;
			}
			break;
		}
		case (3):
		case (4): {
			if (motorID >= 1 && motorID <= 7) {
				return motorFeedback[bus - 1][motorID + 3].torque_current;
			}
			break;
		}
	}
	return -1;
}

uint8_t getMotorTemperature(CAN_Bus bus, MotorType_ID motorType, int8_t motorID) {
	switch (motorType) {
		case (1):
		case (2): {
			if (motorID >= 1 && motorID <= 8) {
				return motorFeedback[bus - 1][motorID - 1].temperate;
			}
			break;
		}
		case (3):
		case (4): {
			if (motorID >= 1 && motorID <= 7) {
				return motorFeedback[bus - 1][motorID + 3].temperate;
			}
			break;
		}
	}
	return -1;
}
