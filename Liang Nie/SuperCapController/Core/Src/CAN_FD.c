/*
 * UART.C
 *
 *  Created on: Feb 20, 2025
 *      Author: swjxw
 */
#include "CAN_FD.h"
#include "main.h"
#include "UART.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "cybergear.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

FDCAN_RxHeaderTypeDef fdcan1_RxHeader;
FDCAN_TxHeaderTypeDef fdcan1_TxHeader;

#define get_motor_feedback(ptr, data) \
{ \
    (ptr)->rotor_position = (uint16_t)((data)[0] << 8 | (data)[1]); \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]); \
    (ptr)->torque_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6]; \
}

motorControlBuffer_t motorControlBuffer[2] = {{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}};
motorFeedback_t motorFeedback[2][11];
PID_data_t PID_data[2][11];

static uint8_t CAN_MotorSendBuffer[8];

void bsp_can_init(void) {
	//MX_FDCAN1_Init();
	//MX_FDCAN2_Init();
	//MX_FDCAN3_Init();
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	//HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void can_filter_init(void) {
	FDCAN_FilterTypeDef fdcan_filter;
	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	fdcan_filter.FilterID1 = 0x00;
	fdcan_filter.FilterID2 = 0x00;
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
	//HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}

uint8_t CANFD_GetRxHeaderDataLength(FDCAN_RxHeaderTypeDef header) {
	uint8_t len;
	if(header.DataLength<=FDCAN_DLC_BYTES_8)
		len = header.DataLength;
	if(header.DataLength<=FDCAN_DLC_BYTES_12)
		len = 12;
	if(header.DataLength<=FDCAN_DLC_BYTES_16)
		len = 16;
	if(header.DataLength<=FDCAN_DLC_BYTES_20)
		len = 20;
	if(header.DataLength<=FDCAN_DLC_BYTES_24)
		len = 24;
	if(header.DataLength<=FDCAN_DLC_BYTES_32)
		len = 32;
	if(header.DataLength<=FDCAN_DLC_BYTES_48)
		len = 48;
	if(header.DataLength<=FDCAN_DLC_BYTES_64)
		len = 64;
	return len;
}

uint32_t CANFD_GetTxHeaderDataLength(uint32_t len) {
	uint32_t length = 0;
	if(len<=8)
		length = len;
	if(len==12)
		length = FDCAN_DLC_BYTES_12;
	if(len==16)
		length = FDCAN_DLC_BYTES_16;
	if(len==20)
		length = FDCAN_DLC_BYTES_20;
	if(len==24)
		length = FDCAN_DLC_BYTES_24;
	if(len==32)
		length = FDCAN_DLC_BYTES_32;
	if(len==48)
		length = FDCAN_DLC_BYTES_48;
	if(len==64)
		length = FDCAN_DLC_BYTES_64;
	return length;
}

uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len) {
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
    pTxHeader.DataLength = CANFD_GetTxHeaderDataLength(len);
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data);
}

uint8_t fdcanx_send_extended(hcan_t *hfdcan, uint32_t id, uint8_t *data, uint32_t len) {
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_EXTENDED_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
    pTxHeader.DataLength = CANFD_GetTxHeaderDataLength(len);
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data);
}

uint8_t fdcanx_receive(hcan_t *hfdcan, uint32_t *rec_id, uint8_t *buf) {
	FDCAN_RxHeaderTypeDef pRxHeader;
	if(HAL_FDCAN_GetRxMessage(hfdcan ,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK) {
		*rec_id = pRxHeader.Identifier;
		return CANFD_GetRxHeaderDataLength(pRxHeader);
	}
	return 0;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	uint8_t rx_data[8] = {0};
	uint32_t rec_id;
	uint8_t rxDecodeBuffer1[4] = {0};
	uint16_t rxDecodeBuffer2[4] = {0};
	fdcanx_receive(hfdcan, &rec_id, rx_data);
	//usart_printf("CAN_Recv = %x %x %x %x %x %x %x %x\r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
	if (hfdcan == &hfdcan1) {		// DJI motors
		switch (rec_id) {
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
				uint8_t i = rec_id - CAN_G1M1_ID;  // get motor id
				if (hfdcan == &hfdcan1) {
					get_motor_feedback(&motorFeedback[0][i], rx_data);
				} else if (0) {
					get_motor_feedback(&motorFeedback[1][i], rx_data);
				}
				break;
			}
			case 0X7F:
				usart_printf("got something = %d\r\n", 0);
				break;
			case CAN_POWER_ID: {
				//memcpy(&power_heat_data, &rx_data, 8);
				break;
			}
			case CAN_HEAT_ID: {
				//memcpy((&power_heat_data.buffer_energy), &rx_data, 8);
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

			default: {
				break;
			}
		}
	}
	if (hfdcan == &hfdcan2) {	//cybergear motors

		rxDecodeBuffer1[0] = rec_id;				// [bit 0 - 15] Target Header ID
		rxDecodeBuffer1[1] = rec_id >> 8;		// [bit 8 - 15] Motor ID
		rxDecodeBuffer1[2] = rec_id >> 16;		// [bit 16 - 23] Run Mode & Error Info
		rxDecodeBuffer1[3] = rec_id >> 24;		// [bit 24 - 28] Communication Type
		rxDecodeBuffer2[0] = rx_data[0]<<8|rx_data[1];	// [byte 0 - 1] Angular Position (-4pi ~ 4pi)
		rxDecodeBuffer2[1] = rx_data[2]<<8|rx_data[3];	// [byte 2 - 3] Angular Velocity in Radians
		rxDecodeBuffer2[2] = rx_data[4]<<8|rx_data[5];	// [byte 4 - 5] Torque in Nm
		rxDecodeBuffer2[3] = rx_data[6]<<8|rx_data[7];	// [byte 6 - 7] Temperature in x10C
		//usart_printf("CAN_Recv = %x %x %x %x %x %x %x %x\r\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
		//usart_printf("CAN_Recv = %d\r\n", rxDecodeBuffer2[0]);
		cybergear_rx_callback(rxDecodeBuffer1, rxDecodeBuffer2);
	}

}

void CAN_DriveMotor(CAN_Bus bus, CAN_ID headerID, int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
    CAN_MotorSendBuffer[0] = m1 >> 8;
    CAN_MotorSendBuffer[1] = m1;
    CAN_MotorSendBuffer[2] = m2 >> 8;
    CAN_MotorSendBuffer[3] = m2;
    CAN_MotorSendBuffer[4] = m3 >> 8;
    CAN_MotorSendBuffer[5] = m3;
    CAN_MotorSendBuffer[6] = m4 >> 8;
    CAN_MotorSendBuffer[7] = m4;
    if (bus == 1) {
    	fdcanx_send_data(&hfdcan1, headerID, CAN_MotorSendBuffer, 8);
    } else if (bus == 2) {
    	fdcanx_send_data(&hfdcan2, headerID, CAN_MotorSendBuffer, 8);
    } else if (bus == 3) {
    	fdcanx_send_data(&hfdcan3, headerID, CAN_MotorSendBuffer, 8);
    } else {      // do nothing
    }
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

	PID_data[busID][ID].pVal = positionTarget - motorFeedback[busID][ID].rotor_position;     // update proportional term
	PID_data[busID][ID].iVal += PID_data[busID][ID].pVal;                                                         // add to integral term
	PID_data[busID][ID].dVal = PID_data[busID][ID].lastVal - motorFeedback[busID][ID].rotor_position;       // update derivative term
	PID_data[busID][ID].lastVal = motorFeedback[busID][ID].rotor_position;                        // save the current RPM to be used in the next cycle
	return_val = (preset.kP * PID_data[busID][ID].pVal) + (preset.kI * PID_data[busID][ID].iVal) + (preset.kD * PID_data[busID][ID].dVal);     // calculates PID result
	return_val = applyCtrlLimit(motorType, return_val);           // applies min/max limits to the final control value
	//usart_printf("ch1: %d\r\n", return_val);
	return return_val;
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

/*
void MX_FDCAN1_Init(void) {
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 6;
  hfdcan1.Init.NominalSyncJumpWidth = 5;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 6;
  hfdcan1.Init.DataSyncJumpWidth = 5;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 5;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_16;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
}

void MX_FDCAN2_Init(void) {
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 6;
  hfdcan2.Init.NominalSyncJumpWidth = 5;
  hfdcan2.Init.NominalTimeSeg1 = 15;
  hfdcan2.Init.NominalTimeSeg2 = 5;
  hfdcan2.Init.DataPrescaler = 6;
  hfdcan2.Init.DataSyncJumpWidth = 5;
  hfdcan2.Init.DataTimeSeg1 = 15;
  hfdcan2.Init.DataTimeSeg2 = 5;
  hfdcan2.Init.MessageRAMOffset = 0x800;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 4;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 4;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
}

void MX_FDCAN3_Init(void) {
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 6;
  hfdcan3.Init.NominalSyncJumpWidth = 5;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 5;
  hfdcan3.Init.DataPrescaler = 6;
  hfdcan3.Init.DataSyncJumpWidth = 5;
  hfdcan3.Init.DataTimeSeg1 = 15;
  hfdcan3.Init.DataTimeSeg2 = 5;
  hfdcan3.Init.MessageRAMOffset = 0x406;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.RxFifo0ElmtsNbr = 4;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 4;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK) {
    Error_Handler();
  }
}
*/
