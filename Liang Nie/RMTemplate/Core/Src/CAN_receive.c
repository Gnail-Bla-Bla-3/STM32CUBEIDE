/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "UART.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern power_heat_data_t power_heat_data;

extern float selfCalcChassisPower;
extern PID_preset_t chassisPreset;

int16_t motorCtrlVal[11] = {0,0,0,0,0,0,0,0,0,0,0};

motor_measure_t motor_feedback[11];
b2b_motorCtrl_t b2bMotorCtrl;
b2b_gyro_t b2bGyro;

int32_t targetRPM[11] = {0,0,0,0,0,0,0,0,0,0,0};
int32_t lastRPM[11] = {0,0,0,0,0,0,0,0,0,0,0};
int32_t dRPM[11];           // derivative of last & current RPM readings
int32_t iRPM[11];           // integral of past encoder readings
int32_t errRPM[11];

int32_t targetPosition[11] = {0,0,0,0,0,0,0,0,0,0,0};
int32_t lastPosition[11] = {0,0,0,0,0,0,0,0,0,0,0};
int32_t dPosition[11];           // derivative of last & current RPM readings
int32_t iPosition[11];           // integral of past encoder readings
int32_t errPosition[11];

extern can_msg_id_e boardID;

float lastPower;
float dPower;           // derivative of last & current RPM readings
float iPower;           // integral of past encoder readings
float errPower;
float powerLim = 30;

static CAN_TxHeaderTypeDef  b2b_tx_message;
static uint8_t              b2b_can_send_data[8];
static CAN_TxHeaderTypeDef  motors_tx_message;
static uint8_t              motors_can_send_data[8];


#define get_motor_feedback(ptr, data) \
{ \
    (ptr)->last_motor_position = (ptr)->motor_position; \
    (ptr)->motor_position = (uint16_t)((data)[0] << 8 | (data)[1]); \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]); \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6]; \
}                                                                                                    // reading feedback from the motors, also note that C610 (M2006) gives output torque for data[4&5] and no temperature reading

#define get_b2b_motorCtrl_feedback(ptr, data) \
{ \
    (ptr)->motor1_Ctrl = (uint16_t)((data)[0] << 8 | (data)[1]); \
    (ptr)->motor2_Ctrl = (uint16_t)((data)[2] << 8 | (data)[3]); \
    (ptr)->motor3_Ctrl = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->motor4_Ctrl = (uint16_t)((data)[6] << 8 | (data)[7]); \
}

#define get_b2b_gyro_feedback(ptr, data) \
{ \
    (ptr)->gyro_x = (uint16_t)((data)[0] << 8 | (data)[1]); \
    (ptr)->gyro_y = (uint16_t)((data)[2] << 8 | (data)[3]); \
    (ptr)->gyro_z = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->reserved = (uint16_t)((data)[6] << 8 | (data)[7]); \
}

/**
  * @brief          reading motor feedback from CAN FIFO
  * @param[in]      pointer to CAN handle
  * @retval         none
  */
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
            static uint8_t i = 0;
            i = rx_header.StdId - CAN_G1M1_ID;                             // get motor id by taking the difference between the first motor's ID (0 indexing) and the current motor's ID
            get_motor_feedback(&motor_feedback[i], rx_data);
            break;
        }
        case CAN_b2b_A_motorCtrl_ID: {
        	if(CAN_b2b_A_ID == boardID) {
        		break;
        	} else {
        		get_b2b_motorCtrl_feedback(&b2bMotorCtrl, rx_data);                             // add back numbering code for extra CAN input lines
        		if (b2bMotorCtrl.motor1_Ctrl >= 3376) {
        			b2bMotorCtrl.motor1_Ctrl = 3376;
        		} else if (b2bMotorCtrl.motor1_Ctrl <= 2132) {
        			b2bMotorCtrl.motor1_Ctrl = 2132;
        		}
        		break;
        	}
        }
        case CAN_b2b_B_gyro_ID: {
        	if(CAN_b2b_B_ID == boardID) {
        		break;
        	} else {
        		get_b2b_gyro_feedback(&b2bGyro, rx_data);                             // add back numbering code for extra CAN input lines
        	    break;
        	}
        }
        default: {
            break;
        }
    }
}

/**
  * @brief          sends board to board (b2b) communication data
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @retval         none
  */
void CAN1_cmd_b2b(can_msg_id_e canID, int16_t data1, int16_t data2, int16_t data3, int16_t data4) {
    uint32_t send_mail_box;
    b2b_tx_message.StdId = canID;
    b2b_tx_message.IDE = CAN_ID_STD;
    b2b_tx_message.RTR = CAN_RTR_DATA;
    b2b_tx_message.DLC = 0x08;
    b2b_can_send_data[0] = (data1 >> 8);
    b2b_can_send_data[1] = data1;
    b2b_can_send_data[2] = (data2 >> 8);
    b2b_can_send_data[3] = data2;
    b2b_can_send_data[4] = (data3 >> 8);
    b2b_can_send_data[5] = data3;
    b2b_can_send_data[6] = (data4 >> 8);
    b2b_can_send_data[7] = data4;
    HAL_CAN_AddTxMessage(&CAN_1, &b2b_tx_message, b2b_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
void CAN2_cmd_chassis_reset_ID(void) {             //         function to set motor IDs (by accessing the "quick setup" function)
    uint32_t send_mail_box;
    motors_tx_message.StdId = 0x700;
    motors_tx_message.IDE = CAN_ID_STD;
    motors_tx_message.RTR = CAN_RTR_DATA;
    motors_tx_message.DLC = 0x08;
    motors_can_send_data[0] = 0;
    motors_can_send_data[1] = 0;
    motors_can_send_data[2] = 0;
    motors_can_send_data[3] = 0;
    motors_can_send_data[4] = 0;
    motors_can_send_data[5] = 0;
    motors_can_send_data[6] = 0;
    motors_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&CAN_2, &motors_tx_message, motors_can_send_data, &send_mail_box);
}

/**
  * @brief          send control information through CAN bus 2
  * @param[in]      canID of the target motor's group and the motor current values for all motors in that group
  * @retval         none
  */
void CAN2_cmd_motors(can_msg_id_e canID, int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
    uint32_t send_mail_box;
    motors_tx_message.StdId = canID;
    motors_tx_message.IDE = CAN_ID_STD;
    motors_tx_message.RTR = CAN_RTR_DATA;
    motors_tx_message.DLC = 0x08;
    motors_can_send_data[0] = m1 >> 8;
    motors_can_send_data[1] = m1;
    motors_can_send_data[2] = m2 >> 8;
    motors_can_send_data[3] = m2;
    motors_can_send_data[4] = m3 >> 8;
    motors_can_send_data[5] = m3;
    motors_can_send_data[6] = m4 >> 8;
    motors_can_send_data[7] = m4;
    HAL_CAN_AddTxMessage(&CAN_2, &motors_tx_message, motors_can_send_data, &send_mail_box);
}


/**
  * @brief          simplification function to make communication with the motor not require the current data for all 4 motors
  * @param[in]      motor ID (1~11) and its target current (-16384 ~ 16384)
  * @retval         none
  */
void set_M3508_current(int8_t motorID, int16_t setCurrent) {
	motorCtrlVal[motorID-1] = setCurrent;
	switch(motorID) {
		case (1):
		case (2):
		case (3):
		case (4): {             // is group 1?
			CAN2_cmd_motors(CAN_GROUP1_ID, motorCtrlVal[0], motorCtrlVal[1], motorCtrlVal[2], motorCtrlVal[3]);
		}
		case (5):
		case (6):
		case (7):
		case (8): {             // is group 2?
			CAN2_cmd_motors(CAN_GROUP2_ID, motorCtrlVal[4], motorCtrlVal[5], motorCtrlVal[6], motorCtrlVal[7]);
			// usart_printf("%d\r\n", 20);
		}
		default: {
	        break;
	    }
	}
}

/**
  * @brief          similar to the above function, but specifically for the GM6020's voltage control mode
  * @param[in]      motor ID (5~11) and its target voltage (-25000 ~ 25000)
  * @retval         none
  */
void set_GM6020_voltage(int8_t motorID, int16_t setVoltage) {
	motorCtrlVal[motorID-1] = setVoltage;      // -1 for 0 indexing and -4 for group 1, which cannot contain GM6020s
	switch(motorID) {
		case (5):
		case (6):
		case (7):
		case (8): {             // is group 2?
			CAN2_cmd_motors(CAN_GROUP2_ID, motorCtrlVal[4], motorCtrlVal[5], motorCtrlVal[6], motorCtrlVal[7]);
		}
		case (9):
		case (10):
		case (11): {          // is group 3?
			CAN2_cmd_motors(CAN_GROUP3_ID, motorCtrlVal[8], motorCtrlVal[9], motorCtrlVal[10], 0);
		}
		default: {
	        break;
	    }
	}
}

void set_GM6020_current(int8_t motorID, int16_t setCurrent) {
	motorCtrlVal[motorID-1] = setCurrent;      // -1 for 0 indexing and -4 for group 1, which cannot contain GM6020s
	switch(motorID) {
		case (5):
		case (6):
		case (7):
		case (8): {             // is group 2?
			CAN2_cmd_motors(CAN_GROUP2C_ID, motorCtrlVal[4], motorCtrlVal[5], motorCtrlVal[6], motorCtrlVal[7]);
		}
		case (9):
		case (10):
		case (11): {          // is group 3?
			CAN2_cmd_motors(CAN_GROUP3C_ID, motorCtrlVal[8], motorCtrlVal[9], motorCtrlVal[10], 0);
		}
		default: {
	        break;
	    }
	}
}

int32_t calc_current2RPM_PID(int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	int32_t return_current_val = 0;
	int8_t ID = motorID - 1;                                                                // god forbid 0 indexing
	iRPM[ID] += RPMtarget - motor_feedback[ID].speed_rpm;      // add to integral term
	dRPM[ID] = lastRPM[ID] - motor_feedback[ID].speed_rpm;       // update derivative term
	errRPM[ID] = RPMtarget - motor_feedback[ID].speed_rpm;     // update proportional term
	lastRPM[ID] = motor_feedback[ID].speed_rpm;
	return_current_val = (preset.kP * errRPM[ID]) + (preset.kI * iRPM[ID]) + (preset.kD * dRPM[ID]);
	//apply power limit
	//if (ID == 6) {sendB2bData(CAN_b2b_B_gyro_ID, errRPM[5], 0, 0, 0);}
	if (return_current_val > 16384) {return_current_val = 16384;}
	if (return_current_val < -16384) {return_current_val = -16384;}
	return return_current_val;
}

int32_t calc_M2006_current2RPM_PID(int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	int32_t return_current_val = 0;
	int8_t ID = motorID - 1;                                                                // god forbid 0 indexing
	iRPM[ID] += RPMtarget - motor_feedback[ID].speed_rpm;      // add to integral term
	dRPM[ID] = lastRPM[ID] - motor_feedback[ID].speed_rpm;       // update derivative term
	errRPM[ID] = RPMtarget - motor_feedback[ID].speed_rpm;     // update proportional term
	lastRPM[ID] = motor_feedback[ID].speed_rpm;
	//if(iRPM[ID] >= 3000) {iRPM[ID] = 3000;}
	return_current_val = (preset.kP * errRPM[ID]) + (preset.kI * iRPM[ID]) + (preset.kD * dRPM[ID]);
	sendB2bData(CAN_b2b_B_gyro_ID, motor_feedback[ID].speed_rpm, -5400, 0, 0);
	//apply power limit
	if (return_current_val > 10000) {return_current_val = 10000;}
	if (return_current_val < -10000) {return_current_val = -10000;}
	return return_current_val;
}

int32_t calc_voltage2RPM_PID(int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	int32_t return_voltage_val = 0;
	int8_t ID = motorID - 1;                                                                // god forbid 0 indexing
	iRPM[ID] += RPMtarget - motor_feedback[ID].speed_rpm;      // add to integral term
	dRPM[ID] = lastRPM[ID] - motor_feedback[ID].speed_rpm;       // update derivative term
	errRPM[ID] = RPMtarget - motor_feedback[ID].speed_rpm;     // update proportional term
	// usart_printf("%d %d\r\n", getMotorRPM(5), 0);
	lastRPM[ID] = motor_feedback[ID].speed_rpm;
	if (iRPM[ID] >= 3400) {iRPM[ID] = 2600;}
	return_voltage_val = (preset.kP * errRPM[ID]) + (preset.kI * iRPM[ID]) + (preset.kD * dRPM[ID]);
	//apply power limit
	if (return_voltage_val > 25000) {return_voltage_val = 25000;}
	if (return_voltage_val < -25000) {return_voltage_val = -25000;}

	return return_voltage_val;
}

int32_t calc_current2Position_PID(int8_t motorID, int16_t position, PID_preset_t preset) {
	int32_t return_current_val = 0;
	int8_t ID = motorID - 1;                                                                // god forbid 0 indexing
	iPosition[ID] += position - motor_feedback[ID].motor_position;      // add to integral term
	dPosition[ID] = lastPosition[ID] - motor_feedback[ID].motor_position;       // update derivative term
	errPosition[ID] = position - motor_feedback[ID].motor_position;     // update proportional term
	lastPosition[ID] = motor_feedback[ID].motor_position;
	return_current_val = (preset.kP * errPosition[ID]) + (preset.kI * iPosition[ID]) + (preset.kD * dPosition[ID]);
	//apply power limit
	if (return_current_val > 16384) {return_current_val = 16384;}
	if (return_current_val < -16384) {return_current_val = -16384;}
	return return_current_val;
}

int32_t calc_voltage2Position_PID(int8_t motorID, int16_t position, PID_preset_t preset) {
	int32_t return_voltage_val = 0;
	int8_t ID = motorID - 1;                                                                // god forbid 0 indexing
	iPosition[ID] += position - motor_feedback[ID].motor_position;      // add to integral term
	dPosition[ID] = lastPosition[ID] - motor_feedback[ID].motor_position;       // update derivative term
	errPosition[ID] = position - motor_feedback[ID].motor_position;     // update proportional term
	lastPosition[ID] = motor_feedback[ID].motor_position;
	return_voltage_val = (preset.kP * errPosition[ID]) + (preset.kI * iPosition[ID]) + (preset.kD * dPosition[ID]);
	//apply power limit
	if (return_voltage_val > 25000) {return_voltage_val = 25000;}
	if (return_voltage_val < -25000) {return_voltage_val = -25000;}
	return return_voltage_val;
}

// DO NOT use current ctrl and voltage ctrl concurrently for GM6020

void setM3508RPM(int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	set_M3508_current(motorID, calc_current2RPM_PID(motorID, RPMtarget, preset));
}

void setGM6020currentRPM(int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	set_GM6020_current(motorID, calc_current2RPM_PID(motorID, RPMtarget, preset));
}

void setGM6020voltageRPM(int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	set_GM6020_voltage(motorID, calc_voltage2RPM_PID(motorID, RPMtarget, preset));
}

void setGM6020currentPosition(int8_t motorID, int16_t position, PID_preset_t preset) {
	set_GM6020_current(motorID, calc_current2Position_PID(motorID, position, preset));

}

void setGM6020voltagePosition(int8_t motorID, int16_t position, PID_preset_t preset) {
	set_GM6020_voltage(motorID, calc_voltage2Position_PID(motorID, position, preset));
}

void setM2006RPM(int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	set_M3508_current(motorID, calc_M2006_current2RPM_PID(motorID, RPMtarget, preset));
}

uint16_t getMotorPosition(int8_t motorID) {
	return motor_feedback[motorID-1].motor_position;
}

int16_t getMotorRPM(int8_t motorID) {
	return motor_feedback[motorID-1].speed_rpm;
}

int16_t getMotorCurrent(int8_t motorID) {
	return motor_feedback[motorID-1].given_current;
}

uint8_t getMotorTemperature(int8_t motorID) {
	return motor_feedback[motorID-1].temperate;
}

int16_t getLastMotorPosition(int8_t motorID) {
	return motor_feedback[motorID-1].last_motor_position;
}


void setB2bID (can_msg_id_e canID) {
	boardID = canID;
}

void sendB2bData(can_msg_id_e canID, int16_t data1, int16_t data2, int16_t data3, int16_t data4) {
	CAN1_cmd_b2b(canID, data1, data2, data3, data4);
}

void setPowerLimit(float wattPower) {
	powerLim = wattPower;
}



chassis_motor_current applyPowerlimit(chassis_motor_config chassis, chassis_motor_RPM targetRPM, float chassisPower) {
	float multiplier = 1;
	float calcPower = 0;
	float chassisVoltage = power_heat_data.chassis_voltage / 1000;
	float diff = 0;
	chassis_motor_current setCurrent = {{0,0,0,0,0,0,0,0}};
	if (chassisVoltage > 24) {
		chassisVoltage = 24;
	}

	//do {
		multiplier = 1;
		calcPower = 0;
		for (int i = 0; i < 8; i++) {
			if (chassis.motorID[i] > 0 && chassis.motorID[i] < 12) {
				setCurrent.motorCurrent[i] = calc_current2RPM_PID(chassis.motorID[i], targetRPM.motorRPM[i], chassisPreset);
				int absCurrent = setCurrent.motorCurrent[i];
				if (setCurrent.motorCurrent[i] < 0) {
					absCurrent = -setCurrent.motorCurrent[i];
				}
				calcPower +=  chassisVoltage * absCurrent / 16384.0 * 20.0;
				//calcPower = chassisPower;
			}
		}
		if (calcPower > powerLim) {
			multiplier  =  powerLim / calcPower;      // pos = over
		} //else {break;}
		diff = chassisPower - calcPower;
		//usart_printf("%f %f %f\r\n", calcPower, chassisPower, diff);
		for (int i = 0; i < 8; i++) {
			targetRPM.motorRPM[i] = multiplier * targetRPM.motorRPM[i] *0.5;

//			if (setCurrent.motorCurrent[i] < 0) {
//				setCurrent.motorCurrent[i] += -multiplier/4 - 1;
//			} else {
//				setCurrent.motorCurrent[i] -= multiplier/4 - 1;
//			}
		}

		//usart_printf("%f %d %d %f %f\r\n", chassisPower, 30, calc_current2RPM_PID(chassis.motorID[0], targetRPM.motorRPM[0], chassisPreset), multiplier, calcPower);

	//} while (calcPower > powerLim);

	//if (iPower >= 1000) {
	//	iPower = 0;
	//}

	return setCurrent;
}


/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void) {
    return &motor_feedback[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void) {
    return &motor_feedback[5];
}

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_trigger_motor_measure_point(void) {
    return &motor_feedback[6];
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i) {
    return &motor_feedback[(i & 0x03)];
}
