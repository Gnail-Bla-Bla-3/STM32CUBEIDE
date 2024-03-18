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
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

int16_t motorCurrent[11] = {0,0,0,0,0,0,0,0,0,0,0};
int16_t motorVoltage[7] = {0,0,0,0,0,0,0};

motor_measure_t motor_feedback[11];
b2b_t b2b;

int16_t targetRPM[11] = {0,0,0,0,0,0,0,0,0,0,0};
int16_t lastTargetRPM[11] = {0,0,0,0,0,0,0,0,0,0,0};
int16_t dRPM[11];           // derivative of last & current RPM readings
int16_t iRPM[11];           // integral of past encoder readings
int16_t errRPM[11];

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

#define get_b2b_feedback(ptr, data) \
{ \
    (ptr)->data1 = (uint16_t)((data)[0] << 8 | (data)[1]); \
    (ptr)->data2 = (uint16_t)((data)[2] << 8 | (data)[3]); \
    (ptr)->data3 = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->data4 = (uint16_t)((data)[6] << 8 | (data)[7]); \
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
        case CAN_b2b_ID: {
            get_b2b_feedback(&b2b, rx_data);                             // add back numbering code for extra CAN input lines
            break;
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
void CAN1_cmd_b2b(int16_t data1, int16_t data2, int16_t data3, int16_t data4) {
    uint32_t send_mail_box;
    b2b_tx_message.StdId = CAN_b2b_ID;
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
void set_motor_current(int8_t motorID, int16_t setCurrent) {
	motorCurrent[motorID-1] = setCurrent;
	switch(motorID) {
		case (1):
		case (2):
		case (3):
		case (4): {             // is group 1?
			CAN2_cmd_motors(CAN_GROUP1_ID, motorCurrent[0], motorCurrent[1], motorCurrent[2], motorCurrent[3]);
		}
		case (5):
		case (6):
		case (7):
		case (8): {             // is group 2?
			CAN2_cmd_motors(CAN_GROUP2_ID, motorCurrent[4], motorCurrent[5], motorCurrent[6], motorCurrent[7]);
		}
		case (9):
		case (10):
		case (11): {          // is group 3?
			CAN2_cmd_motors(CAN_GROUP3_ID, motorCurrent[8], motorCurrent[9], motorCurrent[10], 0);
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
void set_motor_voltage(int8_t motorID, int16_t setVoltage) {
	motorVoltage[motorID-5] = setVoltage;      // -1 for 0 indexing and -4 for group 1, which cannot contain GM6020s
	switch(motorID) {
		case (5):
		case (6):
		case (7):
		case (8): {             // is group 2?
			CAN2_cmd_motors(CAN_GROUP2V_ID, motorVoltage[0], motorVoltage[1], motorVoltage[2], motorVoltage[3]);
		}
		case (9):
		case (10):
		case (11): {          // is group 3?
			CAN2_cmd_motors(CAN_GROUP3V_ID, motorVoltage[4], motorVoltage[5], motorVoltage[6], 0);
		}
		default: {
	        break;
	    }
	}
}

int16_t calc_current2RPM_PID(int8_t motorID, PID_preset_t preset) {
	int16_t return_current_val = 0;
	int8_t ID = motorID - 1;                                                                // god forbid 0 indexing
	iRPM[ID] += targetRPM[ID] - motor_feedback[ID].speed_rpm;      // add to integral term
	dRPM[ID] = targetRPM[ID] - motor_feedback[ID].speed_rpm;       // update derivative term
	errRPM[ID] = targetRPM[ID] - motor_feedback[ID].speed_rpm;     // update proportional term
	return_current_val = (preset.kP * errRPM[ID]) + (preset.kI * iRPM[ID]) + (preset.kD * dRPM[ID]);
	return return_current_val;
}

void setMotorRPM(int8_t motorID, int16_t RPMtarget, PID_preset_t preset) {
	targetRPM[motorID-1] = RPMtarget;
	set_motor_current(motorID, calc_current2RPM_PID(motorID, preset));
}

int16_t getMotorRPM(int8_t motorID) {
	return motor_feedback[motorID-1].speed_rpm;
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
