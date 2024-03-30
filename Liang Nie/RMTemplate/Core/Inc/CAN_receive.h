/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#define CAN_1 hcan1
#define CAN_2 hcan2

/* CAN send and receive ID */
typedef enum
{
	//group identifiers for group 2 of M3508 are the same as group 1 of GM6020 (voltage ctrl)
    CAN_GROUP1_ID = 0x200,               // M3508 G1, M2006 G1
	CAN_GROUP2_ID = 0x1FF,               // M3508 G2, M2006 G2, GM6020 G1
	CAN_GROUP3_ID = 0x2FF,              // GM6020 G2

	CAN_GROUP2C_ID = 0x1FE,          // voltage control for GM6020
	CAN_GROUP3C_ID = 0x2FE,         // group 3 is exclusive to GM6020 motors only and contain 3 IDs

	CAN_G1M1_ID = 0x201,
	CAN_G1M2_ID = 0x202,
	CAN_G1M3_ID = 0x203,
	CAN_G1M4_ID = 0x204,

	CAN_G2M1_ID = 0x205,
	CAN_G2M2_ID = 0x206,
	CAN_G2M3_ID = 0x207,
	CAN_G2M4_ID = 0x208,

	CAN_G3M1_ID = 0x209,
	CAN_G3M2_ID = 0x20A,
	CAN_G3M3_ID = 0x20B,

	CAN_b2b_A_ID = 0x101,
	CAN_b2b_A_motorCtrl_ID = 0x102,

	CAN_b2b_B_ID = 0x111,
	CAN_b2b_B_gyro_ID = 0x112,
	CAN_b2b_B_motorFeedback_ID = 0x112,


} can_msg_id_e;

typedef struct {
    uint16_t motor_position;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_motor_position;
} motor_measure_t;

typedef struct {
    int16_t motor1_Ctrl;
    int16_t motor2_Ctrl;
    int16_t motor3_Ctrl;
    int16_t motor4_Ctrl;
} b2b_motorCtrl_t;

typedef struct {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t reserved;
} b2b_gyro_t;

typedef struct {
    float kP;
    float kI;
    float kD;
} PID_preset_t;

typedef struct {
	uint8_t motorID[8];
} chassis_motor_config;

typedef struct {
	int16_t motorRPM[8];
} chassis_motor_RPM;

typedef struct {
	int16_t motorCurrent[8];
} chassis_motor_current;

extern motor_measure_t motor_feedback[11];
extern b2b_motorCtrl_t b2bMotorCtrl;

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
void CAN1_cmd_b2b(can_msg_id_e canID, int16_t data1, int16_t data2, int16_t data3, int16_t data4);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
extern void CAN2_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
extern void CAN2_cmd_motors(can_msg_id_e canID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          simplification function to make communication with the motor not require current data for all 4 motors
  * @param[in]      none
  * @retval         motor data point
  */
void set_M3508_current(int8_t motorID, int16_t setCurrent);

void set_GM6020_voltage(int8_t motorID, int16_t setVoltage);

void set_GM6020_current(int8_t motorID, int16_t setCurrent);

/**
  * @brief          calculates the current that should be given based on
  * @param[in]      ID of the target motor, struct of the PID preset
  * @retval         current value
  */
int32_t calc_current2RPM_PID(int8_t motorID, int16_t RPMtarget, PID_preset_t preset);

int32_t calc_M2006_current2RPM_PID(int8_t motorID, int16_t RPMtarget, PID_preset_t preset);

/**
  * @brief          sets a motor to run at a target RPM
  * @param[in]      ID of the target motor, target RPM, the struct of the PID preset
  * @retval         none
  */
void setM3508RPM(int8_t motorID, int16_t RPMtarget, PID_preset_t preset);

void setGM6020currentRPM(int8_t motorID, int16_t RPMtarget, PID_preset_t preset);

void setGM6020voltageRPM(int8_t motorID, int16_t RPMtarget, PID_preset_t preset);

void setGM6020currentPosition(int8_t motorID, int16_t position, PID_preset_t preset);

void setGM6020voltagePosition(int8_t motorID, int16_t position, PID_preset_t preset);

void setM2006RPM(int8_t motorID, int16_t RPMtarget, PID_preset_t preset);

uint16_t getMotorPosition(int8_t motorID);

int16_t getMotorRPM(int8_t motorID);

int16_t getMotorCurrent(int8_t motorID);

uint8_t getMotorTemperature(int8_t motorID);

int16_t getLastMotorPosition(int8_t motorID);

void setB2bID (can_msg_id_e canID);

void sendB2bData(can_msg_id_e canID, int16_t data1, int16_t data2, int16_t data3, int16_t data4);

int16_t getB2bData1 (can_msg_id_e canID);

void setPowerLimit(float wattPower);

chassis_motor_current applyPowerlimit(chassis_motor_config chassis, chassis_motor_RPM targetRPM, float chassisPower);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
