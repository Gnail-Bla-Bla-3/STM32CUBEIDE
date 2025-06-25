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

#ifndef CAN_H
#define CAN_H

#include "main.h"

#define CAN_1 hcan1
#define CAN_2 hcan2

/* CAN send and receive ID */
typedef enum {
	//group identifiers for group 2 of M3508 are the same as group 1 of GM6020 (voltage ctrl)
    CAN_DJI_GROUP1_ID = 0x200,               // M3508 G1, M2006 G1
	CAN_DJI_GROUP2_ID = 0x1FF,               // M3508 G2, M2006 G2, GM6020 G1
	CAN_DJI_GROUP3_ID = 0x2FF,              // GM6020 G2

	CAN_DJI_GROUP2_Current_ID = 0x1FE,          // voltage control for GM6020
	CAN_DJI_GROUP3_Current_ID = 0x2FE,         // group 3 is exclusive to GM6020 motors only and contain 3 IDs

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
	CAN_b2b_A_motorCtrl_set_1_ID = 0x102,
	CAN_b2b_A_motorCtrl_set_2_ID = 0x103,
	CAN_b2b_A_RC_control = 0x104,
	CAN_b2b_A_RC_control2 = 0x105,

	CAN_b2b_B_ID = 0x111,
	CAN_b2b_B_gyro_xy_ID = 0x112,
	CAN_b2b_B_gyro_z_ID = 0x113,
	CAN_b2b_B_motorFeedback_ID = 0x114,

	CAN_POWER_ID = 0x101,
	CAN_HEAT_ID = 0x102,
	CAN_STATUS_1_ID = 0x103,
	CAN_STATUS_2_ID = 0x104,
} CAN_ID;

typedef enum {
	Bus1 = 1,
	Bus2 = 2,
} CAN_Bus;

typedef enum {
	M3508 = 1,
	C620 = 1,
	M2006 = 2,
	C610 = 2,
	GM6020 = 3,         // voltage
	GM6020_C = 4,         // current
} MotorType_ID;

typedef struct {
    uint16_t rotor_position;
    int16_t speed_rpm;
    int16_t torque_current;
    uint8_t temperate;
} motorFeedback_t;

typedef struct {
    float kP;
    float kI;
    float kD;
} PID_preset_t;

typedef struct {
	int8_t Group1[4];            //0x200
	int8_t Group2[4];            //0x1FF
	int8_t Group3[3];            //0x2FF
	// ID3_Slot4 does not exist
} DJI_MotorDeclaration_t;

typedef struct {
	int16_t Group1[4];            //0x200
	int16_t Group2[4];            //0x1FF
	int16_t Group3[4];            //0x2FF
} motorControlBuffer_t;

typedef struct {
    int32_t lastVal;
    int32_t pVal;
    int32_t iVal;
    int32_t dVal;
} PID_data_t;

// void settingMaxCurrentVal (float bufferValFromPMM);

void CAN_ResetID(CAN_Bus bus);
void CAN_DriveMotor(CAN_Bus bus, CAN_ID headerID, int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void CAN_defineMotor(CAN_Bus bus, MotorType_ID motorType, int8_t motorID);
void CAN_setMotorDefinition(int8_t *Declaration, MotorType_ID motorType);
int8_t CAN_getMotorDefinition(CAN_Bus bus, uint8_t group, int8_t groupID);
void CAN_transmit(CAN_Bus bus, CAN_ID headerID, uint64_t data);
void CAN_setMotorCurrent(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t setCurrent);
void CAN_setMotorVoltage(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t setVoltage);
void CAN_setMotorCtrlVal(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t ctrlVal);
int32_t calcRPM_PID(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t RPMtarget, PID_preset_t preset);
int32_t calcPosition_PID(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t positionTarget, PID_preset_t preset);
int32_t applyCtrlLimit(MotorType_ID motorType, int32_t val);
void setMotorRPM(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t RPMtarget, PID_preset_t preset);
void setMotorPosition(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t positionTarget, PID_preset_t preset);
uint64_t fourBitShift(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);
uint64_t otherSignals(uint16_t data1, int8_t switch1, int8_t switch2);
uint16_t getRotorPosition(CAN_Bus bus, MotorType_ID motorType, int8_t motorID);

int16_t getMotorRPM(CAN_Bus bus, MotorType_ID motorType, int8_t motorID);

int16_t getMotorCurrent(CAN_Bus bus, MotorType_ID motorType, int8_t motorID);

uint8_t getMotorTemperature(CAN_Bus bus, MotorType_ID motorType, int8_t motorID);

#endif
