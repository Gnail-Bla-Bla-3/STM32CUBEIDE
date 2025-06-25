/*
 * CAN_FD.h
 *
 *  Created on: Feb 21, 2025
 *      Author: Edison
 */

#ifndef INC_CAN_FD_H_
#define INC_CAN_FD_H_

#include "main.h"

#define hcan_t FDCAN_HandleTypeDef

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

	CAN_CYBERGEAR_M1_ID = 0x31,
	CAN_CYBERGEAR_M2_ID = 0x32,
	CAN_CYBERGEAR_M3_ID = 0x33,
	CAN_CYBERGEAR_M4_ID = 0x34,

	CAN_b2b_A_ID = 0x101,
	CAN_b2b_A_motorCtrl_set_1_ID = 0x102,
	CAN_b2b_A_motorCtrl_set_2_ID = 0x103,
	CAN_b2b_A_RC_Val_ID1 = 0x104,
	CAN_b2b_A_RC_Val_ID2 = 0x105,

	CAN_b2b_B_ID = 0x111,
	CAN_b2b_B_gyro_xy_ID = 0x112,
	CAN_b2b_B_gyro_z_ID = 0x113,
	CAN_b2b_B_motorFeedback_ID = 0x114,

	CAN_POWER_ID = 0x101,
	CAN_HEAT_ID = 0x102,
	CAN_STATUS_1_ID = 0x103,
	CAN_STATUS_2_ID = 0x104,

	CAN_EXTENDED_A = 0x1F000000,  // 1f here makes bit 28 ~ 24 "1", so "0001 1111 0000 0000 0000 0000 0000 0000", so all that's left is to add stuff here
} CAN_ID;

typedef struct{           //小米电机结构体
	uint8_t CAN_ID;       //CAN ID
    uint8_t MCU_ID;       //MCU唯一标识符[后8位，共64位]
	float Angle;          //回传角度
	float Speed;          //回传速度
	float Torque;         //回传力矩
	float Temp;			  //回传温度

	uint16_t set_current;
	uint16_t set_speed;
	uint16_t set_position;

	uint8_t error_code;

	float Angle_Bias;

} Cybergear;

//主机CANID设置
#define Master_CAN_ID 0x00                      //主机ID
//控制命令宏定义
#define Communication_Type_GetID 0x00           //获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01 	//用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02	//用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03	    //电机使能运行
#define Communication_Type_MotorStop 0x04	    //电机停止运行
#define Communication_Type_SetPosZero 0x06	    //设置电机机械零位
#define Communication_Type_CanID 0x07	        //更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11	//读取单个参数
#define Communication_Type_SetSingleParameter 0x12	//设定单个参数
#define Communication_Type_ErrorFeedback 0x15	    //故障反馈帧

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

typedef struct {
    float kP;
    float kI;
    float kD;
} PID_preset_t;

typedef enum {
	Bus1 = 1,
	Bus2 = 2,
	Bus3 = 3
} CAN_Bus;

void bsp_can_init(void);
void can_filter_init(void);
int16_t getRCfakechannel(uint8_t index);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_send_extended(hcan_t *hfdcan, uint32_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint32_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void CAN_setMotorCtrlVal(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t ctrlVal);
void setMotorRPM(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t RPMtarget, PID_preset_t preset);
void setMotorPosition(CAN_Bus bus, MotorType_ID motorType, int8_t motorID, int16_t positionTarget, PID_preset_t preset);
int32_t applyCtrlLimit(MotorType_ID motorType, int32_t val);
uint16_t getRotorPosition(CAN_Bus bus, MotorType_ID motorType, int8_t motorID);
int16_t getMotorRPM(CAN_Bus bus, MotorType_ID motorType, int8_t motorID);
int16_t getMotorCurrent(CAN_Bus bus, MotorType_ID motorType, int8_t motorID);
uint8_t getMotorTemperature(CAN_Bus bus, MotorType_ID motorType, int8_t motorID);


#endif /* INC_CAN_FD_H_ */
