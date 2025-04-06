/*
 * cybergear.c
 *
 *  Created on: Mar 26, 2025
 *      Author: Edison
 */

#include "main.h"
#include "CAN_FD.h"
#include "cybergear.h"
#include "UART.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

FDCAN_RxHeaderTypeDef rxMsg;//发送接收结构体
FDCAN_TxHeaderTypeDef txMsg;//发送配置结构体
uint8_t rx_data[8];       //接收数据
uint32_t Motor_Can_ID;    //接收数据电机ID
uint8_t byte[4];          //转换临时数据
uint32_t send_mail_box = {0};//NONE


uint8_t can_txd(/*hcan_t *hfdcan, */FDCAN_TxHeaderTypeDef CAN_Tx, uint8_t *tx_data) {
	//CAN_Tx.Identifier=Index;
	CAN_Tx.IdType=FDCAN_EXTENDED_ID;
	CAN_Tx.TxFrameType=FDCAN_DATA_FRAME;
	CAN_Tx.DataLength = FDCAN_DLC_BYTES_8;
	CAN_Tx.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
	CAN_Tx.BitRateSwitch=FDCAN_BRS_ON;
	CAN_Tx.FDFormat=FDCAN_FD_CAN;
	CAN_Tx.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
	CAN_Tx.MessageMarker=0;
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &CAN_Tx, tx_data); //CAN发送宏定义
}


MI_Motor mi_motor[4];//预先定义四个小米电机

/**
  * @brief          浮点数转4字节函数
  * @param[in]      f:浮点数
  * @retval         4字节数组
  * @description  : IEEE 754 协议
  */
static uint8_t* Float_to_Byte(float f) {
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;
	byte[0] = (longdata & 0xFF000000) >> 24;
	byte[1] = (longdata & 0x00FF0000) >> 16;
	byte[2] = (longdata & 0x0000FF00) >> 8;
	byte[3] = (longdata & 0x000000FF);
	return byte;
}

/**
  * @brief          小米电机回文16位数据转浮点
  * @param[in]      x:16位回文
  * @param[in]      x_min:对应参数下限
  * @param[in]      x_max:对应参数上限
  * @param[in]      bits:参数位数
  * @retval         返回浮点值
  */
static float uint16_to_float(uint16_t x,float x_min,float x_max,int bits) {
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/**
  * @brief          小米电机发送浮点转16位数据
  * @param[in]      x:浮点
  * @param[in]      x_min:对应参数下限
  * @param[in]      x_max:对应参数上限
  * @param[in]      bits:参数位数
  * @retval         返回浮点值
  */
static int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief          写入电机参数
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      Index:写入参数对应地址
  * @param[in]      Value:写入参数值
  * @param[in]      Value_type:写入参数数据类型
  * @retval         none
  */
static void Set_Motor_Parameter(MI_Motor *Motor,uint16_t Index,float Value,char Value_type) {
	uint8_t tx_data[8];
	FDCAN_TxHeaderTypeDef CAN_Tx;
	CAN_Tx.Identifier = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|Motor->CAN_ID;
	CAN_Tx.IdType=FDCAN_EXTENDED_ID;
	CAN_Tx.TxFrameType=FDCAN_DATA_FRAME;
	tx_data[0]=Index;
	tx_data[1]=Index>>8;
	tx_data[2]=0x00;
	tx_data[3]=0x00;
	if(Value_type == 'f') {
		Float_to_Byte(Value);
		tx_data[4]=byte[3];
		tx_data[5]=byte[2];
		tx_data[6]=byte[1];
		tx_data[7]=byte[0];
	}
	else if(Value_type == 's') {
		tx_data[4]=(uint8_t)Value;
		tx_data[5]=0x00;
		tx_data[6]=0x00;
		tx_data[7]=0x00;
	}
	can_txd(CAN_Tx, tx_data);
}


/**
  * @brief          提取电机回复帧扩展ID中的电机CANID
  * @param[in]      CAN_ID_Frame:电机回复帧中的扩展CANID
  * @retval         电机CANID
  */
static uint32_t Get_Motor_ID(uint32_t CAN_ID) {
	return (CAN_ID&0xFFFF)>>8;
}

/**
  * @brief          电机回复帧数据处理函数
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      DataFrame:数据帧
  * @param[in]      IDFrame:扩展ID帧
  * @retval         None
  */
static void Motor_Data_Handler(MI_Motor *Motor,uint8_t DataFrame[8],uint32_t IDFrame) {
		Motor->Angle=uint16_to_float(DataFrame[0]<<8|DataFrame[1],MIN_P,MAX_P,16);
		Motor->Speed=uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);
		Motor->Torque=uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN,T_MAX,16);
		Motor->Temp=(DataFrame[6]<<8|DataFrame[7])*Temp_Gain;
		Motor->error_code=(IDFrame&0x1F0000)>>16;
}

/**
  * @brief          小米电机ID检查
  * @param[in]      id:  控制电机CAN_ID【出厂默认0x7F】
  * @retval         none
  */
void check_cybergear(uint8_t ID) {
    uint8_t tx_data[8] = {0};
    FDCAN_TxHeaderTypeDef CAN_Tx;
    CAN_Tx.Identifier = Communication_Type_GetID<<24|Master_CAN_ID<<8|ID;
    CAN_Tx.IdType=FDCAN_EXTENDED_ID;
    CAN_Tx.TxFrameType=FDCAN_DATA_FRAME;
    can_txd(CAN_Tx, tx_data);
}

/**
  * @brief          使能小米电机
  * @param[in]      Motor:对应控制电机结构体
  * @retval         none
  */
void start_cybergear(MI_Motor *Motor) {
    uint8_t tx_data[8] = {0};
    FDCAN_TxHeaderTypeDef CAN_Tx;
    CAN_Tx.Identifier = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|Motor->CAN_ID;
    CAN_Tx.IdType=FDCAN_EXTENDED_ID;
    CAN_Tx.TxFrameType=FDCAN_DATA_FRAME;
    can_txd(CAN_Tx, tx_data);
}

/**
  * @brief          停止电机
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      clear_error:清除错误位（0 不清除 1清除）
  * @retval         None
  */
void stop_cybergear(MI_Motor *Motor,uint8_t clear_error) {
	uint8_t tx_data[8]={0};
	FDCAN_TxHeaderTypeDef CAN_Tx;
	tx_data[0]=clear_error;//清除错误位设置
	CAN_Tx.Identifier = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|Motor->CAN_ID;
	CAN_Tx.IdType=FDCAN_EXTENDED_ID;
	CAN_Tx.TxFrameType=FDCAN_DATA_FRAME;
	can_txd(CAN_Tx, tx_data);
}

/**
  * @brief          设置电机模式(必须停止时调整！)
  * @param[in]      Motor:  电机结构体
  * @param[in]      Mode:   电机工作模式（1.运动模式Motion_mode 2. 位置模式Position_mode 3. 速度模式Speed_mode 4. 电流模式Current_mode）
  * @retval         none
  */
void set_mode_cybergear(MI_Motor *Motor,uint8_t Mode) {
	Set_Motor_Parameter(Motor, Run_mode, Mode,'s');
}

/**
  * @brief          电流控制模式下设置电流
  * @param[in]      Motor:  电机结构体
  * @param[in]      Current:电流设置
  * @retval         none
  */
void set_current_cybergear(MI_Motor *Motor, float Current) {
	Set_Motor_Parameter(Motor, Iq_Ref, Current, 'f');
}

void set_maxCurrent_cybergear(MI_Motor *Motor, float MaxCurrent) {
	Set_Motor_Parameter(Motor, Limit_Cur, MaxCurrent, 'f');
}

void set_speed_cybergear(MI_Motor *Motor, float RefSpd) {
	Set_Motor_Parameter(Motor, Spd_Ref, RefSpd, 'f');
}


/**
  * @brief          设置电机零点
  * @param[in]      Motor:  电机结构体
  * @retval         none
  */
void set_zeropos_cybergear(MI_Motor *Motor) {
	uint8_t tx_data[8]={0};
	FDCAN_TxHeaderTypeDef CAN_Tx;
	tx_data[0] = 1;
	CAN_Tx.Identifier = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|Motor->CAN_ID;
	CAN_Tx.IdType=FDCAN_EXTENDED_ID;
	CAN_Tx.TxFrameType=FDCAN_DATA_FRAME;
	can_txd(CAN_Tx, tx_data);
}

/**
  * @brief          设置电机CANID
  * @param[in]      Motor:  电机结构体
  * @param[in]      Motor:  设置新ID
  * @retval         none
  */
void set_CANID_cybergear(MI_Motor *Motor,uint8_t CAN_ID) {
	uint8_t tx_data[8]={0};
	FDCAN_TxHeaderTypeDef CAN_Tx;
	CAN_Tx.Identifier = Communication_Type_CanID<<24|CAN_ID<<16|Master_CAN_ID<<8|0x7F;
	CAN_Tx.IdType=FDCAN_EXTENDED_ID;
	CAN_Tx.TxFrameType=FDCAN_DATA_FRAME;
    Motor->CAN_ID = CAN_ID;//将新的ID导入电机结构体
    can_txd(CAN_Tx, tx_data);
}
/**
  * @brief          小米电机初始化
  * @param[in]      Motor:  电机结构体
  * @param[in]      Can_Id: 小米电机ID(默认0x7F)
  * @param[in]      Motor_Num: 电机编号
  * @param[in]      mode: 电机工作模式（0.运动模式Motion_mode 1. 位置模式Position_mode 2. 速度模式Speed_mode 3. 电流模式Current_mode）
  * @retval         none
  */
void init_cybergear(MI_Motor *Motor,uint8_t Can_Id, uint8_t mode) {
	Motor->CAN_ID=Can_Id;       //ID设置
	set_mode_cybergear(Motor,mode);//设置电机模式
	start_cybergear(Motor);        //使能电机
}

/**
  * @brief          小米运控模式指令
  * @param[in]      Motor:  目标电机结构体
  * @param[in]      torque: 力矩设置[-12,12] N*M
  * @param[in]      MechPosition: 位置设置[-12.5,12.5] rad
  * @param[in]      speed: 速度设置[-30,30] rpm
  * @param[in]      kp: 比例参数设置
  * @param[in]      kd: 微分参数设置
  * @retval         none
  */
void motor_controlmode(MI_Motor *Motor,float torque, float MechPosition, float speed, float kp, float kd) {
    uint8_t tx_data[8];//发送数据初始化
    FDCAN_TxHeaderTypeDef CAN_Tx;
    //装填发送数据
    tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
    tx_data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
    tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);

    CAN_Tx.Identifier = Communication_Type_MotionControl<<24|float_to_uint(torque,T_MIN,T_MAX,16)<<8|Motor->CAN_ID;//装填扩展帧数据
    can_txd(CAN_Tx, tx_data);
}

/*****************************回调函数 负责接回传信息 可转移至别处*****************************/
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */

void cybergear_rx_callback(uint8_t rxBuffer1[4], uint16_t rxBuffer2[4]) {
	uint8_t cybergear_rx_data[8] = {0};
	uint16_t cybergear_rec_id;
	uint8_t cybergear_rx_id = rxBuffer1[1];
	float debug = 0;

	//fdcanx_receive(&hfdcan1, &cybergear_rec_id, cybergear_rx_data);
	//Motor_Can_ID=Get_Motor_ID(cybergear_rec_id);
	switch(cybergear_rx_id)                   //将对应ID电机信息提取至对应结构体
	    {
		case 0X7F:
				mi_motor[0].MCU_ID = rx_data[0];
				mi_motor[0].Angle = rxBuffer2[0];   		 //todo: map 0~65535 to -4pi~4pi
				mi_motor[0].Speed = rxBuffer2[1]; 	 //todo: map 0~65535 to -30~30rad/s
				mi_motor[0].Torque = rxBuffer2[2];  	 //todo: map 0~65535 to -12Nm~12Nm
				mi_motor[0].Temp = rxBuffer2[3];  					 //todo: map to x10 C
				mi_motor[0].error_code = rxBuffer1[2];

		case 0x31:
				mi_motor[0].Angle = (rxBuffer2[0] - 32767);   		//todo: map 0~65535 to -4pi~4pi
				mi_motor[0].Speed = (rxBuffer2[1] - 32767)*30;    //todo: map 0~65535 to -30~30rad/s
				mi_motor[0].Torque = (rxBuffer2[2] - 32767)*12;   //todo: map 0~65535 to -12Nm~12Nm
				mi_motor[0].Temp = rxBuffer2[3]/10;						//todo: map to x10 C
				mi_motor[0].error_code = rxBuffer1[2];
				usart_printf("Recv in 0x31 = %d\r\n", mi_motor[0].Speed);
			break;
		default:
			break;
	}
}

/*
void HAL_CAN_RxFifo0MsgPendingCallback(hcan_t *hfdcan) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxMsg, rx_data);//接收数据
	Motor_Can_ID=Get_Motor_ID(rxMsg.ExtId);//首先获取回传电机ID信息
    switch(Motor_Can_ID)                   //将对应ID电机信息提取至对应结构体
    {
        case 0X7F:
            if(rxMsg.ExtId>>24 != 0)               //检查是否为广播模式
                Motor_Data_Handler(&mi_motor[0],rx_data,rxMsg.ExtId);
            else
                mi_motor[0].MCU_ID = rx_data[0];
            break;
        default:
            break;
    }
}*/


