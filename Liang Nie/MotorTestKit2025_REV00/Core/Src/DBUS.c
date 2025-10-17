/*
 * DBUS.c
 *
 *  Created on: May 13, 2025
 *      Author: moose
 */

#include "DBUS.h"
#include "main.h"

DR16_DBUS_t DR16_DBUS;
uint8_t DBUS_Buff[128];

uint8_t DBUS_getBits(uint8_t data, uint8_t startBit, uint8_t len);


void DBUS_Rx_Init(UART_HandleTypeDef* huart){
	HAL_UART_Receive_DMA (huart, DBUS_Buff, 18);
	DR16_DBUS.CH0 = 1024;
	DR16_DBUS.CH1 = 1024;
	DR16_DBUS.CH2 = 1024;
	DR16_DBUS.CH3 = 1024;
	DR16_DBUS.RESERVED = 1024;
}


void DBUS_Update(){
	DR16_DBUS.CH0 = (DBUS_Buff[0] | (DBUS_Buff[1] << 8)) & 0x07ff;        //!< Channel 0
	DR16_DBUS.CH1 = ((DBUS_Buff[1] >> 3) | (DBUS_Buff[2] << 5)) & 0x07ff; //!< Channel 1
	DR16_DBUS.CH2 = ((DBUS_Buff[2] >> 6) | (DBUS_Buff[3] << 2) | (DBUS_Buff[4] << 10)) &0x07ff;       //!< Channel 2
	DR16_DBUS.CH3 = ((DBUS_Buff[4] >> 1) | (DBUS_Buff[5] << 7)) & 0x07ff; //!< Channel 3
	DR16_DBUS.S1 = ((DBUS_Buff[5] >> 4) & 0x0003);                  //!< Switch left
	DR16_DBUS.S2 = ((DBUS_Buff[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
	DR16_DBUS.MouseX = DBUS_Buff[6] | (DBUS_Buff[7] << 8);                    //!< Mouse X axis
	DR16_DBUS.MouseY = DBUS_Buff[8] | (DBUS_Buff[9] << 8);                    //!< Mouse Y axis
	DR16_DBUS.MouseZ = DBUS_Buff[10] | (DBUS_Buff[11] << 8);                  //!< Mouse Z axis
	DR16_DBUS.MouseL = DBUS_Buff[12];                                  //!< Mouse Left Is Pressed ?
	DR16_DBUS.MouseR = DBUS_Buff[13];                                  //!< Mouse Right Is Pressed ?
	DR16_DBUS.KeyData1 = DBUS_Buff[14];                    //!< KeyBoard value lower bits
	DR16_DBUS.KeyData2 = DBUS_Buff[15];                    //!< KeyBoard value higher bits
	DR16_DBUS.RESERVED = DBUS_Buff[16] | (DBUS_Buff[17] << 8);                 //NULL
}

int16_t getDR16_CH0(){
	if(DR16_DBUS.CH0>0){
		return (int16_t)DR16_DBUS.CH0-1024;
	}
	else{
		return 0;
	}
}

int16_t getDR16_CH1(){
	if(DR16_DBUS.CH1>0){
		return (int16_t)DR16_DBUS.CH1-1024;
	}
	else{
		return 0;
	}
}

int16_t getDR16_CH2(){
	if(DR16_DBUS.CH2>0){
		return (int16_t)DR16_DBUS.CH2-1024;
	}
	else{
		return 0;
	}
}

int16_t getDR16_CH3(){
	if(DR16_DBUS.CH3>0){
		return (int16_t)DR16_DBUS.CH3-1024;
	}
	else{
		return 0;
	}
}

uint8_t getDR16_S1(){
	return DR16_DBUS.S1;
}

uint8_t getDR16_S2(){
	return DR16_DBUS.S2;
}

int16_t getDR16_CH4(){
	if(DR16_DBUS.RESERVED>0){
		return (int16_t)DR16_DBUS.RESERVED-1024;
	}
	else{
		return 0;
	}
}

uint8_t getDR16_W(){
	return DBUS_getBits(DR16_DBUS.KeyData1,0,1);
}

uint8_t getDR16_S(){
	return DBUS_getBits(DR16_DBUS.KeyData1,1,1);
}

uint8_t getDR16_A(){
	return DBUS_getBits(DR16_DBUS.KeyData1,2,1);
}

uint8_t getDR16_D(){
	return DBUS_getBits(DR16_DBUS.KeyData1,3,1);
}

uint8_t getDR16_Q(){
	return DBUS_getBits(DR16_DBUS.KeyData1,4,1);
}

uint8_t getDR16_E(){
	return DBUS_getBits(DR16_DBUS.KeyData1,5,1);
}

uint8_t getDR16_Shift(){
	return DBUS_getBits(DR16_DBUS.KeyData1,6,1);
}

uint8_t getDR16_Ctrl(){
	return DBUS_getBits(DR16_DBUS.KeyData1,7,1);
}

uint8_t getDR16_R(){
	return DBUS_getBits(DR16_DBUS.KeyData2,0,1); //r
}

uint8_t getDR16_F(){
	return DBUS_getBits(DR16_DBUS.KeyData2,1,1); //f
}

uint8_t getDR16_G(){
	return DBUS_getBits(DR16_DBUS.KeyData2,2,1); //g
}

uint8_t getDR16_Z(){
	return DBUS_getBits(DR16_DBUS.KeyData2,3,1); //z
}

uint8_t getDR16_X(){
	return DBUS_getBits(DR16_DBUS.KeyData2,4,1); //x
}

uint8_t getDR16_C(){
	return DBUS_getBits(DR16_DBUS.KeyData2,5,1); //c
}

uint8_t getDR16_V(){
	return DBUS_getBits(DR16_DBUS.KeyData2,6,1); //v
}

uint8_t getDR16_B(){
	return DBUS_getBits(DR16_DBUS.KeyData2,7,1); //b
}

//mouse
int16_t getDR16_MouseX(){
	return DR16_DBUS.MouseX;
}

int16_t getDR16_MouseY(){
	return DR16_DBUS.MouseY;
}

int16_t getDR16_MouseZ(){
	return DR16_DBUS.MouseZ;
}

uint8_t getDR16_MouseL(){
	return DR16_DBUS.MouseL;
}

uint8_t getDR16_MouseR(){
	return DR16_DBUS.MouseR;
}


uint8_t DBUS_getBits(uint8_t data, uint8_t startBit, uint8_t len){
	uint8_t result = 0;
	for(uint8_t index=0;index<len;index++){
		result = result | (((data>>(startBit+index))& 1)<<index);
	}
	return result;
}


