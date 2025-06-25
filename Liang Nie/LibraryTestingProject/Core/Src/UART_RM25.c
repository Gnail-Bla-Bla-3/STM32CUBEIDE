/*
 * UART.c
 *
 *  Created on: May 13, 2025
 *      Author: moose
 */

#include "UART_RM25.h"
#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

static uint8_t RxInitBuff[256] = {0};

static uint8_t RxBuff[256] = {0};

game_status_t game_status;
game_result_t game_result;
robot_HP_t robot_HP;
event_data_t event_data;
referee_warning_t referee_warning;

dart_info_t dart_info;

robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_position_t robot_position;
buffs_t buffs;

damage_data_t damage_data;
shoot_data_t shoot_data;
projectile_allowance_t projectile_allowance;
rfid_status_t rfid_status;
dart_client_cmd_t dart_client_cmd;
robot_positions_t robot_positions;
radar_mark_data_t radar_mark_data;
sentry_info_t sentry_info;
radar_info_t radar_info;
sentry_cmd_t sentry_cmd;
radar_cmd_t radar_cmd;

map_command_t map_command;
map_robot_data_t map_robot_data;
map_data_t map_data;
custom_info_t custom_info;

pc_control_t pc_control;
custom_controller_robot_t custom_controller_robot;
custom_client_data_t custom_client_data;

custom_message_t custom_message;



uint8_t mainHeaderOffset = 5;
uint8_t seq = 0x00;
uint8_t SOF = 0xA5;


unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e,
0x20, 0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2,
0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23,
0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03,
0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63,
0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18,
0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5,
0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9,
0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98,
0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a,
0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93,
0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d,
0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce,
0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c,
0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 0xab, 0xf5,
0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57,
0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77,
0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34,
0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a,
0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7,
0x89, 0x6b, 0x35
};


uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
{
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

/*
** Descriptions: CRC8 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8){

	uint32_t ucIndex;

	while (dwLength>0){
		ucIndex = ucCRC8^(*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
		dwLength--;
	}

	return(ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength){

uint8_t ucExpected = 0;

	if ((pchMessage == 0) || (dwLength <= 2)) {
		return 0;
	}

	ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);

	return ( ucExpected == pchMessage[dwLength-1] );
}


/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC){
	uint8_t chData;

	if (pchMessage == NULL){
		return 0xFFFF;
	}

	while(dwLength>0){
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
		dwLength--;
	}

	return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength){

	uint16_t wExpected = 0;

	if ((pchMessage == NULL) || (dwLength <= 2)){
		return 0;
	}

	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);

	return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}


/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength){
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
	return;
	}
	wCRC = Get_CRC16_Check_Sum ((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
	pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}

/*
** Descriptions: Send a message in fmt format over chosen uart instance
** Input: UART instance to send over, message to send in fmt format
** Output: message transmitted over uart
*/
void UART_Printf(UART_HandleTypeDef *huart,const char *fmt,...) {
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
    len = vsprintf((char *)tx_buf, fmt, ap);
    va_end(ap);
    HAL_UART_Transmit(huart, tx_buf, len,100);
}

/*
** Descriptions: Initialize the chosen uart instance using advanced reception service.
** 				 Must be called previously to enable HAL_UARTEx_RxEventCallback function
** Input: Target uart instance
*/
void UART_Ex_Init(UART_HandleTypeDef *huart) {
	HAL_UARTEx_ReceiveToIdle_DMA(huart, RxInitBuff, 256);

}

/*
** Descriptions: is triggered on recieve
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {

	//write uart to RxBuff until idle state
	HAL_UARTEx_ReceiveToIdle_DMA(huart, RxBuff, 256);

	//parse frame information
	uint16_t cmd_id = ((uint16_t) RxBuff[6] << 8) | RxBuff[5];
	uint16_t data_length = ((uint16_t) RxBuff[2] << 8) | RxBuff[1];
	uint16_t CRC16 = ((uint16_t) RxBuff[data_length+8] << 8) | RxBuff[data_length+7];

	//fills transmissionBuffer array with transmission bytes without CRC16
	uint8_t transmissionBuff[data_length+7];
	for(uint16_t i = 0; i<data_length+7;i++){
		transmissionBuff[i] = RxBuff[i];
	}

	uint16_t expected_CRC16 = Get_CRC16_Check_Sum(transmissionBuff,data_length+7, CRC_INIT);


	if(expected_CRC16 == CRC16){
		//seperate data
		uint8_t data[data_length];
		memcpy(data,&RxBuff[7],data_length);

		switch (cmd_id) {
			case GAME_STATUS_HEADER : {
				game_status.game_type = getBits(data[0],0,4);
				game_status.current_stage = getBits(data[0],4,4);
				memcpy(&game_status.stage_remain_time,&data[1],2);
				memcpy(&game_status.SyncTimeStamp,&data[3],8);
				GAME_STATUS_RxEventCallback(game_status.game_type,game_status.current_stage,game_status.game_progress,game_status.stage_remain_time,game_status.SyncTimeStamp);
				break;
			}
			case GAME_RESULT_HEADER : {
				game_result.winner=data[0];
				GAME_RESULT_RxEventCallback(game_result.winner);
				break;
			}
			case ROBOT_HP_HEADER : {
				memcpy(&robot_HP.red_1_robot_HP,&data[0],2);
				memcpy(&robot_HP.red_2_robot_HP,&data[2],2);
				memcpy(&robot_HP.red_3_robot_HP,&data[4],2);
				memcpy(&robot_HP.red_4_robot_HP,&data[6],2);
				memcpy(&robot_HP.red_7_robot_HP,&data[10],2);
				memcpy(&robot_HP.red_outpost_HP,&data[12],2);
				memcpy(&robot_HP.red_base_HP,&data[14],2);
				memcpy(&robot_HP.blue_1_robot_HP,&data[16],2);
				memcpy(&robot_HP.blue_2_robot_HP,&data[18],2);
				memcpy(&robot_HP.blue_3_robot_HP,&data[20],2);
				memcpy(&robot_HP.blue_4_robot_HP,&data[22],2);
				memcpy(&robot_HP.blue_7_robot_HP,&data[26],2);
				memcpy(&robot_HP.blue_outpost_HP,&data[28],2);
				memcpy(&robot_HP.blue_base_HP,&data[30],2);
				ROBOT_HP_HEADER_RxEventCallback(robot_HP.red_1_robot_HP, robot_HP.red_2_robot_HP, robot_HP.red_3_robot_HP, robot_HP.red_4_robot_HP, robot_HP.red_7_robot_HP, robot_HP.red_outpost_HP, robot_HP.red_base_HP, robot_HP.blue_1_robot_HP, robot_HP.blue_2_robot_HP, robot_HP.blue_3_robot_HP, robot_HP.blue_4_robot_HP, robot_HP.blue_7_robot_HP, robot_HP.blue_outpost_HP, robot_HP.blue_base_HP);
				break;
			}
			case EVENT_DATA_HEADER : {
				event_data.resupply_zone_1 = getBits(data[0],0,1);
				event_data.resupply_zone_2 = getBits(data[0],1,1);
				event_data.resupply_zone_3 = getBits(data[0],2,1);
				event_data.small_power_rune = getBits(data[0],3,1);
				event_data.large_power_rune = getBits(data[0],4,1);
				event_data.central_elevated_ground = getBits(data[0],5,2);
				event_data.trapezoid_elevated_ground = getBits(data[0],7,1) | (getBits(data[1],0,1)<<1);
				uint8_t last_dart_hit[] = {getBits(data[1],1,8),getBits(data[2],0,2)};
				memcpy(&event_data.last_dart_hit,last_dart_hit,2);
				event_data.last_dart_hit_target = getBits(data[2],2,3);
				event_data.central_buff = getBits(data[2],5,2);
				EVENT_DATA_RxEventCallback(event_data.resupply_zone_1, event_data.resupply_zone_2, event_data.resupply_zone_3, event_data.small_power_rune, event_data.large_power_rune, event_data.central_elevated_ground, event_data.trapezoid_elevated_ground, event_data.last_dart_hit, event_data.last_dart_hit_target, event_data.central_buff);
				break;
			}
			case REFEREE_WARNING_HEADER : {
				memcpy(&referee_warning.penalty,&data[0],1);
				memcpy(&referee_warning.offending_robot_id,&data[1],1);
				memcpy(&referee_warning.count,&data[2],1);
				REFEREE_WARNING_RxEventCallback(referee_warning.penalty,referee_warning.offending_robot_id,referee_warning.count);
				break;
			}
			case DART_INFO_HEADER : {
				memcpy(&dart_info.dart_remaining_time,&data[0],1);
				dart_info.last_dart_hit_target = getBits(data[1],0,3);
				dart_info.last_target_hit_count = getBits(data[1],3,3);
				dart_info.current_dart_target = getBits(data[1],6,2);
				DART_INFO_RxEventCallback(dart_info.dart_remaining_time,dart_info.last_dart_hit_target,dart_info.last_target_hit_count, dart_info.current_dart_target);
				break;
			}
			case ROBOT_STATUS_HEADER : {
				memcpy(&robot_status.robot_id,&data[0],1);
				memcpy(&robot_status.robot_level,&data[1],1);
				memcpy(&robot_status.current_HP,&data[2],2);
				memcpy(&robot_status.maximum_HP,&data[4],2);
				memcpy(&robot_status.shooter_barrel_cooling_value,&data[6],2);
				memcpy(&robot_status.shooter_barrel_heat_limit,&data[8],2);
				memcpy(&robot_status.chassis_power_limit,&data[10],2);
				robot_status.power_management_gimbal_output = getBits(data[12],0,1);
				robot_status.power_management_chassis_output = getBits(data[12],1,1);
				robot_status.power_management_shooter_output = getBits(data[12],2,1);
				ROBOT_STATUS_RxEventCallback(robot_status.robot_id, robot_status.robot_level, robot_status.current_HP, robot_status.maximum_HP, robot_status.shooter_barrel_cooling_value, robot_status.shooter_barrel_heat_limit, robot_status.chassis_power_limit, robot_status.power_management_gimbal_output, robot_status.power_management_chassis_output, robot_status.power_management_shooter_output);
				break;
			}
			case POWER_HEAT_DATA_HEADER : {
				memcpy(&power_heat_data.buffer_energy,&data[8],2);
				memcpy(&power_heat_data.shooter_17mm_1_barrel_heat,&data[10],2);
				memcpy(&power_heat_data.shooter_17mm_2_barrel_heat,&data[12],2);
				memcpy(&power_heat_data.shooter_42mm_barrel_heat,&data[14],2);
				POWER_HEAT_DATA_RxEventCallback(power_heat_data.buffer_energy, power_heat_data.shooter_17mm_1_barrel_heat, power_heat_data.shooter_17mm_2_barrel_heat, power_heat_data.shooter_42mm_barrel_heat);
				break;
			}
			case ROBOT_POSITION_HEADER : {
				memcpy(&robot_position.x,&data[0],4);
				memcpy(&robot_position.y,&data[0],4);
				memcpy(&robot_position.angle,&data[0],4);
				ROBOT_POSITION_RxEventCallback(robot_position.x, robot_position.y, robot_position.angle);
				break;
			}
			case BUFF_HEADER : {
				memcpy(&buffs.recovery_buff,&data[0],1);
				memcpy(&buffs.cooling_buff,&data[1],1);
				memcpy(&buffs.defence_buff,&data[2],1);
				memcpy(&buffs.vulnerability_buff,&data[3],1);
				memcpy(&buffs.attack_buff,&data[4],2);
				memcpy(&buffs.remaining_energy,&data[5],1);
				BUFF_RxEventCallback(buffs.recovery_buff, buffs.cooling_buff, buffs.defence_buff, buffs.vulnerability_buff, buffs.attack_buff, buffs.remaining_energy);
				break;
			}
			case DAMAGE_DATA_HEADER : {
				damage_data.armor_id = getBits(data[0],0,4);
				damage_data.HP_deduction_reason = getBits(data[0],4,4);
				DAMAGE_SOURCE_RxEventCallback(damage_data.armor_id, damage_data.HP_deduction_reason);
				break;
			}
			case PROJECTILE_INFO_HEADER : {
				memcpy(&shoot_data.bullet_type,&data[0],1);
				memcpy(&shoot_data.shooter_number,&data[1],1);
				memcpy(&shoot_data.launching_frequency,&data[2],1);
				memcpy(&shoot_data.initial_speed,&data[3],4);
				PROJECTILE_INFO_RxEventCallback(shoot_data.bullet_type, shoot_data.shooter_number, shoot_data.launching_frequency,shoot_data.initial_speed);
				break;
			}
			case PROJECTILE_ALLOWANCE_HEADER : {
				memcpy(&projectile_allowance.projectile_allowance_17mm,&data[0],2);
				memcpy(&projectile_allowance.projectile_allowance_42mm,&data[2],2);
				memcpy(&projectile_allowance.remaining_gold_coin,&data[4],2);
				PROJECTILE_ALLOWANCE_RxEventCallback(projectile_allowance.projectile_allowance_17mm, projectile_allowance.projectile_allowance_42mm, projectile_allowance.remaining_gold_coin);
				break;
			}
			case RFID_HEADER : {
				memcpy(&rfid_status.rfid_status,data,4);
				RFID_RxEventCallback(rfid_status.rfid_status);
				break;
			}
			case DART_COMMAND_HEADER : {
				memcpy(&dart_client_cmd.dart_launch_opening_status,&data[0],1);
				memcpy(&dart_client_cmd.target_change_remaining_time,&data[2],2);
				memcpy(&dart_client_cmd.latest_launch_cmd_remaining_time,&data[4],2);
				DART_COMMAND_RxEventCallback(dart_client_cmd.dart_launch_opening_status, dart_client_cmd.target_change_remaining_time, dart_client_cmd.latest_launch_cmd_remaining_time);
				break;
			}
			case TEAM_POSITION_HEADER : {
				memcpy(&robot_positions.hero_x,&data[0],4);
				memcpy(&robot_positions.hero_y,&data[4],4);
				memcpy(&robot_positions.engineer_x,&data[8],4);
				memcpy(&robot_positions.engineer_y,&data[12],4);
				memcpy(&robot_positions.standard_3_x,&data[16],4);
				memcpy(&robot_positions.standard_3_y,&data[20],4);
				memcpy(&robot_positions.standard_4_x,&data[24],4);
				memcpy(&robot_positions.standard_4_y,&data[28],4);
				TEAM_POSITION_RxEventCallback(robot_positions.hero_x, robot_positions.hero_y, robot_positions.engineer_x, robot_positions.engineer_y, robot_positions.standard_3_x, robot_positions.standard_3_y, robot_positions.standard_4_x, robot_positions.standard_4_y, robot_positions.standard_5_x, robot_positions.standard_5_y);
				break;
			}
			case RADAR_MARKING_HEADER : {
				radar_mark_data.mark_hero_progress = getBits(data[0],0,1);
				radar_mark_data.mark_engineer_progress = getBits(data[0],1,1);
				radar_mark_data.mark_standard_3_progress = getBits(data[0],2,1);
				radar_mark_data.mark_standard_4_progress = getBits(data[0],3,1);
				radar_mark_data.mark_sentry_progress = getBits(data[0],4,1);
				RADAR_MARKING_RxEventCallback(radar_mark_data.mark_hero_progress, radar_mark_data.mark_engineer_progress, radar_mark_data.mark_standard_3_progress, radar_mark_data.mark_standard_4_progress, radar_mark_data.mark_sentry_progress);
				break;
			}
			case SENTRY_HEADER : {
				uint8_t exchanged_projectiles_buff[2] = {data[0],getBits(data[1],0,3)};
				memcpy(&sentry_info.exchanged_projectiles,&exchanged_projectiles_buff,2);
				sentry_info.projectile_exchange_count = getBits(data[1],3,4);
				sentry_info.HP_exchange_count = getBits(data[1],7,1) | (getBits(data[2],0,3)<<1);
				sentry_info.confirm_free_respawn = getBits(data[2],3,1);
				sentry_info.instant_respawn_available = getBits(data[2],4,1);
				uint8_t instant_respawn_cost_buff[2] = {getBits(data[2],5,3)| (getBits(data[3],0,5)<<3),getBits(data[3],5,2)};
				memcpy(&sentry_info.instant_respawn_cost,instant_respawn_cost_buff,2);
				sentry_info.in_combat = getBits(data[4],0,1);
				uint8_t remaining_allowance_to_exchange_buff[2] = {getBits(data[4],1,7)|(getBits(data[5],0,1)<<7),getBits(data[5],1,3)};
				memcpy(&sentry_info.remaining_allowance_to_exchange,remaining_allowance_to_exchange_buff,2);
				SENTRY_RxEventCallback(sentry_info.exchanged_projectiles, sentry_info.projectile_exchange_count, sentry_info.HP_exchange_count, sentry_info.confirm_free_respawn, sentry_info.instant_respawn_available, sentry_info.instant_respawn_cost, sentry_info.in_combat, sentry_info.remaining_allowance_to_exchange);
				break;
			}
			case RADAR_BUFF_HEADER : {
				radar_info.double_vulnerability_chances = getBits(data[0],0,1);
				radar_info.double_vulnerability_triggered = getBits(data[0],1,1);
				RADAR_BUFF_RxEventCallback(radar_info.double_vulnerability_chances, radar_info.double_vulnerability_triggered);
				break;
			}

			//unfinished
			case ROBOT_INTERACTION_HEADER : {

				break;
			}
			case SENTRY_CMD_HEADER : {
				break;
			}
			case RADAR_CMD_HEADER : {
				break;
			}
			//end of unfinished

			case CUSTOM_CONTROLLER_ROBOT_HEADER : {
				memcpy(&custom_controller_robot.custom_data,data,30);
				CUSTOM_CONTROLLER_ROBOT_RxEventCallback(custom_controller_robot.custom_data);
				break;
			}
			case MINIMAP_COMMAND_HEADER : {
				memcpy(&map_command.target_position_x,&data[0],4);
				memcpy(&map_command.target_position_y,&data[4],4);
				memcpy(&map_command.cmd_keyboard,&data[8],1);
				memcpy(&map_command.target_robot_id,&data[9],1);
				memcpy(&map_command.cmd_source,&data[10],2);
				MINIMAP_COMMAND_RxEventCallback(map_command.target_position_x, map_command.target_position_y, map_command.cmd_keyboard, map_command.target_robot_id, map_command.cmd_source);
				break;
			}
			case PC_CONTROL_HEADER : {
				memcpy(&pc_control.mouse_x,&data[0],2);
				memcpy(&pc_control.mouse_y,&data[2],2);
				memcpy(&pc_control.mouse_z,&data[4],2);
				memcpy(&pc_control.left_button_down,&data[6],1);
				memcpy(&pc_control.right_button_down,&data[7],1);
				pc_control.w = getBits(data[8],0,1);
				pc_control.s = getBits(data[8],1,1);
				pc_control.a = getBits(data[8],2,1);
				pc_control.d = getBits(data[8],3,1);
				pc_control.shift = getBits(data[8],4,1);
				pc_control.ctrl = getBits(data[8],5,1);
				pc_control.q = getBits(data[8],6,1);
				pc_control.e = getBits(data[8],7,1);
				pc_control.r = getBits(data[9],0,1);
				pc_control.f = getBits(data[9],1,1);
				pc_control.g = getBits(data[9],2,1);
				pc_control.z = getBits(data[9],3,1);
				pc_control.x = getBits(data[9],4,1);
				pc_control.c = getBits(data[9],5,1);
				pc_control.v = getBits(data[9],6,1);
				pc_control.b = getBits(data[9],7,1);
				PC_CONTROL_RxEventCallback(pc_control.mouse_x,pc_control.mouse_y,pc_control.mouse_z,pc_control.left_button_down,pc_control.right_button_down,pc_control.w,pc_control.s,pc_control.a,pc_control.d,pc_control.shift,pc_control.ctrl,pc_control.q,pc_control.e,pc_control.r,pc_control.f,pc_control.g,pc_control.z,pc_control.x,pc_control.c,pc_control.v,pc_control.b);
				break;
			}
			case MINIMAP_TARGET_HEADER : {
				break;
			}
			case CUSTOM_CONTROLLER_CLIENT_HEADER : {
				break;
			}
			case MINIMAP_DATA_HEADER : {
				memcpy(&map_data.intention,&data[0],1);
				memcpy(&map_data.start_position_x,&data[1],2);
				memcpy(&map_data.start_position_y,&data[3],2);
				memcpy(&map_data.delta_x,&data[5],49);
				memcpy(&map_data.delta_y,&data[54],49);
				memcpy(&map_data.sender_id,&data[103],2);
				MINIMAP_DATA_RxEventCallback(map_data.intention, map_data.start_position_x, map_data.start_position_y, map_data.delta_x, map_data.delta_y, map_data.sender_id);
				break;
			}
			case CUSTOM_MESSAGE_HEADER : {
				//UART_Printf(&huart1,"CallBack Check");
				memcpy(&custom_message.custom_message,data,128);
				CUSTOM_MESSAGE_RxEventCallback(custom_message.custom_message);
				break;
			}
		}
	}
}

void UART_Transmit_RM(UART_HandleTypeDef *huart,uint16_t cmd_id, uint8_t data[]){

	uint16_t data_len = 0;
	switch(cmd_id){
		case(GAME_STATUS_HEADER):{
			data_len = game_status_len;
		}
		case(GAME_RESULT_HEADER):{
			data_len = game_result_len;
		}
		case(ROBOT_HP_HEADER):{
			data_len = game_robot_HP_len;
		}
		case(EVENT_DATA_HEADER):{
			data_len = event_data_len;
		}
		case(REFEREE_WARNING_HEADER):{
			data_len = referee_warning_len;
		}
		case(DART_INFO_HEADER):{
			data_len = dart_info_len;
		}
		case(ROBOT_STATUS_HEADER):{
			data_len = robot_status_len;
		}
		case(POWER_HEAT_DATA_HEADER):{
			data_len = power_heat_data_len;
		}
		case(ROBOT_POSITION_HEADER):{
			data_len = robot_pos_len;
		}
		case(BUFF_HEADER):{
			data_len = buff_len;
		}
		case(DAMAGE_DATA_HEADER):{
			data_len = damage_data_len;
		}
		case(PROJECTILE_INFO_HEADER):{
			data_len = shoot_data_len;
		}
		case(PROJECTILE_ALLOWANCE_HEADER):{
			data_len = projectile_allowance_len;
		}
		case(RFID_HEADER):{
			data_len = rfid_status_len;
		}
		case(DART_COMMAND_HEADER):{
			data_len = dart_client_cmd_len;
		}
		case(TEAM_POSITION_HEADER):{
			data_len = ground_robot_position_len;
		}
		case(RADAR_MARKING_HEADER):{
			data_len = radar_mark_data_len;
		}
		case(SENTRY_HEADER):{
			data_len = sentry_info_len;
		}
		case(RADAR_BUFF_HEADER):{
			data_len = radar_info_len;
		}
		case(ROBOT_INTERACTION_HEADER):{
			data_len = robot_interaction_data_len;
		}
		case(CUSTOM_CONTROLLER_ROBOT_HEADER):{
			data_len = custom_robot_data_len;
		}
		case(MINIMAP_COMMAND_HEADER):{
			data_len = map_command_len;
		}
		case(PC_CONTROL_HEADER):{
			data_len = pc_control_len;
		}
		case(MINIMAP_TARGET_HEADER):{
			data_len = 11;//not finished
		}
		case(CUSTOM_CONTROLLER_CLIENT_HEADER):{
			data_len = custom_client_data_len;
		}
		case(MINIMAP_DATA_HEADER):{
			data_len = map_data_len;
		}
		case(CUSTOM_INFO_HEADER):{
			data_len = custom_info_len;
		}
		case(SENTRY_CMD_HEADER):{
			data_len = sentry_cmd_len;
		}
		case(RADAR_CMD_HEADER):{
			data_len = radar_cmd_len;
		}
		case(CUSTOM_MESSAGE_HEADER):{
			data_len = custom_message_len;
		}
	}

	uint8_t headerBuff[] = {SOF, 0x00, 0x00,seq};
	memcpy(&headerBuff[1],&data_len,2);
	uint8_t CRC8 = Get_CRC8_Check_Sum(headerBuff,4,CRC8_INIT);


	uint8_t cmd_id_buffer[2];
	memcpy(cmd_id_buffer, &cmd_id, 2);

	uint8_t messageBuff[7+data_len];
	messageBuff[0] = headerBuff[0];
	messageBuff[1] = headerBuff[1];
	messageBuff[2] = headerBuff[2];
	messageBuff[3] = headerBuff[3];
	messageBuff[4] = CRC8;
	messageBuff[5] = cmd_id_buffer[0];
	messageBuff[6] = cmd_id_buffer[1];
	for(int i = 0; i<data_len;i++){
		messageBuff[7+i] = data[i];
	}
	//UART_Printf(huart,"CallBackCheck");
	uint16_t CRC16 = Get_CRC16_Check_Sum(messageBuff, data_len+7, CRC_INIT);
	uint8_t CRC16_buffer[2];
	memcpy(CRC16_buffer, &CRC16, 2);


	uint8_t message[9+data_len];
	message[0] = headerBuff[0];
	message[1] = headerBuff[1];
	message[2] = headerBuff[2];
	message[3] = headerBuff[3];
	message[4] = CRC8;
	message[5] = cmd_id_buffer[0];
	message[6] = cmd_id_buffer[1];
	for(int i = 0; i<data_len;i++){
		message[7+i] = data[i];
	}
	message[7+data_len] = CRC16_buffer[0];
	message[8+data_len] = CRC16_buffer[1];

	//HAL_UART_Transmit(huart, message, data_len + 9, 100);

	if(seq<255)
		seq++;
	else{
		seq=0;
	}
}







uint8_t getBits(uint8_t data, uint8_t startBit, uint8_t len){
	uint8_t result = 0;
	for(uint8_t index=0;index<len;index++){
		result = result | (((data>>(startBit+index))& 1)<<index);
	}
	return result;
}

__weak void GAME_STATUS_RxEventCallback(uint8_t game_type,uint8_t current_stage,uint8_t game_progress, uint16_t stage_remain_time,uint64_t SyncTimeStamp){
	//NOTE: This function should not be modified
}

__weak void GAME_RESULT_RxEventCallback(uint8_t winner){
	//NOTE: This function should not be modified
}

__weak void ROBOT_HP_HEADER_RxEventCallback(uint16_t red_1_robot_HP, uint16_t red_2_robot_HP, uint16_t red_3_robot_HP, uint16_t red_4_robot_HP, uint16_t red_7_robot_HP, uint16_t red_outpost_HP, uint16_t red_base_HP, uint16_t blue_1_robot_HP, uint16_t blue_2_robot_HP, uint16_t blue_3_robot_HP, uint16_t blue_4_robot_HP, uint16_t blue_7_robot_HP, uint16_t blue_outpost_HP, uint16_t blue_base_HP){
	//NOTE: This function should not be modified
}

__weak void EVENT_DATA_RxEventCallback(uint8_t resupply_zone_1, uint8_t resupply_zone_2, uint8_t resupply_zone_3, uint8_t small_power_rune, uint8_t large_power_rune, uint8_t central_elevated_ground, uint8_t trapezoid_elevated_ground, uint16_t last_dart_hit, uint8_t last_dart_hit_target, uint8_t central_buff){
	//NOTE: This function should not be modified
}

__weak void REFEREE_WARNING_RxEventCallback(uint8_t penalty,uint8_t offending_robot_id,uint8_t count){
	//NOTE: This function should not be modified
}

__weak void DART_INFO_RxEventCallback(uint8_t dart_remaining_time,uint8_t last_dart_hit_target,uint8_t last_target_hit_count, uint8_t current_dart_target){
	//NOTE: This function should not be modified
}

__weak void ROBOT_STATUS_RxEventCallback(uint8_t robot_id, uint8_t robot_level, uint16_t current_HP, uint16_t maximum_HP, uint16_t shooter_barrel_cooling_value, uint16_t shooter_barrel_heat_limit, uint16_t chassis_power_limit, uint8_t power_management_gimbal_output, uint8_t power_management_chassis_output, uint8_t power_management_shooter_output){
	//NOTE: This function should not be modified
}

__weak void POWER_HEAT_DATA_RxEventCallback(uint16_t buffer_energy, uint16_t shooter_17mm_1_barrel_heat, uint16_t shooter_17mm_2_barrel_heat, uint16_t shooter_42mm_barrel_heat){
	//NOTE: This function should not be modified
}

__weak void ROBOT_POSITION_RxEventCallback(float x, float y, float angle){
	//NOTE: This function should not be modified
}

__weak void BUFF_RxEventCallback(uint8_t recovery_buff, uint8_t cooling_buff, uint8_t defence_buff, uint8_t vulnerability_buff, uint16_t attack_buff, uint8_t remaining_energy){
	//NOTE: This function should not be modified
}

__weak void DAMAGE_SOURCE_RxEventCallback(uint8_t armor_id, uint8_t HP_deduction_reason){
	//NOTE: This function should not be modified
}

__weak void PROJECTILE_INFO_RxEventCallback(uint8_t bullet_type, uint8_t shooter_number, uint8_t launching_frequency,float initial_speed){
	//NOTE: This function should not be modified
}

__weak void PROJECTILE_ALLOWANCE_RxEventCallback(uint16_t projectile_allowance_17mm, uint16_t projectile_allowance_42mm, uint16_t remaining_gold_coin){
	//NOTE: This function should not be modified
}

__weak void RFID_RxEventCallback(uint32_t rfid_status){
	//NOTE: This function should not be modified
}

__weak void DART_COMMAND_RxEventCallback(uint8_t dart_launch_opening_status, uint16_t target_change_remaining_time, uint16_t latest_launch_cmd_remaining_time){
	//NOTE: This function should not be modified
}

__weak void TEAM_POSITION_RxEventCallback(float hero_x, float hero_y, float engineer_x, float engineer_y, float standard_3_x, float standard_3_y, float standard_4_x, float standard_4_y, float standard_5_x, float standard_5_y){
	//NOTE: This function should not be modified
}

__weak void RADAR_MARKING_RxEventCallback(uint8_t mark_hero_progress, uint8_t mark_engineer_progress, uint8_t mark_standard_3_progress, uint8_t mark_standard_4_progress, uint8_t mark_sentry_progress){
	//NOTE: This function should not be modified
}

__weak void SENTRY_RxEventCallback(uint16_t exchanged_projectiles, uint8_t projectile_exchange_count, uint8_t HP_exchange_count, uint8_t confirm_free_respawn, uint8_t instant_respawn_available, uint16_t instant_respawn_cost, uint8_t in_combat, uint16_t remaining_allowance_to_exchange){
	//NOTE: This function should not be modified
}

__weak void RADAR_BUFF_RxEventCallback(uint8_t double_vulnerability_chances, uint8_t double_vulnerability_triggered){
	//NOTE: This function should not be modified
}

__weak void ROBOT_INTERACTION_RxEventCallback(){
	//NOTE: This function should not be modified
}

__weak void CUSTOM_CONTROLLER_ROBOT_RxEventCallback(uint8_t customData[30]){
	//NOTE: This function should not be modified
}

__weak void MINIMAP_COMMAND_RxEventCallback(float target_position_x, float target_position_y, uint8_t cmd_keyboard, uint8_t target_robot_id, uint16_t cmd_source){
	//NOTE: This function should not be modified
}

__weak void PC_CONTROL_RxEventCallback(int16_t mouse_x, int16_t mouse_y, int16_t mouse_z, uint8_t left_button_down, uint8_t right_button_down, uint8_t w, uint8_t s, uint8_t a, uint8_t d, uint8_t shift,uint8_t ctrl,uint8_t q, uint8_t e, uint8_t r, uint8_t f, uint8_t g, uint8_t z, uint8_t x, uint8_t c, uint8_t v, uint8_t b){
	//NOTE: This function should not be modified
}

__weak void MINIMAP_TARGET_RxEventCallback(uint16_t target_robot_id, float target_position_x, float target_position_y){
	//NOTE: This function should not be modified
}

__weak void CUSTOM_CONTROLLER_CLIENT_RxEventCallback(uint16_t key_value, uint16_t x_position, uint16_t mouse_left, uint16_t y_position, uint16_t mouse_right){
	//NOTE: This function should not be modified
}

__weak void MINIMAP_DATA_RxEventCallback(uint8_t intention, uint16_t start_position_x, uint16_t start_position_y, int8_t delta_x[49], int8_t delta_y[49], uint16_t sender_id){
	//NOTE: This function should not be modified
}

__weak void CUSTOM_MESSAGE_RxEventCallback(uint8_t customMessage[128]){
	//NOTE: This function should not be modified
}


