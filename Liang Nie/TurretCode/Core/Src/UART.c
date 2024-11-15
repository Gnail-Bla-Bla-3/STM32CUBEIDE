/*
 * UART.c
 *
 *  Created on: Mar 15, 2024
 *      Author: EDY
 */

#include "UART.h"
#include "remote_control.h"
#include "bsp_rc.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "main.h"

static uint8_t RxBuff_1[256];
static uint8_t RxBuff_2[256];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

game_status_t game_status;
game_result_t game_result;
game_robot_HP_t robot_HP;
event_data_t event_data;
ext_supply_projectile_action_t ext_supply_projectile_action;
referee_warning_t referee_warning;
dart_info_t dart_info;
robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_pos_t robot_position;
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
custom_client_data_t custom_client_data;

uint8_t mainHeaderOffset = 5;

void usart_Init(void) {
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuff_1, 256);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RxBuff_2, 256);
}

void usart_printf(const char *fmt,...) {
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
    len = vsprintf((char *)tx_buf, fmt, ap);           //return length of string
    va_end(ap);
    HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart6) {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RxBuff_2, 256);
		uint16_t RxBuff16 = ((uint16_t) RxBuff_2[6] << 8) | RxBuff_2[5];
		//usart_printf("callback triggered at %d | %d | %d | %d || %d \r\n", RxBuff_2[0], RxBuff_2[1], RxBuff_2[2], RxBuff_2[3], RxBuff16);
		//usart_printf("test");
		switch (RxBuff16) {
		    case GAME_STATUS_HEADER : {
		    	memcpy(&game_status, (RxBuff_2 + frame_header_offset), game_status_len);
		    }
		    case GAME_RESULT_HEADER : {
		    	memcpy(&game_result, (RxBuff_2 + frame_header_offset), game_result_len);
		    }
		    case ROBOT_HP_HEADER : {
		    	memcpy(&robot_status, (RxBuff_2 + frame_header_offset), robot_status_len);
		    }
		    case EVENT_DATA_HEADER : {
		    	//memcpy(&robot_status, (RxBuff_2 + frame_header_offset), robot_status_len);
		    }
		    case PROJECTILE_SUPPLY_HEADER : {
		    	memcpy(&ext_supply_projectile_action, (RxBuff_2 + frame_header_offset), ext_supply_projectile_action_len);
		    }
		    case REFEREE_WARNING_HEADER : {
		    	memcpy(&referee_warning, (RxBuff_2 + frame_header_offset), referee_warning_len);
		    }
		    case DART_INFO_HEADER : {
		    	memcpy(&dart_info, (RxBuff_2 + frame_header_offset), dart_info_len);
		    }
		    case ROBOT_HEADER : {
		    	memcpy(&robot_status, (RxBuff_2 + frame_header_offset), robot_status_len);
		    }
		    case POWER_HEAT_HEADER : {
		    	memcpy(&power_heat_data, (RxBuff_2 + frame_header_offset), power_heat_data_len);
		    }
		    case POSITION_HEADER: {
		    	memcpy(&robot_position, (RxBuff_2 + frame_header_offset), robot_pos_len);
		    }// edit below
		    case BUFF_HEADER: {
		    	memcpy(&buffs, (RxBuff_2 + frame_header_offset), buff_len);
		    }
		    case AIR_SUPPORT_HEADER: {
		    	//memcpy(&robot_position, (RxBuff_2 + frame_header_offset), robot_pos_len);
		    }
		    case DAMAGE_SOURCE_HEADER: {
		    	memcpy(&damage_data, (RxBuff_2 + frame_header_offset), damage_data_len);
		    }
		    case PROJECTILE_INFO_HEADER: {
		    	memcpy(&shoot_data, (RxBuff_2 + frame_header_offset), shoot_data_len);
		    }
		    case PROJECTILE_ALLOWANCE_HEADER: {
		    	memcpy(&projectile_allowance, (RxBuff_2 + frame_header_offset), projectile_allowance_len);
		    }
		    case RFID_HEADER: {
		    	memcpy(&rfid_status, (RxBuff_2 + frame_header_offset), rfid_status_len);
		    }
		    case DART_COMMAND_HEADER: {
		    	memcpy(&dart_client_cmd, (RxBuff_2 + frame_header_offset), dart_client_cmd_len);
		    }
		    case TEAM_POSITION_HEADER: {
		    	memcpy(&robot_positions, (RxBuff_2 + frame_header_offset), ground_robot_position_len);
		    }
		    case RADAR_MARKING_HEADER: {
		    	memcpy(&radar_mark_data, (RxBuff_2 + frame_header_offset), radar_mark_data_len);
		    }
		    case SENTRY_HEADER: {
		    	memcpy(&sentry_info, (RxBuff_2 + frame_header_offset), sentry_info_len);
		    }
		    case RADAR_BUFF_HEADER: {
		    	memcpy(&radar_info, (RxBuff_2 + frame_header_offset), radar_info_len);
		    }
		    case ROBOT_INTERACTION_HEADER: {
		    	memcpy(&robot_position, (RxBuff_2 + frame_header_offset), robot_interaction_data_len);
		    }
		    case CUSTOM_CONTROLLER_ROBOT_HEADER: {
		    	//memcpy(&robot_position, (RxBuff_2 + frame_header_offset), custom_info_len);
		    }
		    case MINIMAP_COMMAND_HEADER: {
		    	//memcpy(&robot_position, (RxBuff_2 + frame_header_offset), map_command_len);
		    }
		    case PC_CONTROL_HEADER: {
		    	memcpy(&pc_control, (RxBuff_2 + frame_header_offset), pc_control_len);
		        if(pc_control.keyboard_values_1 > 127){
		        	pc_control.e = 1;
		        	pc_control.keyboard_values_1= pc_control.keyboard_values_1 - 128;
		        }else{
		        	pc_control.e = 0;
		        }
		        if(pc_control.keyboard_values_1>63){
		        	pc_control.q = 1;
		            pc_control.keyboard_values_1= pc_control.keyboard_values_1 - 64;
		        }else{
		        	pc_control.q = 0;
		        }
		        if(pc_control.keyboard_values_1>31){
		        	pc_control.ctrl = 1;
		            pc_control.keyboard_values_1= pc_control.keyboard_values_1 - 32;
		        }else{
		        	pc_control.ctrl = 0;
		        }
		        if(pc_control.keyboard_values_1>15){
		        	pc_control.shift = 1;
		            pc_control.keyboard_values_1= pc_control.keyboard_values_1 - 16;
		        }else{
		        	pc_control.shift = 0;
		        }
		        if(pc_control.keyboard_values_1>7){
		        	pc_control.d = 1;
		            pc_control.keyboard_values_1= pc_control.keyboard_values_1 - 8;
		        }else{
		        	pc_control.d = 0;
		        }
		        if(pc_control.keyboard_values_1>3){
		        	pc_control.a = 1;
		            pc_control.keyboard_values_1= pc_control.keyboard_values_1 - 4;
		        }else{
		        	pc_control.a = 0;
		        }
		        if(pc_control.keyboard_values_1>1){
		        	pc_control.s = 1;
		            pc_control.keyboard_values_1= pc_control.keyboard_values_1 - 2;
		        }else{
		        	pc_control.s = 0;
		        }
		        if(pc_control.keyboard_values_1 > 0){
		        	pc_control.w = 1;
		        }else{
		        	pc_control.w = 0;
		        }

		        if(pc_control.keyboard_values_2 > 127){
					pc_control.b = 1;
					pc_control.keyboard_values_2= pc_control.keyboard_values_2 - 128;
				}else{
					pc_control.b = 0;
				}
				if(pc_control.keyboard_values_2>63){
					pc_control.v = 1;
					pc_control.keyboard_values_2= pc_control.keyboard_values_2 - 64;
				}else{
					pc_control.v = 0;
				}
				if(pc_control.keyboard_values_2>31){
					pc_control.c = 1;
					pc_control.keyboard_values_2= pc_control.keyboard_values_2 - 32;
				}else{
					pc_control.c = 0;
				}
				if(pc_control.keyboard_values_2>15){
					pc_control.x = 1;
					pc_control.keyboard_values_2= pc_control.keyboard_values_2 - 16;
				}else{
					pc_control.x = 0;
				}
				if(pc_control.keyboard_values_2>7){
					pc_control.z = 1;
					pc_control.keyboard_values_2= pc_control.keyboard_values_2 - 8;
				}else{
					pc_control.z = 0;
				}
				if(pc_control.keyboard_values_2>3){
					pc_control.g = 1;
					pc_control.keyboard_values_2= pc_control.keyboard_values_2 - 4;
				}else{
					pc_control.g = 0;
				}
				if(pc_control.keyboard_values_2>1){
					pc_control.f = 1;
					pc_control.keyboard_values_2= pc_control.keyboard_values_2 - 2;
				}else{
					pc_control.f = 0;
				}
				if(pc_control.keyboard_values_2 > 0){
					pc_control.r = 1;
				}else{
					pc_control.r = 0;
				}

		    }
		    case MINIMAP_TARGET_HEADER: {
		    	//memcpy(&robot_position, (RxBuff_2 + frame_header_offset), robot_pos_len);
		    }
		    case CUSTOM_CONTROLLER_CLIENT_HEADER: {
		    	//memcpy(&robot_position, (RxBuff_2 + frame_header_offset), robot_pos_len);
		    }
		    case MINIMAP_DATA_HEADER: {
		    	memcpy(&robot_position, (RxBuff_2 + frame_header_offset), map_data_len);
		    }
		    case CUSTOM_INFO_HEADER: {
		    	//memcpy(&robot_position, (RxBuff_2 + frame_header_offset), robot_pos_len);
		    }

		    default: {
		    	//usart_printf("no match \r\n");
		        break;
		    }
		}
	}
}

