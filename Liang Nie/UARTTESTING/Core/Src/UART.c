/*
 * UART.c
 *
 *  Created on: Mar 15, 2024
 *      Author: EDY
 */

#include "UART.h"
/*
#include "remote_control.h"
#include "bsp_rc.h"
#include "CAN.h"
*/
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
    HAL_UART_Transmit(&huart6, tx_buf, len, 50);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
/*
	if (1) {
			HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RxBuff_2, 256);
			//usart_printf("beanus");
			uint16_t RxBuff16 = ((uint16_t) RxBuff_2[6] << 8) | RxBuff_2[5];

			switch (RxBuff16) {
					    case GAME_STATUS_HEADER : {
					    	//memcpy(&game_status, (RxBuff_2 + frame_header_offset), game_status_len);
					    	break;
					    }
					    case GAME_RESULT_HEADER : {
					    	//memcpy(&game_result, (RxBuff_2 + frame_header_offset), game_result_len);
					    	break;
					    }
					    case ROBOT_HP_HEADER : {
					    	//memcpy(&robot_status, (RxBuff_2 + frame_header_offset), robot_status_len);
					    	break;
					    }
					    case EVENT_DATA_HEADER : {
					    	//memcpy(&robot_status, (RxBuff_2 + frame_header_offset), robot_status_len);
					    	break;
					    }
					    //case PROJECTILE_SUPPLY_HEADER : {
					    	//memcpy(&ext_supply_projectile_action, (RxBuff_2 + frame_header_offset), ext_supply_projectile_action_len);
					    	//break;
					    //}
					    case REFEREE_WARNING_HEADER : {
					    	//memcpy(&referee_warning, (RxBuff_2 + frame_header_offset), referee_warning_len);
					    	break;
					    }
					    case DART_INFO_HEADER : {
					    	//memcpy(&dart_info, (RxBuff_2 + frame_header_offset), dart_info_len);
					    	break;
					    }
					    case ROBOT_HEADER : {

					    	//memcpy(&robot_status, (RxBuff_2 + frame_header_offset), robot_status_len);
					    	uint64_t statusData[2] = {0, 0};
					    	memcpy(&statusData[0], (RxBuff_2 + frame_header_offset), 8);
					    	memcpy(&statusData[1], (RxBuff_2 + frame_header_offset + 8), 8);
					    	CAN_transmit(Bus1, CAN_STATUS_1_ID, statusData[0]);
					    	CAN_transmit(Bus1, CAN_STATUS_2_ID, statusData[1]);
					    	break;
					    }
					    case POWER_HEAT_HEADER : {
					    	memcpy(&power_heat_data, (RxBuff_2 + frame_header_offset), power_heat_data_len);
					    	uint64_t powerHeatData[2] = {0, 0};
					    	memcpy(&powerHeatData[0], (RxBuff_2 + frame_header_offset), 8);
					    	memcpy(&powerHeatData[1], (RxBuff_2 + frame_header_offset + 8), 8);
					    	CAN_transmit(Bus1, CAN_POWER_ID, powerHeatData[0]);
					    	CAN_transmit(Bus1, CAN_HEAT_ID, powerHeatData[1]);
					    	break;
					    }
					    case POSITION_HEADER: {
					    	//memcpy(&robot_position, (RxBuff_2 + frame_header_offset), robot_pos_len);
					    	break;
					    }// edit below
					    case BUFF_HEADER: {
					    	//memcpy(&buffs, (RxBuff_2 + frame_header_offset), buff_len);
					    	break;
					    }
					    case AIR_SUPPORT_HEADER: {
					    	//memcpy(&robot_position, (RxBuff_2 + frame_header_offset), robot_pos_len);
					    	break;
					    }
					    case DAMAGE_SOURCE_HEADER: {
					    	//memcpy(&damage_data, (RxBuff_2 + frame_header_offset), damage_data_len);
					    	break;
					    }
					    case PROJECTILE_INFO_HEADER: {
					    	//memcpy(&shoot_data, (RxBuff_2 + frame_header_offset), shoot_data_len);
					    	break;
					    }
					    case PROJECTILE_ALLOWANCE_HEADER: {
					    	//memcpy(&projectile_allowance, (RxBuff_2 + frame_header_offset), projectile_allowance_len);
					    	break;
					    }
					    case RFID_HEADER: {
					    	//memcpy(&rfid_status, (RxBuff_2 + frame_header_offset), rfid_status_len);
					    	break;
					    }
					    case DART_COMMAND_HEADER: {
					    	//memcpy(&dart_client_cmd, (RxBuff_2 + frame_header_offset), dart_client_cmd_len);
					    	break;
					    }
					    case TEAM_POSITION_HEADER: {
					    	//memcpy(&robot_positions, (RxBuff_2 + frame_header_offset), ground_robot_position_len);
					    	break;
					    }
					    case RADAR_MARKING_HEADER: {
					    	//memcpy(&radar_mark_data, (RxBuff_2 + frame_header_offset), radar_mark_data_len);
					    	break;
					    }
					    case SENTRY_HEADER: {
					    	//memcpy(&sentry_info, (RxBuff_2 + frame_header_offset), sentry_info_len);
					    	break;
					    }
					    case RADAR_BUFF_HEADER: {
					    	//memcpy(&radar_info, (RxBuff_2 + frame_header_offset), radar_info_len);
					    	break;
					    }
					    case ROBOT_INTERACTION_HEADER: {
					    	//memcpy(&robot_position, (RxBuff_2 + frame_header_offset), robot_interaction_data_len);
					    	break;
					    }default: {
					    	//usart_printf("no match \r\n");
					        break;
					    }
			}

	}
	*/

}

