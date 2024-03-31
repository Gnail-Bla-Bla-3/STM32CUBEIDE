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
remote_control_t remote_control;
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
		uint16_t RxBuff16 = ((uint16_t) RxBuff_2[5] << 8) | RxBuff_2[6];
		//usart_printf("callback triggered at %d | %d | %d | %d || %d \r\n", RxBuff_2[0], RxBuff_2[1], RxBuff_2[2], RxBuff_2[3], RxBuff16);
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
		    case PROJECTILE_SUPPLY_HEADER : {}
		    case REFEREE_WARNING_HEADER : {}
		    case DART_INFO_HEADER : {}
		    case ROBOT_HEADER : {
		    	memcpy(&robot_status, (RxBuff_2 + frame_header_offset), robot_status_len);
		    }
		    case POWER_HEAT_HEADER : {
		    	memcpy(&power_heat_data, (RxBuff_2 + frame_header_offset), power_heat_data_len);
		    }
		    case POSITION_HEADER: {}
		    case BUFF_HEADER: {}
		    case AIR_SUPPORT_HEADER: {}
		    case DAMAGE_SOURCE_HEADER: {}
		    case PROJECTILE_INFO_HEADER: {}
		    case PROJECTILE_ALLOWANCE_HEADER: {}
		    case RFID_HEADER: {}
		    case DART_COMMAND_HEADER: {}
		    case TEAM_POSITION_HEADER: {}
		    case RADAR_MARKING_HEADER: {}
		    case SENTRY_HEADER: {}
		    case RADAR_BUFF_HEADER: {}
		    case ROBOT_INTERACTION_HEADER: {}
		    case CUSTOM_CONTROLLER_ROBOT_HEADER: {}
		    case MINIMAP_COMMAND_HEADER: {}
		    case CONTROLLER_HEADER: {}
		    case MINIMAP_TARGET_HEADER: {}
		    case CUSTOM_CONTROLLER_CLIENT_HEADER: {}
		    case MINIMAP_DATA_HEADER: {}
		    case CUSTOM_INFO_HEADER: {}

		    default: {
		    	//usart_printf("no match \r\n");
		        break;
		    }
		}
	}
}

