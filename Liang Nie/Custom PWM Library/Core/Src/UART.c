/*
 * UART.c
 *
 *  Created on: Mar 15, 2024
 *      Author: EDY
 */

#include "UART.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "main.h"

static uint8_t RxBuff_1[256];
static uint8_t RxBuff_2[256];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

game_status_t game_status;
power_heat_data_t power_heat_data;
robot_status_t robot_status;
event_data_t event_data;
ext_supply_projectile_action_t ext_supply_projectile_action;
referee_warning_t referee_warning;
dart_info_t dart_info;

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
		    	game_status.game_type = RxBuff_2[7];
		    	game_status.game_progress = RxBuff_2[8];
		    	game_status.stage_remain_time = ((uint16_t) RxBuff_2[9] << 8) | RxBuff_2[10];
		    	game_status.SyncTimeStamp =
		    		((uint64_t) RxBuff_2[11] << 56) |
		    		((uint64_t) RxBuff_2[12] << 48) |
		    		((uint64_t) RxBuff_2[13] << 40) |
		    		((uint64_t) RxBuff_2[14] << 32) |
		    		((uint64_t) RxBuff_2[15] << 24) |
		    		((uint64_t) RxBuff_2[16] << 16) |
		    		((uint64_t) RxBuff_2[17] << 8) | RxBuff_2[18];
		    	//usart_printf("-\r\n");
		    	//usart_printf("Received status message: type = %d | progress = %d | time = %d | stamp = %d\r\n", game_status.game_type, game_status.game_progress, game_status.stage_remain_time, game_status.SyncTimeStamp);
		    	break;
		    }
		    case GAME_RESULT_HEADER : {
		    	game_status.game_type = RxBuff_2[7];
		    }
		    case ROBOT_HP_HEADER : {}
		    case EVENT_DATA_HEADER : {}
		    case PROJECTILE_SUPPLY_HEADER : {}
		    case REFEREE_WARNING_HEADER : {}
		    case DART_INFO_HEADER : {}
		    case ROBOT_HEADER : {
		    	robot_status.robot_id = RxBuff_2[7];
		    	robot_status.robot_level = RxBuff_2[8];
		    	robot_status.current_HP = ((uint16_t) RxBuff_2[9] << 8) | RxBuff_2[10];
		    	robot_status.maximum_HP = ((uint16_t) RxBuff_2[11] << 8) | RxBuff_2[12];
		    	robot_status.shooter_barrel_cooling_value = ((uint16_t) RxBuff_2[13] << 8) | RxBuff_2[14];
		    	robot_status.shooter_barrel_heat_limit = ((uint16_t) RxBuff_2[15] << 8) | RxBuff_2[16];
		    	robot_status.chassis_power_limit = ((uint16_t) RxBuff_2[17] << 8) | RxBuff_2[18];
		    	robot_status.power_management_gimbal_output = RxBuff_2[19];
		    	robot_status.power_management_chassis_output = RxBuff_2[20];
		    	robot_status.power_management_shooter_output = RxBuff_2[21];
		    	//usart_printf("got %d\r\n", robot_status.robot_id);
		    }
		    case POWER_HEAT_HEADER : {
		    	power_heat_data.chassis_voltage = ((uint16_t) RxBuff_2[8] << 8) | RxBuff_2[7];
		    	power_heat_data.chassis_current = ((uint16_t) RxBuff_2[10] << 8) | RxBuff_2[9];
		    	power_heat_data.chassis_power = ((uint32_t) RxBuff_2[14] << 24) | ((uint32_t) RxBuff_2[13] << 16) | ((uint32_t) RxBuff_2[12] << 8) | RxBuff_2[11];
		    	power_heat_data.buffer_energy = ((uint16_t) RxBuff_2[15] << 8) | RxBuff_2[16];
		    	power_heat_data.shooter_17mm_1_barrel_heat = ((uint16_t) RxBuff_2[18] << 8) | RxBuff_2[17];
		    	power_heat_data.shooter_17mm_2_barrel_heat = ((uint16_t) RxBuff_2[20] << 8) | RxBuff_2[19];
		    	power_heat_data.shooter_42mm_barrel_heat = ((uint16_t) RxBuff_2[22] << 8) | RxBuff_2[21];

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

