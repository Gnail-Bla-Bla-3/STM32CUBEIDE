/*
 * UART.h
 *
 *  Created on: Mar 15, 2024
 *      Author: EDY
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#include "main.h"
#include "remote_control.h"
#include "bsp_rc.h"

typedef enum
{
	//frame header definitions for the referee system's UART outputs
    GAME_STATUS_HEADER = 0x0001,
	GAME_RESULT_HEADER = 0x0002,
	ROBOT_HP_HEADER = 0x0003,
	EVENT_DATA_HEADER = 0x0101,
	PROJECTILE_SUPPLY_HEADER = 0x0102,
	REFEREE_WARNING_HEADER = 0x0104,
	DART_INFO_HEADER = 0x0105,
	ROBOT_HEADER = 0x0201,
	POWER_HEAT_HEADER = 0x0202,
	POSITION_HEADER = 0x0203,
	BUFF_HEADER = 0x0204,
	AIR_SUPPORT_HEADER = 0x0205,
	DAMAGE_SOURCE_HEADER = 0x0206,
	PROJECTILE_INFO_HEADER = 0x0207,
	PROJECTILE_ALLOWANCE_HEADER = 0x0208,
	RFID_HEADER = 0x0209,
	DART_COMMAND_HEADER = 0x020A,
	TEAM_POSITION_HEADER = 0x020B,
	RADAR_MARKING_HEADER = 0x020C,
	SENTRY_HEADER = 0x020D,
	RADAR_BUFF_HEADER = 0x020E,
	ROBOT_INTERACTION_HEADER = 0x0301,
	CUSTOM_CONTROLLER_ROBOT_HEADER = 0x0302,
	MINIMAP_COMMAND_HEADER = 0x0303,
	CONTROLLER_HEADER = 0x0304,
	MINIMAP_TARGET_HEADER = 0x0305,
	CUSTOM_CONTROLLER_CLIENT_HEADER = 0x0306,
	MINIMAP_DATA_HEADER = 0x0307,
	CUSTOM_INFO_HEADER = 0x0308,

	// sub-content IDs for 0x0301
	UI_DELETE_HEADER = 0x0100,
	UI_EDIT_1_HEADER = 0x0101,
	UI_EDIT_3_HEADER = 0x0102,
	UI_EDIT_5_HEADER = 0x0103,
	UI_EDIT_7_HEADER = 0x0104,
	UI_PRINT_CHAR_3_HEADER = 0x0110,
	SENTRY_CMD_HEADER = 0x0120,
	RADAR_CMD_HEADER = 0x0121,

} uart_headers;

typedef enum {
	frame_header_offset = 7,

	game_status_len = 11,
	game_result_len = 1,
	game_robot_HP_len = 32,
	event_data_len = 4,
	ext_supply_projectile_action_len = 4,
	referee_warning_len = 3,
	dart_info_len = 3,
	robot_status_len = 13,
	power_heat_data_len = 16,
	robot_pos_len = 16,
	buff_len = 6,


	hurt_data_len = 1,
	shoot_data_len = 7,
	projectile_allowance_len = 6,
	rfid_status_len = 4,
	dart_client_cmd_len = 6,
	ground_robot_position_len = 40,
	radar_mark_data_len = 6,
	sentry_info_len = 4,
	radar_info_len = 1,
	robot_interaction_data_len = 128,
	interaction_layer_delete_len = 1,
	interaction_figure_len = 1,
	interaction_figure_2_len = 1,
	interaction_figure_3_len = 1,
	interaction_figure_4_len = 1,
	ext_client_custom_character_len = 1,

	sentry_cmd_len = 103,
	radar_cmd_len = 32,
	map_command_len = 15,
	map_robot_data_len = 10,
	map_data_len = 103,
	custom_info_len = 30,
	remote_control_len = 12,
	custom_client_data_len = 8,

} lengths_and_offsets_t;

typedef struct {
	uint8_t game_type;
	uint8_t game_progress;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} game_status_t;

typedef struct {
	uint8_t winner;
} game_result_t;

typedef struct {
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} game_robot_HP_t;

typedef struct {
	uint32_t event_data;
} event_data_t;

typedef struct {
	uint8_t reserved;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef struct {
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
} referee_warning_t;

typedef struct {
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} dart_info_t;

typedef struct {
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
} robot_status_t;

typedef struct {
	uint16_t chassis_voltage;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

typedef struct {
	float x;
	float y;
	float angle;
} robot_pos_t;

typedef struct {
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
} buffs_t;

typedef struct {
	uint8_t armor_id : 4;
	uint8_t HP_deduction_reason : 4;
} damage_data_t;

typedef struct {
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
} shoot_data_t;

typedef struct {
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
} projectile_allowance_t;

typedef struct {
	uint32_t rfid_status;
} rfid_status_t;

typedef struct {
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

typedef struct {
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float standard_5_x;
	float standard_5_y;
} robot_positions_t;

typedef struct {
	uint8_t mark_hero_progress;
	uint8_t mark_engineer_progress;
	uint8_t mark_standard_3_progress;
	uint8_t mark_standard_4_progress;
	uint8_t mark_standard_5_progress;
	uint8_t mark_sentry_progress;
} radar_mark_data_t;

typedef struct {
	uint32_t sentry_info;
} sentry_info_t;

typedef struct {
	uint8_t radar_info;
} radar_info_t;

typedef struct {
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[113];
} robot_interaction_data_t;

typedef struct {
	uint8_t delete_type;
	uint8_t layer;
} interaction_layer_delete_t;

typedef struct {
	uint8_t figure_name[3];
	uint32_t operate_tpye:3;
	uint32_t figure_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t details_a:9;
	uint32_t details_b:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t details_c:10;
	uint32_t details_d:11;
	uint32_t details_e:11;
} interaction_figure_t;

typedef struct {
	interaction_figure_t interaction_figure[3];
} interaction_figure_2_t;

typedef struct {
	interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

typedef struct {
	interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;

typedef struct {
	interaction_figure_t graphic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;

typedef struct {
	uint32_t sentry_cmd;
} sentry_cmd_t;

typedef struct {
	uint8_t radar_cmd;
} radar_cmd_t;

typedef struct {
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
} map_command_t;

typedef struct {
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
} map_robot_data_t;

typedef struct {
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
} map_data_t;

typedef struct {
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;

typedef struct {
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} remote_control_t;

typedef struct {
	uint16_t key_value;
	uint16_t x_position:12;
	uint16_t mouse_left:4;
	uint16_t y_position:12;
	uint16_t mouse_right:4;
	uint16_t reserved;
} custom_client_data_t;



void usart_Init(void);

void usart_printf(const char *fmt,...);

void usart_printRC(void);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* INC_UART_H_ */
