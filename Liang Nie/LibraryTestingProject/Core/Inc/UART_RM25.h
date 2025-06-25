/*
 * UART.h
 *
 *  Created on: May 13, 2025
 *      Author: moose
 */

#ifndef INC_UART_RM25_H_
#define INC_UART_RM25_H_
#include "main.h"

typedef enum
{
	//frame header definitions for the referee system's UART outputs
    GAME_STATUS_HEADER = 0x0001,
	GAME_RESULT_HEADER = 0x0002,
	ROBOT_HP_HEADER = 0x0003,
	EVENT_DATA_HEADER = 0x0101,
	REFEREE_WARNING_HEADER = 0x0104,
	DART_INFO_HEADER = 0x0105,
	ROBOT_STATUS_HEADER = 0x0201,
	POWER_HEAT_DATA_HEADER = 0x0202,
	ROBOT_POSITION_HEADER = 0x0203,
	BUFF_HEADER = 0x0204,
	DAMAGE_DATA_HEADER = 0x0206,
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
	PC_CONTROL_HEADER = 0x0304,
	MINIMAP_TARGET_HEADER = 0x0305,
	CUSTOM_CONTROLLER_CLIENT_HEADER = 0x0306,
	MINIMAP_DATA_HEADER = 0x0307,
	CUSTOM_INFO_HEADER = 0x0308,
	CUSTOM_MESSAGE_HEADER = 0x0401,

	// sub-content IDs for 0x0301
	UI_DELETE_HEADER = 0x0100,
	UI_EDIT_1_HEADER = 0x0101,
	UI_EDIT_3_HEADER = 0x0102,
	UI_EDIT_5_HEADER = 0x0103,
	UI_EDIT_7_HEADER = 0x0104,
	UI_PRINT_CHAR_3_HEADER = 0x0110,
	SENTRY_CMD_HEADER = 0x0120,
	RADAR_CMD_HEADER = 0x0121,


} uart_headers_t;

typedef enum {
	frame_header_offset = 7,

	game_status_len = 11,
	game_result_len = 1,
	game_robot_HP_len = 32,
	event_data_len = 4,
	referee_warning_len = 3,
	dart_info_len = 3,
	robot_status_len = 13,
	power_heat_data_len = 16,
	robot_pos_len = 16,
	buff_len = 7,


	damage_data_len = 1,
	shoot_data_len = 7,
	projectile_allowance_len = 6,
	rfid_status_len = 4,
	dart_client_cmd_len = 6,
	ground_robot_position_len = 40,
	radar_mark_data_len = 1,
	sentry_info_len = 6,
	radar_info_len = 1,
	robot_interaction_data_len = 127,
	interaction_layer_delete_len = 1,
	interaction_figure_len = 1,
	interaction_figure_2_len = 1,
	interaction_figure_3_len = 1,
	interaction_figure_4_len = 1,
	ext_client_custom_character_len = 1,

	sentry_cmd_len = 14,
	radar_cmd_len = 1,

	map_command_len = 112, //0303
	map_robot_data_len = 24, //0305
	map_data_len = 105,  //0307
	custom_info_len = 34,

	pc_control_len = 12, //0304
	custom_robot_data_len = 30, //0302

	custom_client_data_len = 8,
	custom_message_len = 128 //0401

} lengths_and_offsets_t;

typedef enum{
	noop = 0,
	add = 1,
	modify = 2,
	delete = 3,
}graphic_operation_t;

typedef enum{
	line = 0,
	rect = 1,
	circ = 2,
	ellipse = 3,
	arc = 4,
	float_type = 5,
	int_type = 6,
	char_type = 7,
}graphic_type_t;

typedef enum{
	team_color = 0,
	yellow = 1,
	lime = 2,
	orange = 3,
	purple = 4,
	pink = 5,
	cyan = 6,
	black = 7,
	white = 8,
}colour_t;




typedef struct
{
uint8_t figure_name[3];
uint32_t operate_type:3;
uint32_t figure_type:3;
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
uint32_t graphic_config_1;
uint32_t graphic_config_2;
uint32_t graphic_config_3;
uint8_t isUsed0;
}interaction_figure_t;


typedef struct {
	uint8_t game_type;
	uint8_t current_stage;
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
	//uint16_t red_5_robot_HP; reserved as of 2025
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	//uint16_t blue_5_robot_HP; reserved as of 2025
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} robot_HP_t;

typedef struct {
	uint8_t resupply_zone_1;
	uint8_t resupply_zone_2;
	uint8_t resupply_zone_3;
	uint8_t small_power_rune;
	uint8_t large_power_rune;
	uint8_t central_elevated_ground;
	uint8_t trapezoid_elevated_ground;
	uint16_t last_dart_hit;
	uint8_t last_dart_hit_target;
	uint8_t central_buff;
} event_data_t;

typedef struct {
	uint8_t penalty;
	uint8_t offending_robot_id;
	uint8_t count;
} referee_warning_t;

typedef struct {
	uint8_t dart_remaining_time;
	uint8_t last_dart_hit_target;
	uint8_t last_target_hit_count;
	uint8_t current_dart_target;
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
	//uint16_t chassis_voltage; reserved as of 2025
	//uint16_t chassis_current; reserved as of 2025
	//float chassis_power; reserved as of 2025
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

typedef struct {
	float x;
	float y;
	float angle;
} robot_position_t;

typedef struct {
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy;
} buffs_t;

typedef struct {
	uint8_t armor_id;
	uint8_t HP_deduction_reason;
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
	uint32_t rfid_status;//not done cuz frick dat
} rfid_status_t;

typedef struct {
	uint8_t dart_launch_opening_status;
	uint16_t target_change_remaining_time;
	uint16_t latest_launch_cmd_remaining_time;
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
	uint8_t mark_sentry_progress;
} radar_mark_data_t;

typedef struct {
	uint16_t exchanged_projectiles;
	uint8_t projectile_exchange_count;
	uint8_t HP_exchange_count;
	uint8_t confirm_free_respawn;
	uint8_t instant_respawn_available;
	uint16_t instant_respawn_cost;
	uint8_t in_combat;
	uint16_t remaining_allowance_to_exchange;
} sentry_info_t;

typedef struct {
	uint8_t double_vulnerability_chances;
	uint8_t double_vulnerability_triggered;
} radar_info_t;

//interaction communications are non-functional
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

//end of unfinished section


typedef struct {
	uint32_t sentry_cmd;
} sentry_cmd_t;

typedef struct {
	uint8_t radar_cmd;
} radar_cmd_t;


typedef struct {
	uint8_t custom_data[30];
}custom_controller_robot_t;

typedef struct {
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint16_t cmd_source;
} map_command_t;

typedef struct {
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t left_button_down;
	uint8_t right_button_down;
	uint8_t keyboard_values_1;
	uint8_t keyboard_values_2;
	uint16_t reserved;

	uint8_t w;
	uint8_t s;
	uint8_t a;
	uint8_t d;
	uint8_t shift;
	uint8_t ctrl;
	uint8_t q;
	uint8_t e;

	uint8_t r;
	uint8_t f;
	uint8_t g;
	uint8_t z;
	uint8_t x;
	uint8_t c;
	uint8_t v;
	uint8_t b;
} pc_control_t;

typedef struct {
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
} map_data_t;

typedef struct {
	uint16_t key_value;
	uint16_t x_position:12;
	uint16_t mouse_left:4;
	uint16_t y_position:12;
	uint16_t mouse_right:4;
	uint16_t reserved;
} custom_client_data_t;

typedef struct {
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
} map_robot_data_t;

typedef struct {
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;

typedef struct {
	uint8_t custom_message[128];
}custom_message_t;

uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);


uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength);

void UART_Printf(UART_HandleTypeDef *huart,const char *fmt,...);

void UART_Ex_Init(UART_HandleTypeDef *huart);

void UART_Transmit_RM(UART_HandleTypeDef *huart,uint16_t cmd_id, uint8_t data[]);

uint8_t getBits(uint8_t data, uint8_t startBit, uint8_t len);

void GAME_STATUS_RxEventCallback(uint8_t game_type,uint8_t current_stage,uint8_t game_progress, uint16_t stage_remain_time,uint64_t SyncTimeStamp);
void GAME_RESULT_RxEventCallback(uint8_t winner);
void ROBOT_HP_HEADER_RxEventCallback(uint16_t red_1_robot_HP, uint16_t red_2_robot_HP, uint16_t red_3_robot_HP, uint16_t red_4_robot_HP, uint16_t red_7_robot_HP, uint16_t red_outpost_HP, uint16_t red_base_HP, uint16_t blue_1_robot_HP, uint16_t blue_2_robot_HP, uint16_t blue_3_robot_HP, uint16_t blue_4_robot_HP, uint16_t blue_7_robot_HP, uint16_t blue_outpost_HP, uint16_t blue_base_HP);
void EVENT_DATA_RxEventCallback(uint8_t resupply_zone_1, uint8_t resupply_zone_2, uint8_t resupply_zone_3, uint8_t small_power_rune, uint8_t large_power_rune, uint8_t central_elevated_ground, uint8_t trapezoid_elevated_ground, uint16_t last_dart_hit, uint8_t last_dart_hit_target, uint8_t central_buff);
void REFEREE_WARNING_RxEventCallback(uint8_t penalty,uint8_t offending_robot_id,uint8_t count);
void DART_INFO_RxEventCallback(uint8_t dart_remaining_time,uint8_t last_dart_hit_target,uint8_t last_target_hit_count, uint8_t current_dart_target);
void ROBOT_STATUS_RxEventCallback(uint8_t robot_id, uint8_t robot_level, uint16_t current_HP, uint16_t maximum_HP, uint16_t shooter_barrel_cooling_value, uint16_t shooter_barrel_heat_limit, uint16_t chassis_power_limit, uint8_t power_management_gimbal_output, uint8_t power_management_chassis_output, uint8_t power_management_shooter_output);
void POWER_HEAT_DATA_RxEventCallback(uint16_t buffer_energy, uint16_t shooter_17mm_1_barrel_heat, uint16_t shooter_17mm_2_barrel_heat, uint16_t shooter_42mm_barrel_heat);
void ROBOT_POSITION_RxEventCallback(float x, float y, float angle);
void BUFF_RxEventCallback(uint8_t recovery_buff, uint8_t cooling_buff, uint8_t defence_buff, uint8_t vulnerability_buff, uint16_t attack_buff, uint8_t remaining_energy);
void DAMAGE_SOURCE_RxEventCallback(uint8_t armor_id, uint8_t HP_deduction_reason);
void PROJECTILE_INFO_RxEventCallback(uint8_t bullet_type, uint8_t shooter_number, uint8_t launching_frequency,float initial_speed);
void PROJECTILE_ALLOWANCE_RxEventCallback(uint16_t projectile_allowance_17mm, uint16_t projectile_allowance_42mm, uint16_t remaining_gold_coin);
void RFID_RxEventCallback(uint32_t rfid_status);
void DART_COMMAND_RxEventCallback(uint8_t dart_launch_opening_status, uint16_t target_change_remaining_time, uint16_t latest_launch_cmd_remaining_time);
void TEAM_POSITION_RxEventCallback(float hero_x, float hero_y, float engineer_x, float engineer_y, float standard_3_x, float standard_3_y, float standard_4_x, float standard_4_y, float standard_5_x, float standard_5_y);
void RADAR_MARKING_RxEventCallback(uint8_t mark_hero_progress, uint8_t mark_engineer_progress, uint8_t mark_standard_3_progress, uint8_t mark_standard_4_progress, uint8_t mark_sentry_progress);
void SENTRY_RxEventCallback(uint16_t exchanged_projectiles, uint8_t projectile_exchange_count, uint8_t HP_exchange_count, uint8_t confirm_free_respawn, uint8_t instant_respawn_available, uint16_t instant_respawn_cost, uint8_t in_combat, uint16_t remaining_allowance_to_exchange);
void RADAR_BUFF_RxEventCallback(uint8_t double_vulnerability_chances, uint8_t double_vulnerability_triggered);


void CUSTOM_CONTROLLER_ROBOT_RxEventCallback(uint8_t customData[30]);
void MINIMAP_COMMAND_RxEventCallback(float target_position_x, float target_position_y, uint8_t cmd_keyboard, uint8_t target_robot_id, uint16_t cmd_source);
void PC_CONTROL_RxEventCallback(int16_t mouse_x, int16_t mouse_y, int16_t mouse_z, uint8_t left_button_down, uint8_t right_button_down, uint8_t w, uint8_t s, uint8_t a, uint8_t d, uint8_t shift,uint8_t ctrl,uint8_t q, uint8_t e, uint8_t r, uint8_t f, uint8_t g, uint8_t z, uint8_t x, uint8_t c, uint8_t v, uint8_t b);
void MINIMAP_TARGET_RxEventCallback(uint16_t target_robot_id, float target_position_x, float target_position_y);
void CUSTOM_CONTROLLER_CLIENT_RxEventCallback(uint16_t key_value, uint16_t x_position, uint16_t mouse_left, uint16_t y_position, uint16_t mouse_right);
void MINIMAP_DATA_RxEventCallback(uint8_t intention, uint16_t start_position_x, uint16_t start_position_y, int8_t delta_x[49], int8_t delta_y[49], uint16_t sender_id);
void CUSTOM_MESSAGE_RxEventCallback(uint8_t customMessage[128]);

#endif /* INC_UART_RM25_H_ */
