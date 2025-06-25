#include "I2C.h"
#include "UART.h"
#include "main.h"

static uint8_t RxBuff_1[256];
extern I2C_HandleTypeDef hi2c2;
openMV_data_t openMV_data;

void I2C2_init(void) {
	HAL_I2C_Master_Receive_DMA(&hi2c2, OPENMV_ADDRESS, RxBuff_1, 256);
	usart_printf("I2C 2 initialized /r/n");
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c2)	{
	HAL_I2C_Master_Receive_DMA(hi2c2, OPENMV_ADDRESS, RxBuff_1, 256);
	usart_printf("fuck %d, %d, %d /r/n", RxBuff_1[0], RxBuff_1[1], RxBuff_1[2]);
	openMV_data.status = RxBuff_1[0];
	openMV_data.x_cord = RxBuff_1[1];
	openMV_data.y_cord = RxBuff_1[2];
}

uint8_t get_openMV_status(void) {
	return openMV_data.status;
}

uint8_t get_openMV_x(void) {
	return openMV_data.x_cord;
}

uint8_t get_openMV_y(void) {
	return openMV_data.y_cord;
}



