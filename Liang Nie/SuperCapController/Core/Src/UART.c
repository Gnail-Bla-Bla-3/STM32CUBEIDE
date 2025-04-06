/*
 * UART.C
 *
 *  Created on: Feb 20, 2025
 *      Author: swjxw
 */

#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "main.h"
#include "UART.h"

static uint8_t RxBuff_1[256];
static uint8_t RxBuff_2[256];
static uint8_t RxBuff_3[256];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void usart_Init(void) {
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuff_1, 256);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuff_2, 256);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RxBuff_3, 256);
}

void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

    HAL_UART_Transmit_DMA(&huart3, tx_buf, len);

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RxBuff_3, 256);
	//usart_printf("data=%d \r\n", RxBuff_3[0]);
}

