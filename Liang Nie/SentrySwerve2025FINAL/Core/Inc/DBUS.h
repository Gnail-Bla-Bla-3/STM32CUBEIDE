/*
 * DBUS.h
 *
 *  Created on: May 13, 2025
 *      Author: moose
 */

#ifndef INC_DBUS_H_
#define INC_DBUS_H_
#include "main.h"

typedef union {
    struct {
        uint16_t CH0 :11;
        uint16_t CH1 :11;
        uint16_t CH2 :11;
        uint16_t CH3 :11;
        uint8_t S1 :2;
        uint8_t S2 :2;
        int16_t MouseX :16;
        int16_t MouseY :16;
        int16_t MouseZ :16;
        uint8_t MouseL :8;
        uint8_t MouseR :8;
        uint8_t KeyData1 :8;
        uint8_t KeyData2 :8;
        uint16_t RESERVED :16;
    };
} DR16_DBUS_t;

void DBUS_Rx_Init(UART_HandleTypeDef* huart);
void DBUS_Update();

int16_t getDR16_CH0();

int16_t getDR16_CH1();

int16_t getDR16_CH2();

int16_t getDR16_CH3();

uint8_t getDR16_S1();

uint8_t getDR16_S2();

int16_t getDR16_CH4();

uint8_t getDR16_W();

uint8_t getDR16_S();

uint8_t getDR16_A();

uint8_t getDR16_D();

uint8_t getDR16_Q();

uint8_t getDR16_E();

uint8_t getDR16_Shift();

uint8_t getDR16_Ctrl();

uint8_t getDR16_R();

uint8_t getDR16_F();

uint8_t getDR16_G();

uint8_t getDR16_Z();

uint8_t getDR16_X();

uint8_t getDR16_C();

uint8_t getDR16_V();

uint8_t getDR16_B();

int16_t getDR16_MouseX();

int16_t getDR16_MouseY();

int16_t getDR16_MouseZ();

uint8_t getDR16_MouseL();

uint8_t getDR16_MouseR();



#endif /* INC_DBUS_H_ */
