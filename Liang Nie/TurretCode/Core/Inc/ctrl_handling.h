/*
 * ctrl_handling.h
 *
 *  Created on: May 08, 2024
 *      Author: JAN
 */

#include "main.h"
#include "UART.h"

void getKeyboardVelocity(int targetVelocity[], double sens, pc_control_t pc_control);

int timer2value(int counter, double sens);
