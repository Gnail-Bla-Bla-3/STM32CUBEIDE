/*
 * IST8310.h
 *
 *  Created on: May 17, 2025
 *      Author: liangnie
 */

#ifndef INC_IST8310_H_
#define INC_IST8310_H_
#include "main.h"

typedef enum {
	IST8310_SLAVE_ADDRESS_8BIT = 0x1C,
	IST8310_WHO_AM_I_REGISTER = 0x00,
	IST8310_PULSE_DURATION_CONTROL_REGISTER = 0x42,
	IST8310_CONTROL_REGISTER_1 = 0x0A,

	/*
	Technically this 0x03 is only for X_LOW, but all the other Directional Registers are beside it, so all i really need is the 0x03 but for later use:
	X_LOW = 0x03  | Y_LOW = 0x05  | Z_LOW = 0x07
	X_HIGH = 0x04 | Y_HIGH = 0x06 | Z_HIGH = 0x08
	*/
	IST8310_OUTPUT_VALUE_X_TO_Z = 0x03,

} IST8310_headers;


void IST8310_Init(I2C_HandleTypeDef *i2c);
void IST8310_Update();
int16_t getIST8310_X();
int16_t getIST8310_Y();
int16_t getIST8310_Z();


#endif /* INC_IST8310_H_ */
