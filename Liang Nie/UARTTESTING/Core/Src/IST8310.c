/*
 * IST8310.c
 *
 *  Created on: May 17, 2025
 *      Author: liangnie
 */

#include "IST8310.h"
#include "main.h"


I2C_HandleTypeDef *i2c3 = NULL;
uint8_t IST8310_WriteSent = 0;
uint8_t XtoZOutputValues[6]={0, 0, 0, 0, 0, 0};
int16_t processedValues[3] = {0, 0, 0};

/*
 * Takes the 6 unsigned 8 Bit ints received from IST8310 and combines them into 3 SIGNED 16 bit ints
 * No need to do the matrix thing as the chip does it for us thank god
 */
void processData(int16_t processedData[3], uint8_t unprocessedData[6]) {
	for (uint8_t i = 0; i < 3; i++) {
		// memcpy(&processedData[i], &unprocessedData[i*2], 2);
		processedData[i] = 0;
		processedData[i] += unprocessedData[i*2];
		processedData[i] += unprocessedData[(i*2)+1] << 8;
	}
	// usart_printf("%d %d %d\r\n", processedData[0], processedData[1], processedData[2]);
}

/*
 * Starts by resetting the pin PG6. Note that in this case, 0 is on and 1 is off, I found out about
 *
 * Next this function Reads the "Who Am I" byte to the "Who Am I" Register
 * If IST8310 is receiving properly, it'll return with 0x10, else it will return 0x00, whuch means it isn't receiving
 *
 * Funally, it send the Optimization command to Optimize the chip... the documentation told us to do it so we did :)
 */
void IST8310_Init(I2C_HandleTypeDef *i2c) {

	// Sets Pointer for later access to this library
	i2c3 = i2c;

	uint8_t whoAmI = 0;
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, 0);
	HAL_Delay(6);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, 1);
	HAL_Delay(6);

	// Checks if the Magnometer is Reponding
	HAL_I2C_Mem_Read(&i2c3, IST8310_SLAVE_ADDRESS_8BIT, IST8310_WHO_AM_I_REGISTER, 1, &whoAmI, 1, 50);
	if (whoAmI != 0x10) {
		usart_printf("Magnometer Not Responding!! \r\n");
		return 0;
	}
	HAL_Delay(6);

	// Performance Optimization
	uint8_t OptimizationDataToSend = 0xC0;
	HAL_I2C_Mem_Write(&i2c3, IST8310_SLAVE_ADDRESS_8BIT, IST8310_PULSE_DURATION_CONTROL_REGISTER, 1, &OptimizationDataToSend, 1, 50);
	HAL_Delay(6);


}

/*
 * Start by writing 0x01 to the CR1
 * This tells the chip to read the magnometer readings
 *
 * Next the board checks if the ReadyToRead GPIO pin is 1. If not, it extends the delay by 5ms
 *
 * Next it reads all 6 registers on the chip such that it used by the user.
 * Do note the "IST8310_OUTPUT_VALUE_X_TO_Z" is the register address for X_LOW but because all the register addresses are beside each other, we can read them all with a 6 byte array
 *
 * Finally, the function proccesses the data such that it is usable for the general user
 */
void IST8310_Update() {

	if (IST8310_WriteSent == 0) {
		uint8_t SMMDataToSend = 0x01;
		HAL_I2C_Mem_Write(&i2c3, IST8310_SLAVE_ADDRESS_8BIT, IST8310_CONTROL_REGISTER_1, 1, &SMMDataToSend, 1, 50);
		IST8310_WriteSent = 1;
	}

	if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3) == 1 && IST8310_WriteSent == 1) {
		HAL_I2C_Mem_Read(&i2c3, IST8310_SLAVE_ADDRESS_8BIT, IST8310_OUTPUT_VALUE_X_TO_Z, 1, XtoZOutputValues, 6, 50);
		processData(processedValues, XtoZOutputValues);
		IST8310_WriteSent = 0;
	}

	// HAL_I2C_Mem_Read(&i2c3, IST8310_SLAVE_ADDRESS_8BIT, IST8310_OUTPUT_VALUE_X_TO_Z, 1, XtoZOutputValues, 6, 50);

	// osDelay(6);

	// processData(processedValues, XtoZOutputValues);

}

/*
 * Kind of obvioous, if you don't know what's happening with these 3 functions, ask Liang
 */
int16_t getIST8310_X() {
	return processedValues[0];
}

int16_t getIST8310_Y() {
	return processedValues[1];
}
int16_t getIST8310_Z() {
	return processedValues[2];
}
