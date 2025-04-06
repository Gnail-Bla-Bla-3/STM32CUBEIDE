#ifndef I2C_H
#define I2C_H

#include "main.h"

#define OPENMV_ADDRESS 0x0E << 1  //the I2C address of the openMV board we set

typedef struct
{
  uint8_t status;
  uint8_t x_cord;
  uint8_t y_cord;
} openMV_data_t;

void I2C2_init(void);

#endif
