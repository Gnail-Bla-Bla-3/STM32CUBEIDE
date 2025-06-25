/*
 * custom_math.h
 *
 *  Created on: Jun 14, 2025
 *      Author: moose
 */

#ifndef INC_CUSTOM_MATH_H_
#define INC_CUSTOM_MATH_H_
#include "main.h"

float getAngle(int16_t x, int16_t y);
float getNorm(int16_t x, int16_t y);
float angSmallestDiff(float target, float current);
float getX(float norm,float angle);
float getY(float norm,float angle);
float normalize3D(float vector[3]);

#endif /* INC_CUSTOM_MATH_H_ */
