/*
 * custom_math.c
 *
 *  Created on: Jun 14, 2025
 *      Author: moose
 */
#include "custom_math.h"
#include "main.h"
#include "math.h"


float getAngle(int16_t x, int16_t y){
	if(x==0&&y==0){
		return (float)0.0;
	}
	double angleBuff = atan((double)y/(double)x)/2.0/M_PI*360.0;
	float angle = (float)angleBuff;

	if(x<0){
		angle = angle+180.0;
	}
	if(x>=0&&y<0){
		angle = 360.0+angle;
	}
	return angle;
}

float getNorm(int16_t x, int16_t y){
	return (sqrtf(x*x+y*y));
}

float angSmallestDiff(float target, float current){
	if(target>current){
		if((target-current)>=180.0){
			return target-current-360.0;
		}
		if((target-current)<180.0){
			return target-current;
		}
	}
	else if(target<current){
		if((current-target)>=180.0){
			return target-current+360.0;
		}
		if((current-target)<180.0){
			return target-current;
		}
	}else{
		return 0.0;
	}
}

float getY(float norm,float angle){
	return (norm*sin(angle*2.0*M_PI/360.0));
}

float getX(float norm,float angle){
	return (norm*cosf(angle*2.0*M_PI/360.0));
}

float normalize3D(float vector[3]){
	float norm = sqrtf(vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2]);
	vector[0] = vector[0]/norm;
	vector[1] = vector[1]/norm;
	vector[2] = vector[2]/norm;
	return norm;
}
