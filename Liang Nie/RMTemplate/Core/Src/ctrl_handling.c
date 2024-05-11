/*
 * ctrl_handling.c
 *
 *  Created on: May 08, 2024
 *      Author: JAN
 */
#include "UART.h"
#include "main.h"
#include "ctrl_handling.h"
#include "math.h"


float maxSpeed = 9006.3;
int wCounter = 0;
int aCounter = 0;
int sCounter = 0;
int dCounter = 0;

//targetVelocity must be initialized to 0,0
void getKeyboardVelocity(int targetVelocity[], double sens, pc_control_t pc_control){

	double counterLimit = 3/sens;


	if((pc_control.w==1)){
		wCounter++;
	}else if(wCounter>1){
		wCounter=0;
	}else{
		wCounter=0;
	}
	targetVelocity[1] += timer2value(wCounter, sens);

	if((pc_control.s==1)){
		sCounter++;
	}else if(sCounter>1){
		sCounter=0;
	}else{
		sCounter=0;
	}
	targetVelocity[1] -= timer2value(sCounter, sens);

	if((pc_control.d==1)){
		dCounter++;
	}else if(dCounter>1){
		dCounter=0;
	}else{
		dCounter=0;
	}
	targetVelocity[0] += timer2value(dCounter, sens);


	if((pc_control.a==1)){
		aCounter++;
	}else if(aCounter>1){
		aCounter=0;
	}else{
		aCounter=0;
	}
	targetVelocity[0] -= timer2value(aCounter, sens);
}


int timer2value(int counter, double sens){
	return (int)(maxSpeed/(1.0+exp(-sens*((double)counter))));
}
