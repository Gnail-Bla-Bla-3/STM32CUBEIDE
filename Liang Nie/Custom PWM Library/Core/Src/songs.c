/*
 * songs.c
 *
 *  Created on: May 4, 2024
 *      Author: Liang Nie
 */


#include "pwm.h"
#include "main.h"
#include "songs.h"
#include "UART.h"

//struct SongFormat NeverGoingToGiveYouUp = {0, 0,  61, 11,  {440, 494, 523, 587, 659, 698, 784, 880, 988, 1047, 10}, {2, 3, 5, 3, 7, 10, 7, 6, 2, 3, 5, 3, 6, 10, 6, 5, 4, 3, 2, 3, 5, 3, 5, 6, 4, 3, 2, 10, 2, 6, 5, 2, 3, 5, 3, 7, 10, 7, 6, 2, 3, 5, 3, 9, 4, 5, 4, 3, 2, 3, 5, 3, 5, 6, 4, 3, 2, 10, 2, 6, 5},   {10, 10, 10, 10, 29, 1, 30, 60, 10, 10, 10, 10, 29, 1, 30, 30, 10, 20, 10, 10, 10, 10, 40, 20, 30, 10, 39, 1, 20, 40, 80, 10, 10, 10, 10, 29, 1, 30, 60, 10, 10, 10, 10, 40, 20, 30, 10, 20, 10, 10, 10, 10, 40, 20, 30, 10, 39, 1, 20, 40, 80}};

uint8_t songProgress = 0;


uint16_t timeTracker = 0;
uint16_t noteTracker = 0;
uint32_t frequenciesAtA4[12] = {440, 494, 523, 587, 659, 698, 784, 880, 988, 1047,  1174,  20};
uint8_t NeverGoingToGiveYouUpNotes[61] = {2, 3, 5, 3, 7, 11, 7, 6, 2, 3, 5, 3, 6, 11, 6, 5, 4, 3, 2, 3, 5, 3, 5, 6, 4, 3, 2, 11, 2, 6, 5, 2, 3, 5, 3, 7, 11, 7, 6, 2, 3, 5, 3, 9, 4, 5, 4, 3, 2, 3, 5, 3, 5, 6, 4, 3, 2, 11, 2, 6, 5};
uint8_t NeverGoingToGiveYouUpRests[61] = {10, 10, 10, 10, 29, 1, 30, 60, 10, 10, 10, 10, 29, 1, 30, 30, 10, 20, 10, 10, 10, 10, 40, 20, 30, 10, 39, 1, 20, 40, 80, 10, 10, 10, 10, 29, 1, 30, 60, 10, 10, 10, 10, 40, 20, 30, 10, 20, 10, 10, 10, 10, 40, 20, 30, 10, 39, 1, 20, 40, 80};


uint8_t CG1Notes [8] = {5, 11, 5, 10, 11, 10, 9, 7};
uint8_t CG2Notes [58] = {7, 9, 7, 9, 7, 9, 7, 5, 11, 5, 11, 5, 11, 5, 11, 5, 3, 5, 6, 7, 11, 7, 5, 11, 5, 11, 5, 11, 5, 11, 5, 2, 11, 2, 6, 5, 11, 5, 11, 5, 11, 5, 11, 5, 11, 5, 3, 5, 6, 7, 11, 7, 5, 9, 7, 9, 7, 9} ;
uint8_t CG1Rests [8] = {19, 1, 20, 19,  1,  20, 40, 60} ;
uint8_t CG2Rests [58] = {40, 20, 20, 20, 20, 20, 40, 39, 1 , 39, 1 ,39, 1 , 19, 1 , 40, 20, 20, 20, 19, 1, 20, 39, 1 , 39, 1 ,39, 1 , 39, 1 , 20 , 39, 1, 20 , 40, 79, 1 , 39, 1 , 39, 1 , 39, 1 , 19, 1 , 40, 20, 20, 20, 19, 1 , 20 , 60, 20, 20, 20, 20, 40};

uint32_t frequencyEMajorAtA5[15] = {880, 988, 1109, 1244, 1319, 1480, 1661, 1760, 1976, 2217, 2489, 2637, 1568, 659, 20};
uint8_t HD1Notes[45] = {6, 12, 6, 4, 14, 4, 14, 4, 1, 14, 1, 6, 1, 4, 6, 8, 11, 9, 8, 9, 8, 6, 8, 4, 14, 4, 14, 4, 2, 4, 5, 4, 6, 8, 6, 8, 2, 1, 2, 4, 13, 14, 13, 14, 13};
uint8_t HD2Notes[39] = {4, 5, 6, 4, 1, 14, 1, 14, 1, 4, 1, 4, 6, 5, 14, 5, 4, 14, 4, 6, 4, 6, 8, 7, 14, 7, 6, 5, 6, 14, 6, 14, 6, 5, 14, 5, 14, 5, 4};
uint8_t HD3Notes[46] = {4, 5, 6, 14, 6, 4, 14, 4, 1, 14, 1, 14, 1, 14, 1, 4, 1, 4, 6, 5, 14, 5, 4, 14, 4, 6, 4, 6, 8, 7, 14, 7, 6, 5, 6, 14, 6, 14, 6, 5, 14, 5, 14, 5, 4};
uint8_t HD1Rests[45] = {10, 10, 20, 19, 1, 19, 1, 10, 29, 1, 40, 40, 20, 10, 20, 110, 10, 10, 10, 10, 10, 10, 10, 9, 1, 9, 1, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 20, 40, 39, 1, 39, 1, 20};
uint8_t HD2Rests[39] = {10, 10, 40, 40, 59, 1, 9, 1, 10, 20, 20, 20, 20, 29, 1, 30, 9, 1, 10, 20, 20, 20, 20, 29, 1, 30, 10, 10, 9, 1, 9, 1, 20, 9, 1, 9, 1, 20, 60};
uint8_t HD3Rests[46] = {10, 10, 19, 1, 20, 19, 1, 20, 29, 1, 29, 1, 9, 1, 10, 20, 20, 20, 20, 29, 1, 30, 9, 1, 10, 20, 20, 20, 20, 29, 1, 30, 10, 10, 9, 1, 9, 1, 20, 9, 1, 9, 1, 20, 60};

void initializeCaliforniaGirls() {
	songProgress = 0;
	timeTracker = 0;
	noteTracker = 0;
}

uint32_t HampsterNotes() {
	uint32_t actualFrequency = 0;
	if (songProgress == 0 || songProgress == 1) {
		if (timeTracker > HD1Rests[noteTracker]) {
			noteTracker++;
			timeTracker = 0;
		}

		if (noteTracker > 44) {
			noteTracker = 0;
			if (songProgress == 0) {
				songProgress = 1;
			} else {
				songProgress = 2;
			}
		}
		actualFrequency =  frequencyEMajorAtA5[HD1Notes[noteTracker]];
	} else if (songProgress == 2){
		if (timeTracker > HD2Rests[noteTracker]) {
			noteTracker++;
			timeTracker = 0;
		}

		if (noteTracker > 38) {
			songProgress = 3;
			noteTracker = 0;
		}
		actualFrequency = frequencyEMajorAtA5[HD2Notes[noteTracker]];
	} else {
		if (timeTracker > HD3Rests[noteTracker]) {
			noteTracker++;
			timeTracker = 0;
		}

		if (noteTracker > 45) {
			songProgress = 0;
			noteTracker = 0;
		}
		actualFrequency = frequencyEMajorAtA5[HD3Notes[noteTracker]];
	}
	timeTracker++;
	return actualFrequency;

}

uint32_t CaliforniaGirlsNotes() {
	uint32_t actualFrequency = 0;
	if (songProgress == 0) {
		if (timeTracker > CG1Rests[noteTracker]) {
			noteTracker++;
			timeTracker = 0;
		}

		if (noteTracker > 7) {
			songProgress = 1;
			noteTracker = 0;;
		}
		actualFrequency =  frequenciesAtA4[CG1Notes[noteTracker]];
	} else {

		if (timeTracker > CG2Rests[noteTracker]) {
			noteTracker++;
			timeTracker = 0;
		}

		if (noteTracker > 57) {
			noteTracker = 0;
		}
		actualFrequency = frequenciesAtA4[CG2Notes[noteTracker]];
	}
	// usart_printf();
	// PWMOutput(Buzzer, 1, frequenciesAtA4[NeverGoingToGiveYouUpNotes[noteTracker]]);

	timeTracker++;

	return  actualFrequency;
}


uint32_t NeverGonnaGiveYouUpNotes() {
	if (timeTracker > NeverGoingToGiveYouUpRests[noteTracker]) {
		noteTracker++;
		timeTracker = 0;
	}

	if (noteTracker > 60) {
		noteTracker = 0;
	}
	// usart_printf();
	// PWMOutput(Buzzer, 1, frequenciesAtA4[NeverGoingToGiveYouUpNotes[noteTracker]]);

	timeTracker++;

	return  frequenciesAtA4[NeverGoingToGiveYouUpNotes[noteTracker]];
}
