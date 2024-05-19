/*
 * songs.h
 *
 *  Created on: May 4, 2024
 *      Author: Liang Nie
 */

#ifndef INC_SONGS_H_
#define INC_SONGS_H_
#include "main.h"

struct SongFormat {
	uint16_t currentLenghtTrack;
	uint16_t currentTimeLoop;
	uint16_t length;
	uint8_t frequencyLength;
	uint32_t frequencys[16];
	uint8_t notes[128];
	uint8_t lengths[128];
};

uint32_t CountryRoadsNotes();
uint32_t MarryHadALittleLambNotes();
uint32_t NeverGonnaGiveYouUpNotes();
uint32_t CaliforniaGirlsNotes();
uint32_t HampsterNotes();
void initializeCaliforniaGirls();

#endif /* INC_SONGS_H_ */
