/*
 * ProjectApp.h
 *
 *  Created on: Oct 5, 2020
 *      Author: mikha
 */

#ifndef INC_PROJECTAPP_H_
#define INC_PROJECTAPP_H_

#define CHANNELS_COUNT 8
#define START_MEASURMENTS (CHANNELS_COUNT+1)

#include "stdint.h"

typedef enum
{
	Selector_0=0,
	Selector_1=1
}Selector;

typedef struct
{
	uint8_t timeToSend;
	uint32_t count;
	uint32_t ticks[CHANNELS_COUNT];
	uint8_t state;
	uint8_t outStr[128];

}ProjectApp;

extern ProjectApp app;

void appTick();
void nextState();
void startCounter();
void stopCounter();

void changeChannel(uint8_t ch);
void toggleSelector(Selector selector,uint8_t bit0, uint8_t bit1, uint8_t bit2);

uint8_t getBit(uint16_t value, uint8_t bit);

#endif /* INC_PROJECTAPP_H_ */
