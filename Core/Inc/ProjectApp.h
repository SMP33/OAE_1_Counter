/*
 * ProjectApp.h
 *
 *  Created on: Oct 5, 2020
 *      Author: mikha
 */

#ifndef INC_PROJECTAPP_H_
#define INC_PROJECTAPP_H_

#define CHANNELS_COUNT 8
#define START_MEASUREMENTS (CHANNELS_COUNT+1)

#define LTR_TRG_IN_Pin GPIO_PIN_1
#define LTR_TRG_IN_GPIO_Port GPIOC
#define LTR_TRG_IN_EXTI_IRQn EXTI1_IRQn
#define LTR_TRG_OUT_Pin GPIO_PIN_3
#define LTR_TRG_OUT_GPIO_Port GPIOC

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
	uint8_t outStr[256];

}ProjectApp;

extern ProjectApp app;

void appTick();
void startMeasurement();
void nextMeasurement();
void startCounter();
void stopCounter();
void transmitAppData();

void changeChannel(uint8_t ch);
void toggleSelector(Selector selector,uint8_t bit0, uint8_t bit1, uint8_t bit2);

uint8_t getBit(uint16_t value, uint8_t bit);

#endif /* INC_PROJECTAPP_H_ */
