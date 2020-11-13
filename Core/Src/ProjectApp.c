/*
 * ProjectApp.c
 *
 *  Created on: Oct 5, 2020
 *      Author: mikha
 */

#include "ProjectApp.h"
#include "stm32f4xx.h"

ProjectApp app={0,0,0};

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim12;

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);

void appTick()
{
	if(app.timeToSend)
	{
		sendCounterData();
	}
}

void startCounter()
{
	__HAL_TIM_SET_COUNTER(&htim2, 0x0000);
	__HAL_TIM_SET_COUNTER(&htim12, 0x0000);

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim12);
}

void sendCounterData() {

	app.timeToSend = 0;
	app.count = htim2.Instance->CNT;

	sprintf(app.outStr, "Ch %u: %lu Hz\r\n",app.channel, app.count);
	CDC_Transmit_FS(app.outStr, strlen(app.outStr));
	HAL_Delay(1e1);

	 startCounter();
}

void stopCounter()
{
	HAL_TIM_Base_Stop(&htim2);
}

void changeChannel(uint8_t* data)
{
	stopCounter();
	HAL_TIM_Base_Stop_IT(&htim12);

	app.channel=(data[0]-48)%8;

	uint8_t bit0=getBit(app.channel, 1);
	uint8_t bit1=getBit(app.channel, 2);
	uint8_t bit2=getBit(app.channel, 3);

	toggleSelector(Selector_0, bit0, bit1, bit2);
	toggleSelector(Selector_1, bit0, bit1, bit2);

	sprintf(app.outStr, "\nChannel %u was selected!\n\n",app.channel);

	CDC_Transmit_FS(app.outStr, strlen(app.outStr));


	startCounter();
}

uint8_t getBit(uint16_t value, uint8_t bit)
{
	return (value & ( 1 << bit )) >> bit;
}

int _write(int file, char *ptr, int len) {
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}
