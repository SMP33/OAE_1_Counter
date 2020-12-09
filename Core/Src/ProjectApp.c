/*
 * ProjectApp.c
 *
 *  Created on: Oct 5, 2020
 *      Author: mikha
 */

#include "ProjectApp.h"
#include "stm32f4xx.h"

ProjectApp app = { 0, 0, 0 };

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart2;

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);

void appTick() {
	if(app.state==START_MEASURMENTS)
	{
		changeChannel(1);

	} else if (app.timeToSend) {
		nextState();

	}
}

void startCounter() {
	__HAL_TIM_SET_COUNTER(&htim2, 0x0000);
	__HAL_TIM_SET_COUNTER(&htim12, 0x0000);

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim12);
}

void nextState() {

	stopCounter();
	HAL_TIM_Base_Stop_IT(&htim12);

	app.timeToSend=0;
	app.count= htim2.Instance->CNT;
	app.ticks[app.state-1]=app.count;

	sprintf(app.outStr, "Ch %u: %lu Hz\r\n", app.state, app.count);
	CDC_Transmit_FS(app.outStr, strlen(app.outStr));
	HAL_Delay(1e1);

	app.state++;
	if(app.state>CHANNELS_COUNT){
		app.state=0;
		sprintf(app.outStr,"Stop measurments\n");
		CDC_Transmit_FS(app.outStr, strlen(app.outStr));
		HAL_Delay(1e1);
	} else {
		changeChannel(app.state);
	}


}

void stopCounter() {
	HAL_TIM_Base_Stop(&htim2);
}

void changeChannel(uint8_t ch) {

	app.state = ch;

	uint8_t bit0 = getBit(app.state - 1, 0);
	uint8_t bit1 = getBit(app.state - 1, 1);
	uint8_t bit2 = getBit(app.state - 1, 2);

	toggleSelector(Selector_0, bit0, bit1, bit2);
	toggleSelector(Selector_1, bit0, bit1, bit2);

	//sprintf(app.outStr, "\nChannel %u was selected!\nBits: %u%u%u\n\n", app.state,bit2,bit1,bit0);
	if(ch==1){
		sprintf(app.outStr, "\nStart measurments:\n");
			CDC_Transmit_FS(app.outStr, strlen(app.outStr));
			HAL_Delay(1e1);
	}
	startCounter();
}

uint8_t getBit(uint16_t value, uint8_t bit) {
	return (value & (1 << bit)) >> bit;
}

int _write(int file, char *ptr, int len) {
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}
