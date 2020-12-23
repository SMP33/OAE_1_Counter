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
	int flag = 0;
	if (app.state == START_MEASUREMENTS) {
		changeChannel(1);
	} else if (app.timeToSend) {
		nextMeasurement();
	} else if (app.state == 0) {

		//Детектим запрос
		if (HAL_GPIO_ReadPin(LTR_TRG_IN_GPIO_Port, LTR_TRG_IN_Pin)) {
			flag = 1;
			for (int i = 0; i < 5; i++) {
				HAL_Delay(1);
				if (!HAL_GPIO_ReadPin(LTR_TRG_IN_GPIO_Port, LTR_TRG_IN_Pin)) {
					break;
				}
			}

			if (flag) {
				startMeasurement();
			}

		}

	}
}

void startMeasurement() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	app.state = START_MEASUREMENTS;
}

void startCounter() {
	__HAL_TIM_SET_COUNTER(&htim2, 0x0000);
	__HAL_TIM_SET_COUNTER(&htim12, 0x0000);

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim12);
}

void transmitAppData() {
	sprintf(app.outStr,
			"\n1. %d\n2. %d\n3. %d\n4. %d\n5. %d\n6. %d\n7. %d\n8. %d\n Stop measurements\n",
			app.ticks[0], app.ticks[1], app.ticks[2], app.ticks[3],
			app.ticks[4], app.ticks[4], app.ticks[6], app.ticks[7]);
	CDC_Transmit_FS(app.outStr, strlen(app.outStr));
	HAL_Delay(1e2);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

	HAL_GPIO_WritePin(LTR_TRG_OUT_GPIO_Port, LTR_TRG_OUT_Pin, GPIO_PIN_SET);
	HAL_Delay(1e1);
	HAL_GPIO_WritePin(LTR_TRG_OUT_GPIO_Port, LTR_TRG_OUT_Pin, GPIO_PIN_RESET);

	for (int i = 0; i < 8; i++) {
		uint32_t c = (10 * (i));
		app.ticks[i] = (c + 1) << 24 | (c + 2) << 16 | (c + 3) << 8 | (c + 4);
	}

	uint32_t transDelay=3e2;

	HAL_Delay(transDelay);
	HAL_UART_Transmit(&huart2, app.ticks, sizeof(uint32_t) * 2, 1e3);
	HAL_Delay(transDelay);
	HAL_UART_Transmit(&huart2, app.ticks + 2, sizeof(uint32_t) * 2, 1e3);
	HAL_Delay(transDelay);
	HAL_UART_Transmit(&huart2, app.ticks + 4, sizeof(uint32_t) * 2, 1e3);
	HAL_Delay(transDelay);
	HAL_UART_Transmit(&huart2, app.ticks + 6, sizeof(uint32_t) * 2, 1e3);
}

void nextMeasurement() {

	stopCounter();
	HAL_TIM_Base_Stop_IT(&htim12);

	app.timeToSend = 0;
	app.count = htim2.Instance->CNT;
	app.ticks[app.state - 1] = app.count;

	//sprintf(app.outStr, "Ch %u: %lu Hz\r\n", app.state, app.count);
	sprintf(app.outStr, "\b\b\b%d/8", app.state);
	CDC_Transmit_FS(app.outStr, strlen(app.outStr));
	HAL_Delay(1e1);

	app.state++;
	if (app.state > CHANNELS_COUNT) {
		app.state = 0;
		transmitAppData();
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
	if (ch == 1) {
		sprintf(app.outStr, "\nStart measurements:    ");
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
