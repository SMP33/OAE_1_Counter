/*
 * ProjectApp.c
 *
 *  Created on: Oct 5, 2020
 *      Author: mikha
 */

#include "ProjectApp.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"

ProjectApp app={0,0};
extern UART_HandleTypeDef huart2;

void EXTI9_5_IRQHandler(void)
{
	app.ticksCounter++;
	EXTI->PR|=EXTI_PR_PR5;
}

void loadAndWrite(void)
{
	app.ticks=app.ticksCounter;
	app.ticksCounter=0;

	uint8_t bufSize=64;
	char buf[bufSize];
	sprintf(buf,"-> %lu Hz\r\n",app.ticks);
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf, strlen(buf));
}
