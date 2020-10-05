/*
 * ProjectApp.c
 *
 *  Created on: Oct 5, 2020
 *      Author: mikha
 */

#include "ProjectApp.h"
#include "stm32f4xx.h"

ProjectApp app={0,0};

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */

void EXTI9_5_IRQHandler(void)
{
	app.ticksCounter++;
	EXTI->PR|=EXTI_PR_PR5;
}

