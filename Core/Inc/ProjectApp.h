/*
 * ProjectApp.h
 *
 *  Created on: Oct 5, 2020
 *      Author: mikha
 */

#ifndef INC_PROJECTAPP_H_
#define INC_PROJECTAPP_H_

#include "stdint.h"

typedef struct
{
	uint32_t ticks;
	uint32_t ticksCounter;
}ProjectApp;

extern ProjectApp app;
extern void EXTI9_5_IRQHandler(void);

#endif /* INC_PROJECTAPP_H_ */
