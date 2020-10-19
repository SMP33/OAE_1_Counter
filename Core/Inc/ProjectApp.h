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
	uint8_t timeToSend;
	uint32_t channel[4];
	uint32_t higthBits[4];

}ProjectApp;

extern ProjectApp app;

#endif /* INC_PROJECTAPP_H_ */
