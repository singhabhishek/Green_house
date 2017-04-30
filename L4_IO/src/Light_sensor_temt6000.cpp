/*
 * Light_sensor_temt6000.cpp
 *
 *  Created on: Apr 30, 2017
 *      Author: Abhishek
 */

#include <stdlib.h>
#include <stdint.h>
#include "adc0.h"
#include "Light_sensor_temt6000.hpp"
#include "LPC17xx.h"


#define ADC_RESOLUTION (4095)
static uint16_t reading;

uint16_t getRawLightValue()
{
	LPC_PINCON->PINSEL3 |=  (3 << 28);
	reading = adc0_get_reading(4);
	return reading;
}

uint8_t  getPercentLightValue()
{
	return (uint8_t)(((reading*100)/ADC_RESOLUTION));
}

