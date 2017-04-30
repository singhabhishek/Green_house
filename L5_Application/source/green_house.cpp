/*
 * green_house.cpp
 *
 *  Created on: Apr 30, 2017
 *      Author: Abhishek
 */

#include <stdio.h>
#include "green_house.hpp"
#include "Light_sensor_temt6000.hpp"
#include "i2c2.hpp"
#include "utilities.h"
#include "io.hpp"
#include "gpio.hpp"
#include "Adafruit_RA8875-master/Adafruit_RA8875.h"

struct sensor_readings
{
	uint16_t light_sensor_raw;
	char light_sensor_percentage;
	uint8_t temperature_sensor;
	uint8_t humidity_sensor;
};

static struct sensor_readings sReading;
Adafruit_RA8875 tft(3,4);
char lsReading[100];
char tempReading[100];
char humidityReading[100];


#define ADDRESS      			0x40
#define TEMP_MEASURE_HOLD 		0xE3
#define HUMD_MEASURE_HOLD  		0xE5
#define TEMP_MEASURE_NOHOLD  	0xF3
#define HUMD_MEASURE_NOHOLD  	0xF5
#define TEMP_PREV   			0xE0
#define WRITE_USER_REG  		0xE6
#define READ_USER_REG  			0xE7
#define SOFT_RESET 				0xFE


void i2c_demo(void * p)
{
	I2C2& i2c = I2C2::getInstance();
	const uint8_t dev_address = ADDRESS;
	uint8_t reg_address = TEMP_MEASURE_NOHOLD;
	uint8_t data = {0};
	while(1)
	{
		printf("Response = %d\n", i2c.checkDeviceResponse(dev_address));
		reg_address = TEMP_MEASURE_NOHOLD;
		data = i2c.readReg(dev_address, reg_address);
		//i2c.readRegisters(dev_address, reg_address, data, 16);
		//printf("%d-%d\n", data[0], data[1]);
		//delay_ms(100);
		//reg_address = 0xC9;
		//data = i2c.readReg(dev_address, reg_address);
		printf("%d\n", data);
		delay_ms(2000);
	}
}

void sensor_readings(void * p)
{
	while(1)
	{
		sReading.light_sensor_raw = getRawLightValue();
		sReading.light_sensor_percentage = getPercentLightValue();
		//printf("%d - %d\n", sReading.light_sensor_raw, light_sensor_percentage);
		LD.setNumber(sReading.light_sensor_percentage);
		delay_ms(100);
	}
}

void display_image()
{
	if(!tft.init_display(RA8875_800x480, RA8875_PWM_CLK_DIV1024))
	{
		printf("RA8875 Not Found!\n");
		while(1);
	}
	tft.graphicsMode();
	tft.setLayer(L1);
	tft.fillScreen(RA8875_RED);
	tft.setLayer(L2);
	tft.textMode();
    tft.textColor(RA8875_BLUE,RA8875_WHITE);
	tft.textEnlarge(2);
}



void display_readings(void *p)
{
	display_image();
	int i = 0;
	while(1)
	{
		snprintf(lsReading, 100, "Light Reading = %d", i);
		tft.textSetCursor(100, 40);
		tft.textWrite(lsReading);
		snprintf(tempReading, 100, "Temperature Reading = %d C", ++i);
		tft.textSetCursor(100, 150);
		tft.textWrite(tempReading);
		snprintf(humidityReading, 100, "Humidity Reading = %d", ++i);
		tft.textSetCursor(100, 260);
		tft.textWrite(humidityReading);
		tft.layerEffect(OR);
		delay_ms(1000);
		if(i == 1000)
			i = 0;
	}
}

void Motor_drive(void * p)
{
	static GPIO myPin(P1_29);
	myPin.setAsOutput();
	myPin.enablePullUp();
	myPin.setHigh();
	while(1)
	{
		myPin.setHigh();

		//LPC_GPIO2->FIOCLR = (1 << 3);  // 0
		//printf("------%d - %d\n", getRawLightValue(), getPercentLightValue());
		delay_ms(1000);
	}
}





