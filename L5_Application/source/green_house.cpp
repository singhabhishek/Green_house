/*
 * green_house.cpp
 *
 *  Created on: Apr 30, 2017
 *      Author: Abhishek
 */

#include <stdio.h>
#include "green_house.hpp"
#include "Light_sensor_temt6000.hpp"
#include "utilities.h"
#include "io.hpp"
#include "Adafruit_RA8875-master/Adafruit_RA8875.h"
#include "printf_lib.h"
#include "bme280_sensor.hpp"
#include "tasks.hpp"


Adafruit_RA8875 tft(3,4);
char lsReading[100];
char tempReading[100];
char humidityReading[100];

struct senReadings_t
{
	char light_sensor_percentage;
	uint16_t light_sensor_raw;
	float temperature_sensor;
	float humidity_sensor;
};
struct senReadings_t readings;


void sensor_readings(void * p)
{
	bool status = init();
	if (!status)
	{
		u0_dbg_printf("Could not find a valid BME280 sensor, check wiring!\n");
	}
	else
	{
		u0_dbg_printf("Init BME280 sensor done!\n");
	}
	while(1)
	{
		readings.light_sensor_raw = getRawLightValue();
		readings.light_sensor_percentage = getPercentLightValue();
		readings.temperature_sensor = readT();
		readings.humidity_sensor = readH();
		u0_dbg_printf("%d - %d - %f - %f\n", readings.light_sensor_raw, readings.light_sensor_percentage,
											 readings.temperature_sensor, readings.humidity_sensor);
		delay_ms(100);
	}
}

bool display_image()
{
	if(!tft.init_display(RA8875_800x480, RA8875_PWM_CLK_DIV1024))
	{
		u0_dbg_printf("RA8875 Not Found!\n");
		return false;
	}
	tft.graphicsMode();
	tft.setLayer(L1);
	tft.fillScreen(RA8875_RED);
	tft.setLayer(L2);
	tft.textMode();
    tft.textColor(RA8875_BLUE,RA8875_WHITE);
	tft.textEnlarge(2);
	return true;
}

void display_reading()
{
	snprintf(lsReading, 100, "Light Reading = %d", readings.light_sensor_percentage);
	tft.textSetCursor(100, 40);
	tft.textWrite(lsReading);
	snprintf(tempReading, 100, "Temperature Reading = %.2f C", readings.temperature_sensor);
	tft.textSetCursor(100, 150);
	tft.textWrite(tempReading);
	snprintf(humidityReading, 100, "Humidity Reading = %.2f", readings.humidity_sensor);
	tft.textSetCursor(100, 260);
	tft.textWrite(humidityReading);
	tft.layerEffect(OR);
}

void actuator_action()
{
	if(readings.temperature_sensor > 30)
	{
		LE.on(1); // Represents Fan: ON
	}
	else
	{
		LE.off(1); // // Represents Fan: OFF
	}

	if(readings.humidity_sensor < 40)
	{
		LE.on(2); // Represents Sprinkler: ON
	}
	else
	{
		LE.off(2); // // Represents Sprinkler: OFF
	}

	if(readings.light_sensor_percentage < 50)
	{
		LE.on(3); // Represents Bulb: ON
	}
	else
	{
		LE.off(3); // Represents Bulb: OFF
	}

}

void display_and_actuate(void *p)
{
	bool lcd_status  = display_image();
	while(1)
	{
		actuator_action();
		if(lcd_status == true)
			display_reading();
		delay_ms(1000);
	}
}
