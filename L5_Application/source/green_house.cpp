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
char lsReading[30];
char tempReading[30];
char humidityReading[30];
char sprinkler[20];
char bulb[20];
char fan[20];

bool sprinkler_status;
bool bulb_status;
bool fan_status;

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
		vTaskDelay(100);
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
	tft.textMode();
	tft.textEnlarge(2);
	return true;
}

void display_reading()
{
	tft.fillScreen(RA8875_BLUE);
    tft.textColor(RA8875_WHITE, RA8875_BLACK);
	snprintf(lsReading, 30, "Light Reading = %d", readings.light_sensor_percentage);
	tft.textSetCursor(190, 40);
	tft.textWrite(lsReading);
	snprintf(tempReading, 30, "Temperature Reading = %.2f C", readings.temperature_sensor);
	tft.textSetCursor(50, 150);
	tft.textWrite(tempReading);
	snprintf(humidityReading, 30, "Humidity Reading = %.2f", readings.humidity_sensor);
	tft.textSetCursor(120, 250);
	tft.textWrite(humidityReading);
	tft.layerEffect(OR);
	snprintf(sprinkler, 20, "Sprinkler=%s", sprinkler_status?" ON":"OFF");
	sprinkler_status?tft.textColor(RA8875_WHITE, RA8875_RED):tft.textColor(RA8875_WHITE, RA8875_BLACK);
	tft.textSetCursor(10, 370);
	tft.textWrite(sprinkler);
	tft.layerEffect(OR);
	snprintf(bulb, 20, "Bulb=%s", bulb_status?" ON":"OFF");
	bulb_status?tft.textColor(RA8875_WHITE, RA8875_RED):tft.textColor(RA8875_WHITE, RA8875_BLACK);
	tft.textSetCursor(360, 370);
	tft.textWrite(bulb);
	tft.layerEffect(OR);
	snprintf(fan, 20, "Fan=%s", fan_status?" ON":"OFF");
	fan_status?tft.textColor(RA8875_WHITE, RA8875_RED):tft.textColor(RA8875_WHITE, RA8875_BLACK);
	tft.textSetCursor(605, 370);
	tft.textWrite(fan);
	tft.layerEffect(OR);
}

void actuator_action()
{
	if(readings.temperature_sensor > 30)
	{
		LE.on(1); // Represents Fan: ON
		fan_status = true;
	}
	else
	{
		LE.off(1); // // Represents Fan: OFF
		fan_status = false;
	}

	if(readings.humidity_sensor < 40)
	{
		LE.on(2); // Represents Sprinkler: ON
		sprinkler_status = true;
	}
	else
	{
		LE.off(2); // // Represents Sprinkler: OFF
		sprinkler_status = false;
	}

	if(readings.light_sensor_percentage < 50)
	{
		LE.on(3); // Represents Bulb: ON
		bulb_status = true;
	}
	else
	{
		LE.off(3); // Represents Bulb: OFF
		bulb_status = false;
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
		vTaskDelay(500);
	}
}
