/*
 * bme280_sensor.cpp
 *
 *  Created on: May 6, 2017
 *      Author: Abhishek
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "i2c2.hpp"
#include "printf_lib.h"
#include "utilities.h"
I2C2& i2cc = I2C2::getInstance();

#define BME280_ADDRESS                (0xEE)
#define BME280_REGISTER_DIG_T1        (0x88)
#define BME280_REGISTER_DIG_T2        (0x8A)
#define BME280_REGISTER_DIG_T3        (0x8C)
#define BME280_REGISTER_CHIPID        (0xD0)
#define BME280_REGISTER_TEMPDATA      (0xFA)
#define BME280_REGISTER_CONTROLHUMID  (0xF2)
#define BME280_REGISTER_STATUS        (0XF3)
#define BME280_REGISTER_CONTROL       (0xF4)
#define BME280_REGISTER_CONFIG        (0xF5)
#define BME280_REGISTER_SOFTRESET     (0xE0)
#define BME280_REGISTER_HUMIDDATA     (0xFD)
#define BME280_REGISTER_DIG_H1        (0xA1)
#define BME280_REGISTER_DIG_H2        (0xE1)
#define BME280_REGISTER_DIG_H3        (0xE3)
#define BME280_REGISTER_DIG_H4        (0xE4)
#define BME280_REGISTER_DIG_H5        (0xE5)
#define BME280_REGISTER_DIG_H6        (0xE7)


static int32_t   t_fine;

enum sensor_sampling {
	SAMPLING_NONE = 0b000,
	SAMPLING_X1   = 0b001,
	SAMPLING_X2   = 0b010,
	SAMPLING_X4   = 0b011,
	SAMPLING_X8   = 0b100,
	SAMPLING_X16  = 0b101
};

enum sensor_mode {
	MODE_SLEEP  = 0b00,
	MODE_FORCED = 0b01,
	MODE_NORMAL = 0b11
};

enum sensor_filter {
	FILTER_OFF = 0b000,
	FILTER_X2  = 0b001,
	FILTER_X4  = 0b010,
	FILTER_X8  = 0b011,
	FILTER_X16 = 0b100
};

// standby durations in ms
enum standby_duration {
	STANDBY_MS_0_5  = 0b000,
	STANDBY_MS_10   = 0b110,
	STANDBY_MS_20   = 0b111,
	STANDBY_MS_62_5 = 0b001,
	STANDBY_MS_125  = 0b010,
	STANDBY_MS_250  = 0b011,
	STANDBY_MS_500  = 0b100,
	STANDBY_MS_1000 = 0b101
};

// The config register
struct config {
	// inactive duration (standby time) in normal mode
	// 000 = 0.5 ms
	// 001 = 62.5 ms
	// 010 = 125 ms
	// 011 = 250 ms
	// 100 = 500 ms
	// 101 = 1000 ms
	// 110 = 10 ms
	// 111 = 20 ms
	unsigned int t_sb : 3;

	// filter settings
	// 000 = filter off
	// 001 = 2x filter
	// 010 = 4x filter
	// 011 = 8x filter
	// 100 and above = 16x filter
	unsigned int filter : 3;

	// unused - don't set
	unsigned int none : 1;
	unsigned int spi3w_en : 1;

	unsigned int get() {
		return (t_sb << 5) | (filter << 3) | spi3w_en;
	}
};
config _configReg;


// The ctrl_meas register
struct ctrl_meas {
	// temperature oversampling
	// 000 = skipped
	// 001 = x1
	// 010 = x2
	// 011 = x4
	// 100 = x8
	// 101 and above = x16
	unsigned int osrs_t : 3;

	// pressure oversampling
	// 000 = skipped
	// 001 = x1
	// 010 = x2
	// 011 = x4
	// 100 = x8
	// 101 and above = x16
	unsigned int osrs_p : 3;

	// device mode
	// 00       = sleep
	// 01 or 10 = forced
	// 11       = normal
	unsigned int mode : 2;

	unsigned int get() {
		return (osrs_t << 5) | (osrs_p << 3) | mode;
	}
};
ctrl_meas _measReg;


// The ctrl_hum register
struct ctrl_hum {
	// unused - don't set
	unsigned int none : 5;

	// pressure oversampling
	// 000 = skipped
	// 001 = x1
	// 010 = x2
	// 011 = x4
	// 100 = x8
	// 101 and above = x16
	unsigned int osrs_h : 3;

	unsigned int get() {
		return (osrs_h);
	}
};
ctrl_hum _humReg;

uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
uint16_t v;
int32_t var1, var2;
int32_t adc_T;
uint8_t data[3] = {0};

uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;
int32_t adc_H;
int32_t v_x1_u32r;

bool isReadingCalibration(void)
{
  uint8_t const rStatus = i2cc.readReg(BME280_ADDRESS, BME280_REGISTER_STATUS);
  return (rStatus & (1 << 0)) != 0;
}



void setSampling(sensor_mode       mode,
		 sensor_sampling   tempSampling,
		 sensor_sampling   pressSampling,
		 sensor_sampling   humSampling,
		 sensor_filter     filter,
		 standby_duration  duration)
{
    _measReg.mode     = mode;
    _measReg.osrs_t   = tempSampling;
    _measReg.osrs_p   = pressSampling;

    _humReg.osrs_h    = humSampling;
    _configReg.filter = filter;
    _configReg.t_sb   = duration;

    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
    i2cc.writeReg(BME280_ADDRESS, BME280_REGISTER_CONTROLHUMID, _humReg.get());
    i2cc.writeReg(BME280_ADDRESS, BME280_REGISTER_CONFIG, _configReg.get());
    i2cc.writeReg(BME280_ADDRESS, BME280_REGISTER_CONTROL, _measReg.get());
}



bool init()
{
    if (i2cc.readReg(BME280_ADDRESS, BME280_REGISTER_CHIPID) != 0x60)
    {
        return false;
    }
    i2cc.writeReg(BME280_ADDRESS, BME280_REGISTER_SOFTRESET, 0xB6);
    delay_ms(300);
    while (isReadingCalibration())
          delay_ms(100);
    setSampling(MODE_NORMAL, SAMPLING_X16, SAMPLING_X16, SAMPLING_X16, FILTER_OFF, STANDBY_MS_0_5);
	return true;
}

float readT()
{
	int32_t var1, var2;
	int32_t adc_T;
	float T;

	i2cc.readRegisters(BME280_ADDRESS, BME280_REGISTER_TEMPDATA, data, 24);
	adc_T = ((data[0]<<16) | (data[0]<<8) | (data[1] & 0xFF));
	if (adc_T == 0x800000)
		return NAN;
	adc_T >>= 4;

	i2cc.readRegisters(BME280_ADDRESS, BME280_REGISTER_DIG_T1, data, 16);
	v = ((data[0]<<8) | (data[1] & 0xFF));
	dig_T1 = ((v >> 8) | (v << 8));

	i2cc.readRegisters(BME280_ADDRESS, BME280_REGISTER_DIG_T2, data, 16);
	v = ((data[0]<<8) | (data[1] & 0xFF));
	dig_T2 = (int16_t)((v >> 8) | (v << 8));

	i2cc.readRegisters(BME280_ADDRESS, BME280_REGISTER_DIG_T3, data, 16);
	v = ((data[0]<<8) | (data[1] & 0xFF));
	dig_T3 = (int16_t)((v >> 8) | (v << 8));

	var1 = ((((adc_T>>3) - ((int32_t)dig_T1 <<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;

	T = (t_fine * 5 + 128) >> 8;
	return (T/100);
}


float readH(void)
{
    readT(); // must be done first to get t_fine
	i2cc.readRegisters(BME280_ADDRESS, BME280_REGISTER_HUMIDDATA, data, 16);
	adc_H = ((data[0]<<8) | (data[1] & 0xFF));
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;

    dig_H1 = i2cc.readReg(BME280_ADDRESS, BME280_REGISTER_DIG_H1);

    i2cc.readRegisters(BME280_ADDRESS, BME280_REGISTER_DIG_H2, data, 16);
    v = ((data[0]<<8) | (data[1] & 0xFF));
    dig_H2 = (int16_t)((v >> 8) | (v << 8));

    dig_H3 = i2cc.readReg(BME280_ADDRESS, BME280_REGISTER_DIG_H3);
    dig_H4 = (int16_t)(i2cc.readReg(BME280_ADDRESS, BME280_REGISTER_DIG_H4) << 4) | (i2cc.readReg(BME280_ADDRESS, BME280_REGISTER_DIG_H4+1) & 0xF);
    dig_H5 = (int16_t)(i2cc.readReg(BME280_ADDRESS, BME280_REGISTER_DIG_H5+1) << 4) | (i2cc.readReg(BME280_ADDRESS, BME280_REGISTER_DIG_H5) >> 4);
    dig_H6 = (int8_t)i2cc.readReg(BME280_ADDRESS, BME280_REGISTER_DIG_H6);




    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
                    (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    return  (v_x1_u32r>>12) / 1024.0;
}



