/*
 * bme280_sensor.hpp
 *
 *  Created on: May 6, 2017
 *      Author: Abhishek
 */

#ifndef L4_IO_BME280_SENSOR_HPP_
#define L4_IO_BME280_SENSOR_HPP_

#include <stdint.h>
#include <stdbool.h>

bool init();
float readT();
float readH();

#endif /* L4_IO_BME280_SENSOR_HPP_ */
