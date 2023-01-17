/*
 * sensor.h
 *
 *  Created on: 8 lis 2022
 *      Author: Michał Dydo
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_
// load libraries
#include <math.h>
#include <stm32l476g_discovery.h>
#include <stm32l476g_discovery_gyroscope.h>
#include <stm32l476g_discovery_compass.h>
// Defines
#define WHO_I_AMD_ACC 0b01000001
#define WHO_I_AMD_MAG 0b00111101
#define WHO_I_AM_ADDRESS 0x0F

/*
 * Odczyt i przeliczanie pomiarow z akcelerometru
 */
void readAccelometerData(float * update_data);

/*
 * Odczyt i przeliczanie pomiarow z zyroskopu
 */
void readGyroscopeData(float * update_data);

/*
 * Odczyt i przeliczanie pomiarow z magnetometru
 */
void readCompassData(float * update_data);

#endif /* INC_SENSOR_H_ */
