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

/*
 * Read and calculate accelerometer position from on board sensor
 */
void readAccelometerData(float * update_data);

/*
 * Read and calculate gyroscope position from on board sensor
 */
void readGyroscopeData(float * update_data);

/*
 * Read and calculate compass position from on board sensor
 */
void readCompassData(float * update_data);

#endif /* INC_SENSOR_H_ */
