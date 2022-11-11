/*
 * sensor.c
 *
 *  Created on: 8 lis 2022
 *      Author: Michał Dydo
 */
#include "sensor.h"


void readAccelometerData(float * update_data){
	float formated_data[3];
	int16_t buffer[3];
	BSP_COMPASS_AccGetXYZ(buffer);
	for (size_t itr = 0; itr < 3; ++itr)
		formated_data[itr] = buffer[itr] / 32768.0 * 2.0 ;	// scale acceleration to +/- 2G
	memcpy(update_data, formated_data, 3*sizeof(float));
}

void readGyroscopeData(float * update_data){
	float buffer[3];
	// read data from gyroscope
	BSP_GYRO_GetXYZ(buffer);
	// stop function if readed data is smaller than 2000
	for (size_t itr = 0; itr < 3; ++itr)
		buffer[itr] = buffer[itr] / 32768.0;	// scale acceleration to +/- 2G
	char stop = 1;
	// copy new data to input argument
	memcpy(update_data, buffer, 3*sizeof(float));
}

void readCompassData(float * update_data){
	int16_t buffer[3];
	float formated_data[3];
	BSP_COMPASS_MagGetXYZ(buffer);
	for (size_t itr = 0; itr < 3; ++itr)
		formated_data[itr] = (float)buffer[itr] / 32768.0 * 16.0 ;	// scale acceleration to +/- 2G
	// copy new data to input argument
	memcpy(update_data, formated_data, 3*sizeof(float));
}


