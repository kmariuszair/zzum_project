/*
 * sensor.c
 *
 *  Created on: 8 lis 2022
 *      Author: Michał Dydo
 */
#include "sensor.h"

static uint8_t spi_acc_ver_counter = 0;	// counter to verification SPI bus
static uint8_t spi_mag_ver_counter = 0;	// counter to verification SPI bus


void readAccelometerData(float * update_data){
	float formated_data[3];
	int16_t buffer[3];
	BSP_COMPASS_AccGetXYZ(buffer);
	for (size_t itr = 0; itr < 3; ++itr)
		formated_data[itr] = buffer[itr] / 32768.0 * 2.0 ;	// scale acceleration to +/- 2G
	memcpy(update_data, formated_data, 3*sizeof(float));
	// check SPI bus communication
	if(spi_acc_ver_counter % 8 ==0 ) {
		uint8_t ctrl = ACCELERO_IO_Read(WHO_I_AM_ADDRESS);
		if (ctrl != WHO_I_AMD_ACC)
			if (ctrl != WHO_I_AMD_MAG){
				BSP_COMPASS_Init();// reset accelerometer and magnetometer if device is not responding
				BSP_GYRO_Init();
			}
	}
	spi_acc_ver_counter +=1; // increase counter
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
	// check SPI bus communication
	if(spi_mag_ver_counter % 8 == 0 ) {
		uint8_t ctrl = MAGNETO_IO_Read(WHO_I_AM_ADDRESS);
		if (ctrl != WHO_I_AMD_MAG){
			BSP_COMPASS_Init();// reset accelerometer and magnetometer if device is not responding
			BSP_GYRO_Init();
		}
	}
	spi_mag_ver_counter +=1; // increase counter

}


