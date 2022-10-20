/*
 * transmit_data.h
 *
 *  Created on: Oct 20, 2022
 *      Author: Michał Dydo
 *
 *  This file include function definition and constant to data transmission
 */

#ifndef SRC_TRANSMIT_DATA_HPP_
#define SRC_TRANSMIT_DATA_HPP_

// include libs
#include "stm32l4xx_hal.h"

// external pointer uart handler to communicate with PC
extern UART_HandleTypeDef* uart_to_pc_ptr;

/*
 * Structure with data to be transmitted
 * Terminator and header bytes are included in the function
 */
typedef struct PacketWithControl{
	// packet data
	float control;	// control data to model
}packet_with_control ;

/*
 * Function with packet to send
 */
void send_control_packet(packet_with_control data_to_send);

#endif /* SRC_TRANSMIT_DATA_HPP_ */
