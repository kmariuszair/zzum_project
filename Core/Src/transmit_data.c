/*
 * transmit_data.c
 *
 *  Created on: Oct 20, 2022
 *      Author: Michał Dydo
 */
#include "transmit_data.h"
// Transmission terminator and header sizes
#define  PACKET_CONTROL_HEADER_SIZE 4
#define  PACKET_CONTROL_TERMINATOR_SIZE 4

// Transmission header bytes
const uint8_t PACKET_CONTROL_HEADER [PACKET_CONTROL_HEADER_SIZE]= {170, 170, 170, 170};
// Transmission terminator bytes
const uint8_t PACKET_CONTROL_TERMINATOR[PACKET_CONTROL_TERMINATOR_SIZE]= {153, 153, 153, 153};

// external pointer uart handler to communicate with PC
UART_HandleTypeDef* uart_to_pc_ptr = NULL;

void send_control_packet(packet_with_control data_to_send){

//	// send header bytes
//		HAL_UART_Transmit(uart_to_pc_ptr, PACKET_CONTROL_HEADER, PACKET_CONTROL_HEADER_SIZE, HAL_MAX_DELAY);
//	// send packet bytes
//	uint8_t* bytes_to_send = (uint8_t*)&data_to_send;
//	HAL_UART_Transmit(uart_to_pc_ptr, bytes_to_send, sizeof(data_to_send), HAL_MAX_DELAY);

	char string_to_send[40];
	size_t string_to_send_size = sprintf(string_to_send, "%f", data_to_send.control);
	HAL_UART_Transmit(uart_to_pc_ptr, string_to_send, string_to_send_size, HAL_MAX_DELAY);
	// send terminator bytes
	HAL_UART_Transmit(uart_to_pc_ptr, PACKET_CONTROL_TERMINATOR, PACKET_CONTROL_TERMINATOR_SIZE, HAL_MAX_DELAY);
}

