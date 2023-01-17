/*
 * transmit_data.c
 *
 *  Created on: Oct 20, 2022
 *      Author: Michał Dydo
 */
#include "transmit_data.h"
// Rozmiary naglowkow i stopki do pakietu transmisji
#define  PACKET_CONTROL_HEADER_SIZE 4
#define  PACKET_CONTROL_TERMINATOR_SIZE 2

// Bajty naglowka pakietu
const uint8_t PACKET_CONTROL_HEADER [PACKET_CONTROL_HEADER_SIZE]= {170, 170, 170, 170};
// Bajty stopki pakietu
const char PACKET_CONTROL_TERMINATOR[PACKET_CONTROL_TERMINATOR_SIZE]= "\r\n";

// zewnetrzny wskaznik do UART'a do komunikacji z PC
UART_HandleTypeDef* uart_to_pc_ptr = NULL;

void send_control_packet(packet_with_control data_to_send){
	char string_to_send[40];	// bufor typu string
	// formatowanie danych do zmiennej typu string
	size_t string_to_send_size = sprintf(string_to_send, "%2.2f %2.2f", data_to_send.control, data_to_send.control2);
	// wyslij sformatowane dane do PC
	HAL_UART_Transmit(uart_to_pc_ptr, string_to_send, string_to_send_size, HAL_MAX_DELAY);
	// wyslij bajty stopki
	HAL_UART_Transmit(uart_to_pc_ptr, PACKET_CONTROL_TERMINATOR, PACKET_CONTROL_TERMINATOR_SIZE, HAL_MAX_DELAY);
}
