/*
 * transmit_data.h
 *
 *  Created on: Oct 20, 2022
 *      Author: Michał Dydo
 *
 *  Ten plik zwiera definicje funkcji i stalych sluzacych do transmisji sterowan do PC
 */

#ifndef SRC_TRANSMIT_DATA_HPP_
#define SRC_TRANSMIT_DATA_HPP_

// dodanie odpowiednich bibliotek
#include "stm32l4xx_hal.h"

// zewnetrzny wskaznik do komunikacji z PC
extern UART_HandleTypeDef* uart_to_pc_ptr;

/*
 * Struktura z danymi do przeslania
 * Bajty stopki i naglowka sa uwzgledniane w funkcji wysylajacej
 */
typedef struct PacketWithControl{
	// dane pakietu
	float control;	// dane ze sterowaniem do modelu
	float control2; //
}packet_with_control ;

/*
 * Funkcja z pakietem do wyslanina
 */
void send_control_packet(packet_with_control data_to_send);

#endif /* SRC_TRANSMIT_DATA_HPP_ */
