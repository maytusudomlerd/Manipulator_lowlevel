/*
 * mhainw_protocol.h
 *
 *  Created on: Feb 6, 2022
 *      Author: maytu
 */

#ifndef INC_MHAINW_PROTOCOL_H_
#define INC_MHAINW_PROTOCOL_H_

#define MHAINW_HEADER 0xFF

typedef enum{
	idle,
	header,
	len,
	collect,
	checksum,
	inst
}Protocolstate;

typedef struct{
	UART_HandleTypeDef *handleuart;
	uint8_t Rxbuffer[100];
	uint8_t Txbuffer[100];
	uint8_t data[100];
	uint8_t Rxtail;
	uint8_t len;
	uint8_t inst;
	uint8_t checksum;
	uint16_t cal_checksum;
	uint8_t havedata;
}Protocol;

void mhainw_protocol_init(Protocol *uart,UART_HandleTypeDef *handle);
void mhainw_protocol_state(Protocol *uart);
void mhainw_protocol_updateRxtail(Protocol *uart);

uint8_t UARTgetRxhead(Protocol *uart);

#endif /* INC_MHAINW_PROTOCOL_H_ */
