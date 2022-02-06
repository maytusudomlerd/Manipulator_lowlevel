/*
 * mhainw_protocol.c
 *
 *  Created on: Feb 6, 2022
 *      Author: maytus
 */

#include "mhainw_protocol.h"

void mhainw_protocol_init(Protocol *uart,UART_HandleTypeDef *handle){
	uart->handleuart = handle;
	uart->havedata = 0;
	uart->Rxtail = 0;

	HAL_UART_Receive_IT(uart.handleuart, uart.Rxbuffer , 1);
}

void mhainw_protocol_state(Protocol *uart){

	static Protocolstate state;
	static collectdata = 0;

	if(uart->havedata == 1){
		uint8_t datain = uart->Rxbuffer[uart->Rxtail];
		switch(state){
			//check header
			case idle:
				if(datain == MHAINW_HEADER){
					state = header;
				}
				break;
			// input length
			case header:
				uart->len = datain;
				state = inst;
				break;
			// input instruction
			case len:
				uart->inst = datain;
				state = inst;
			// check length of package
			case inst:
				if(uart->len > 2){
					uart.data[collectdata] = datain; //collect first parameter
					collectdata++;
					state = collect;
				} else{
					uart->checksum = datain;
					state = checksum;
				}
				break;
			// collect data
			case collect:
				if(collectdata <= uart->len-2){
					uart.data[collectdata] = datain;
					collectdata++;
				} else {
					uart.checksum = datain;
					state = checksum;
				}
				break;
			// check checksum of package
			case checksum:
				cal_checksum = uart->inst + uart->len;
				for(int i = 0;i < collectdata ; i++){
					uart->cal_checksum += uart.data[i];
				}
				uart->cal_checksum = ~(uart->cal_checksum & 0xFF);

				// checksum successful
				if(uart->cal_checksum == uart->checksum){
					switch(uart->inst){
					case 0x10: //sethome
						break;
					case 0x20: //catesian jog
						break;
					case 0x21: //joint jog
						break;
					case 0x30: //catesian move
						break;
					case 0x31: //joint move
						break;
					case 0x40: //taskmove traj + casecade control
						break;
					case 0x50: //griper
						break;
					case 0x51: //gripper read current
						break;
					case 0x52: //gripper set current
						break;
					case 0x60: //unit test encoder
						break;
					case 0x61: //motor test
						break;
					case 0x62: //proximity test
						break;
					}
				}
//				else{
//					//send checksum error
//
//				}

				state = idle;

				break;

//			default:
//				//send package error

		collectdata = 0;
		uart->havedata = 0;

		}
	}
}

//call in interrupt
void mhainw_protocol_updateRxtail(Protocol *uart){
	if(uart->Rxtail != UARTgetRxhead(uart)){
		uart->Rxtail = (uart->Rxtail + 1) % 100;
		uart->havedata = 1;
	}
}

uint8_t UARTgetRxhead(Protocol *uart){
	return 100 - uart->handleuart->RxXferCount;

}

