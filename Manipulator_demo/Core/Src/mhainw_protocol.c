/*
 * mhainw_protocol.c
 *
 *  Created on: Feb 6, 2022
 *      Author: maytus
 */

#include "mhainw_protocol.h"
#include "usart.h"

void mhainw_protocol_init(Protocol *uart,UART_HandleTypeDef *handle){
	/*
	 *
	 * function use to initialize UART which used in our robot
	 *
	 * */
	uart->handleuart = handle;

	uart->havedata = 0;
	uart->Rxtail = 0;

	uart->goal_reach[0] = 0;
	uart->goal_reach[1] = 0;
	uart->goal_reach[2] = 0;
	uart->goal_reach[3] = 0;

	HAL_UART_Receive_IT(uart->handleuart, &uart->Rxbuffer[uart->Rxtail] , 1);
}

void mhainw_protocol_state(Protocol *uart){
	/*
	 *
	 * function use to verify the package and split the robot to another state
	 *
	 * */

	static Protocolstate state;
	static uint16_t collectdata = 0;

	if(uart->havedata == 1){
		uint8_t datain = uart->Rxbuffer[uart->Rxtail];
		switch(state){
			//check header
			case idle:
				if(datain == 0xFF){
					state = header;
				}
				break;
			// input length
			case header:
				uart->len = datain;
				state = len;
				break;
			// input instruction
			case len:
				uart->inst = datain;
				state = inst;
				break;
			// check length of package
			case inst:
				if(uart->len > 2){
					uart->data[collectdata] = datain; //collect first parameter
					collectdata++;
					state = collect;
					break;
				} else{
					uart->checksum = datain;
					state = checksum;
				}

			// collect data
			case collect:
				if(collectdata < uart->len-2){
					uart->data[collectdata] = datain;
					collectdata++;
					break;
				} else {
					uart->checksum = datain;
					state = checksum;
				}
			// check checksum of package
			case checksum:
				uart->cal_checksum = uart->inst + uart->len;
				for(int i = 0;i < collectdata ; i++){
					uart->cal_checksum += uart->data[i];
				}

				uart->cal_checksum = ~uart->cal_checksum & 0xFF;

				// checksum successful
				if(uart->cal_checksum == uart->checksum){
					uart->package_verify = 1;
				}
				else{
					UARTsentERR(uart,MHAINW_CHECKSUM_ERR);
				}
				state = idle;
				collectdata = 0;
				break;

			default:
				UARTsentERR(uart, MHAINW_HEADER_ERR);

		}
		uart->havedata = 0;
	}
}


void mhainw_protocol_updateRxtail(Protocol *uart){
	/*
	 *
	 * function is use to update the last index in Rxbuffer and its use to first index to fill data in Rxbuffer
	 *
	 * */
	uart->havedata = 1;
	mhainw_protocol_state(uart);
	uart->Rxtail = (uart->Rxtail + 1) % sizeof(uart->Rxbuffer);
	HAL_UART_Receive_IT(uart->handleuart, &uart->Rxbuffer[uart->Rxtail], 1);

}

void mhainw_protocol_sentdata(Protocol *uart,uint8_t *pData, uint16_t len){
	/*
	 *
	 * function use to arrangement the data and fill in to the Tx buffer
	 *
	 * */

	uint16_t Txlen = sizeof(uart->Txbuffer);

	//check length of data is more than buffer
	uint16_t lendata = (len <= Txlen) ? len : Txlen;  //len=3

	//copy data to Txbuffer
	uint16_t cancpy = (lendata <= Txlen - uart->Txhead) ? lendata : Txlen - uart->Txhead;

	memcpy(&(uart->Txbuffer[uart->Txhead]), pData, cancpy);

	//move head to new position
	uart->Txhead = (uart->Txhead + lendata) % Txlen;

	if(lendata != cancpy){
		memcpy(uart->Txbuffer, pData+cancpy, lendata - cancpy);
	}
	UARTsendit(uart);
}

void UARTsendit(Protocol *uart){
	/*
	 *
	 * function use to sent the data in Tx buffer
	 *
	 * */

	uint16_t Txlen = sizeof(uart->Txbuffer);

	if(uart->Txtail != uart->Txhead){
		uint16_t sentlen = (uart->Txhead > uart->Txtail) ?
				uart->Txhead - uart->Txtail : Txlen - uart->Txhead;

		HAL_UART_Transmit_IT(uart->handleuart, &(uart->Txbuffer[uart->Txtail]),
							sentlen);

		uart->Txtail = (uart->Txtail + sentlen) % Txlen;
	}
}

uint8_t UARTgetRxhead(Protocol *uart){
	/*
	 *
	 * function the use get the first position in buffer to receive the data
	 *
	 * */
	return sizeof(uart->Rxbuffer) - uart->handleuart->RxXferCount;

}

void UARTsentERR(Protocol *uart,uint8_t errtype){
	/*
	 *
	 * function the use to send set of package for alert the error of package
	 *
	 * */
	uint8_t temp[] = { MHAINW_HEADER, 0x02 , errtype};
	mhainw_protocol_sentdata(uart,temp,sizeof(temp));
}

void UARTsentACK(Protocol *uart,uint8_t ack){
	/*
	 *
	 * function the use to send set of package for acknowledge start or finish command
	 *
	 * */
	uint8_t temp[] = { 0xFF, 0x02 , ack};
	mhainw_protocol_sentdata(uart,temp,sizeof(temp));
}



