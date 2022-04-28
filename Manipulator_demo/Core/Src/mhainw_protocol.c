/*
 * mhainw_protocol.c
 *
 *  Created on: Feb 6, 2022
 *      Author: maytus
 */

#include "mhainw_protocol.h"
#include "usart.h"

extern uint8_t numberofdata;

void mhainw_protocol_init(Protocol *uart,UART_HandleTypeDef *handle){
	/*
	 *
	 * function use to initialize UART which used in our robot
	 *
	 * */
	uart->handleuart = handle;

	uart->havedata = 0;
	uart->Rxtail = 0;
	uart->Rxhead = 0;

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
	uint8_t start = uart->Rxhead;
	state = idle;
	if(uart->havedata == 1){
		while(start != (uart->Rxtail + 1)){
			uint8_t datain = uart->Rxbuffer[start];
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
			start++;
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
	static uint8_t state_rx = 0;
	uint8_t len;
	uart->havedata = 1;
	switch(state_rx){
		case 0:
			uart->Rxhead = uart->Rxtail;
			uart->Rxtail = (uart->Rxtail + 1) % sizeof(uart->Rxbuffer);
			HAL_UART_Receive_IT(uart->handleuart, &uart->Rxbuffer[uart->Rxtail], 1);
			state_rx = 1;
			break;
		case 1:
			len = uart->Rxbuffer[uart->Rxtail];
			HAL_UART_Receive_IT(uart->handleuart, &uart->Rxbuffer[uart->Rxtail + 1], len);
			uart->Rxtail = (uart->Rxtail + len) % sizeof(uart->Rxbuffer);
			state_rx = 2;
			break;
		case 2:
			mhainw_protocol_state(uart);
			uart->Rxhead = uart->Rxtail;
			uart->Rxtail = (uart->Rxtail + 1) % sizeof(uart->Rxbuffer);
			HAL_UART_Receive_IT(uart->handleuart, &uart->Rxbuffer[uart->Rxtail], 1);
			state_rx = 0;
			break;
	}

}

void mhainw_protocol_sentdata(Protocol *uart,uint8_t *pData, uint16_t len){
	/*
	 *
	 * function use to arrangement the data and fill in to the Tx buffer
	 *
	 * */

	uint16_t Txlen = sizeof(uart->Txbuffer);

	uart->Txhead = 0;
	uart->Txtail = 0;

//	//copy data to Txbuffer
	memcpy(&(uart->Txbuffer[uart->Txhead]), pData, sizeof(pData));

	//move head to new position
	uart->Txhead = (uart->Txhead + len) % Txlen;
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
		uint16_t sentlen = uart->Txhead - uart->Txtail;

		HAL_UART_Transmit(uart->handleuart, &(uart->Txbuffer[uart->Txtail]),
							sentlen,100);

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
void UARTsentFeedback(Protocol *uart,uint8_t ack,float *data,int lendata){
	/*
	 *
	 * function the use to send set of package for acknowledge start or finish command
	 *
	 * */
	uint8_t size_of_data = ( lendata ) * 2;
	uint8_t temp_feedback[size_of_data + 3];
	uint8_t j =0;
	for(int i=3;i<size_of_data+3;i=i+2){
		temp_feedback[i] = ((int16_t)(data[j] * 100) >> 8) & 0xFF;
		temp_feedback[i+1] = (int16_t)(data[j] * 100) & 0xFF;
		j++;
	}
	temp_feedback[0] = 0xFF;
	temp_feedback[1] = size_of_data + 1;
	temp_feedback[2] = ack;
	mhainw_protocol_sentdata(uart,temp_feedback,sizeof(temp_feedback));

}

void UARTsentGripper(Protocol *uart,uint8_t inst){
	uint8_t temp[4]= {0xFF,0x02,0,0};
	temp[2] = inst;
	int8_t sum = temp[1] + temp[2];
	uint8_t checksum = ~sum & 0xFF;
	temp[3] = checksum;
	mhainw_protocol_sentdata(uart,temp,sizeof(temp));
}


