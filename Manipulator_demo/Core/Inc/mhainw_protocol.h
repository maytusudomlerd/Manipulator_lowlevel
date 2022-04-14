/*
 * mhainw_protocol.h
 *
 *  Created on: Feb 6, 2022
 *      Author: maytus U.
 */

#ifndef INC_MHAINW_PROTOCOL_H_
#define INC_MHAINW_PROTOCOL_H_

#include "usart.h"
#include "string.h"

#define MHAINW_HEADER                         0xFF

//#define MHAINW_INITCHESSPOSE                  0x99

#define MHAINW_WAIT                           0x00
#define MHAINW_IDEL                           0x99

#define MHAINW_SETHOME                        0x10
#define MHAINW_JOG_CATESIAN                   0x20
#define MHAINW_JOG_JOINT                      0x21
#define MAHINW_MOVE_CATESIAN                  0x30
#define MHAINW_MOVE_JOINT                     0x31
#define MHAINW_MOVE_TASK                      0x40
#define MAHINW_MOVE_SETPOINTGEN               0x41
#define MHAINW_MOVE_SETPOINTUPDATE            0x42
#define MHAINW_GRIPPER_GRIP                   0x50
#define MHAINW_GRIPPER_UNGRIP                 0x51
#define MHAINW_REQUIRE_FEEDBACK_ALL           0x60
#define MHAINW_REQUIRE_FEEDBACK_MANIPULATOR   0x61
#define MHAINW_REQUIRE_FEEDBACK_CHESSBOARD    0x62
#define MHAINW_POSITION_CHESSBOARD            0x70
#define MHAINW_POSITION_MANIPULATOR           0x71
#define MHAINW_CHESSBOARD_SETZERO             0x80

#define MHAINW_TRAJGEN                        0xC0


#define MHAINW_SETHOME_ACK                    0xA1
#define MHAINW_JOG_ACK                        0xA2
#define MHAINW_MOVE_ACK                       0xA3
#define MHAINW_TASKMOVE_ACK                   0xA4
#define MHAINW_GRIPPER_ACK                    0xA5
#define MHAINW_FEEDBACK_ACK                   0xA6
#define MHAINW_POSITION_ACK                   0xA7
#define MHAINW_CHESSBOARD_SETZERO_ACK         0xA8

#define MHAINW_HEADER_ERR                     0xEA
#define MHAINW_CHECKSUM_ERR                   0xEB

#define MHAINW_SETHOME_ERR                    0xE1
#define MHAINW_JOG_ERR                        0xE2
#define MHAINW_MOVE_ERR                       0xE3
#define MHAINW_TASKMOVE_ERR                   0xE4
#define MHAINW_GRIPPER_ERR                    0xE5
#define MHAINW_UINT_ERR                       0xE6

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
	uint8_t Rxbuffer[512];
	uint8_t Txbuffer[512];
	int8_t data[100];
	uint8_t Txhead,Rxtail,Txtail;
	uint8_t len;
	uint8_t inst;
	uint8_t checksum;
	uint8_t cal_checksum;
	uint8_t havedata;
	uint8_t goal_reach[4];
	uint8_t package_verify;
}Protocol;

void mhainw_protocol_init(Protocol *uart,UART_HandleTypeDef *handle);
void mhainw_protocol_state(Protocol *uart);
void mhainw_protocol_updateRxtail(Protocol *uart);
void mhainw_protocol_sentdata(Protocol *uart,uint8_t *pData, uint16_t len);

uint8_t UARTgetRxhead(Protocol *uart);
void UARTsentERR(Protocol *uart,uint8_t errtype);
void UARTsentACK(Protocol *uart,uint8_t ack);
void UARTsendit(Protocol *uart);
void UARTsentFeedback(Protocol *uart,uint8_t ack,float *data,int lendata);
void UARTsentGripper(Protocol *uart,uint8_t inst);
#endif /* INC_MHAINW_PROTOCOL_H_ */
