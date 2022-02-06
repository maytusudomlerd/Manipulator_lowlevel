/*
 * mhainw_amt10.h
 *
 *  Created on: Feb 4, 2022
 *      Author: maytu
 */

#ifndef INC_MHAINW_AMT10_H_
#define INC_MHAINW_AMT10_H_

typedef struct{
	TIM_HandleTypeDef *Timehandle;
	uint32_t perv_pos;
}Encoder;


void mhainw_amt10_init(Encoder *enc, TIM_HandleTypeDef *timHandle);
int32_t mhainw_amt10_unwrap(Encoder *enc);


#endif /* INC_MHAINW_AMT10_H_ */
