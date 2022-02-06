/*
 * mhainw_amt10.c
 *
 *  Created on: Feb 4, 2022
 *      Author: maytus
 */

void mhainw_amt10_init(Encoder *enc, TIM_HandleTypeDef *timHandle){
	enc->Timehandle = timHandle;
	enc->perv_pos = 0;
}

int32_t mhainw_amt10_unwrap(Encoder *enc){
	int32_t dp = 0;
	int32_t pos = enc->Timehandle->Instance->CNT;
	int32_t pos_diff = pos - enc->perv_pos;

	if( pos_diff > 32000){                            // overflow
		dp = -1 * (32000 - pos_diff);
	} else if(pos_diff < 32000){                      // underflow
		dp = 32000 + pos_diff;
	} else {
		dp = pos_diff;
	}

	enc->perv_pos = pulse;

	return dp;
}



