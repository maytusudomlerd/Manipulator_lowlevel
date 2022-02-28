/*
 * mhainw_kalmanfilter.h
 *
 *  Created on: Feb 23, 2022
 *      Author: U. Maytus
 */

#ifndef INC_MHAINW_KALMANFILTER_H_
#define INC_MHAINW_KALMANFILTER_H_

#define dt 0.002

typedef struct{
	float R;
	float Q;

	float x1,x2;
	float p11,p12,p21,p22;
	float detP;
}Kalmanfilter;

void mhainw_kalmanfilter_init(Kalmanfilter *kalman,float x1,float x2,float p11,float p12,float p21,float p22,float Q,float R);
void mhainw_kalmanfilter_updatekalman(Kalmanfilter *kalman,float theta);

#endif /* INC_MHAINW_KALMANFILTER_H_ */
