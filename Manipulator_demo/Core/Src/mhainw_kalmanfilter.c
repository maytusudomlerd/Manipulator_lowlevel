/*
 * mhainw_kalmanfilter.c
 *
 *  Created on: Feb 23, 2022
 *      Author: U. Maytus
 */
#include "mhainw_kalmanfilter.h"

#define kalman_tuning

void mhainw_kalmanfilter_init(Kalmanfilter *kalman,float x1,float x2,float p11,float p12,float p21,float p22,float Q,float R){
	kalman->x1 = x1;
	kalman->x2 = x2;

	kalman->p11 = p11;
	kalman->p12 = p12;
	kalman->p21 = p21;
	kalman->p22 = p22;

#ifdef kalman_tuning
	kalman->detP = (p11 * p22) - (p12 * p21);
#endif

	kalman->Q = Q;
	kalman->R = R;
}
/*
 * function that update kalman parameter use 2 input first is struck kalman that keep K iterations kalman parameter
 * second is theta(radiant) that value are read from encoder
 *
 */
void mhainw_kalmanfilter_updatekalman(Kalmanfilter *kalman,float theta){
	float x1 = (kalman->x1);
	float x2 = (kalman->x2);
	float p11 = (kalman->p11);
	float p12 = (kalman->p12);
	float p21 = (kalman->p21);
	float p22 = (kalman->p22);
	float Q = (kalman->Q);
	float R = (kalman->R);
	float dt_pow2 = dt * dt;
	float dt_pow3 = dt * dt * dt;
	float dt_pow4 = dt * dt * dt * dt;

	kalman->x1 = x1 + x2*dt - ((x1 - theta + x2*dt)*(p11 + p21*dt + (Q*dt_pow4)/4 + dt*(p12 + p22*dt)))/(p11 + R + p21*dt + (Q*(dt_pow4))/4 + dt*(p12 + p22*dt));
	kalman->x2 = x2 - (((Q*(dt_pow3))/2 + p22*dt + p21)*(x1 - theta + x2*dt))/(p11 + R + p21*dt + (Q*(dt_pow4))/4 + dt*(p12 + p22*dt));
	kalman->p11 = -((p11 + p21*dt + (Q*(dt_pow4))/4 + dt*(p12 + p22*dt))/(p11 + R + p21*dt + (Q*(dt_pow4))/4 + dt*(p12 + p22*dt)) - 1)*(p11 + p21*dt + (Q*(dt_pow4))/4 + dt*(p12 + p22*dt));
	kalman->p12 = -((p11 + p21*dt + (Q*(dt_pow4))/4 + dt*(p12 + p22*dt))/(p11 + R + p21*dt + (Q*(dt_pow4))/4 + dt*(p12 + p22*dt)) - 1)*((Q*(dt_pow3))/2 + p22*dt + p12);
	kalman->p21 = p21 + p22*dt + (Q*(dt_pow3))/2 - (((Q*(dt_pow3))/2 + p22*dt + p21)*(p11 + p21*dt + (Q*(dt_pow4))/4 + dt*(p12 + p22*dt)))/(p11 + R + p21*dt + (Q*(dt_pow4))/4 + dt*(p12 + p22*dt));
	kalman->p22 = p22 + Q*dt_pow2 - (((Q*dt_pow3)/2 + p22*dt + p12)*((Q*dt_pow3)/2 + p22*dt + p21))/(R + p11 + dt*p21 + (Q*dt_pow4)/4 + dt*(p12 + dt*p22));
#ifdef kalman_tuning
	kalman->detP = (p11 * p22) - (p12 * p21);
#endif
}

