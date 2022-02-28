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
 * function that update kalman parameter use 2 input first is struct kalman that keep K interations kalman parameter
 * second is theta(radent) that value are read form encoder
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

	kalman->x1 = (4*R*x1 + 4*p11*theta + 4*dt_pow2*p22*theta + 4*R*dt*x2 + 4*dt*p12*theta + 4*dt*p21*theta + Q*dt_pow4*theta)/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt_pow4 + 4*dt_pow2*p22);
	kalman->x2 = x2 - (((Q*dt_pow3)/2 + p22*dt + p21)*(x1 - theta + dt*x2))/(R + p11 + dt*p21 + (Q*dt_pow4)/4 + dt*(p12 + dt*p22));
	kalman->p11 = (R*(4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt_pow4 + 4*dt_pow2*p22))/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt_pow4 + 4*dt_pow2*p22);
	kalman->p12 = (2*R*(Q*dt_pow3 + 2*p22*dt + 2*p12))/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt_pow4 + 4*dt_pow2*p22);
	kalman->p21 = (2*R*(Q*dt_pow3 + 2*p22*dt + 2*p21))/(4*R + 4*p11 + 4*dt*p12 + 4*dt*p21 + Q*dt_pow4 + 4*dt_pow2*p22);
	kalman->p22 = p22 + Q*dt_pow2 - (((Q*dt_pow3)/2 + p22*dt + p12)*((Q*dt_pow3)/2 + p22*dt + p21))/(R + p11 + dt*p21 + (Q*dt_pow4)/4 + dt*(p12 + dt*p22));

#ifdef kalman_tuning
	kalman->detP = (p11 * p22) - (p12 * p21);
#endif
}

