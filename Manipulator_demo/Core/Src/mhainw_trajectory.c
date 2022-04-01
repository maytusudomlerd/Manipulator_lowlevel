/*
 * mhainw_trajectory.c
 *
 *  Created on: Feb 28, 2022
 *      Author: K. Peerasate & U. Maytus
 */

#include "mhainw_trajectory.h"

void mhainw_trajectory_init(Trajectory *traj,float delta_t){
	traj->delta_t = delta_t;
	traj->setpoint = 0;
	traj->havetraj =0;
}
void mhainw_trajectory_generatetraj(Trajectory *traj,float *q_i,float *q_f){
	float max_tk = 0;

	for(int i=0;i<4;i++){
		trajectory_generatetraj(&traj[i],q_i[i],traj[i].setpoint);
	}
	//find max tk
	max_tk = traj[0].Tk;
	for(int i=1;i<4;i++){
		if(traj[i].Tk > max_tk){
			max_tk = traj[i].Tk;
		}
	}
	traj[0].Tk = max_tk;
	traj[1].Tk = max_tk;
	traj[2].Tk = max_tk;
	traj[3].Tk = max_tk;
}
void trajectory_generatetraj(Trajectory *traj,float q_i,float q_f){
	float T = 5.0;
	float T_pow2;
	float T_pow3;
	float T_pow4;
	float T_pow5;
	float dq_i = 0;
	float dq_f = 0;
	float ddq_i = 0;
	float ddq_f = 0;
	int flag = 5;
	int joint = 0;
	float maxspeedofjoint[4] = {300,1000,2500,300};
	while(flag){
		T_pow2 = T * T;
		T_pow3 = T * T * T;
		T_pow4 = T * T * T * T;
		T_pow5 = T * T * T * T * T;
		traj->c0 = q_i;
		traj->c1 = dq_i;
		traj->c2 = ddq_i / 2.0;
		traj->c3 = -(20 * q_i -20 * q_f + 8 * T* dq_f +12 * T * dq_i - (ddq_f * T_pow2)+(3 * ddq_i * T_pow2))/(2 * T_pow3);
		traj->c4 = (30 * q_i - 30 * q_f + 14 * T * dq_f+ 16 * T * dq_i-(2 * ddq_f * T_pow2) + (3 * ddq_i * T_pow2))/(2 * T_pow4);
		traj->c5 = -(12 * q_i- 12 * q_f+ 6 * T * dq_f + 6 * T * dq_i-(ddq_f * T_pow2)+(ddq_i * T_pow2))/(2 * T_pow5);
		traj->Vmax = (((((60.0 * q_f - 60.0 * q_i) - (14.0 * T * dq_f)) - 14.0 * T * dq_i) + ddq_f * traj->c5) - ddq_i * traj->c5) / (32.0 * T);
		if(traj->Vmax < maxspeedofjoint[joint]){
			traj->Tk = T;
			flag=0;
		} else{
			T *= 2;
		}
		joint = (joint + 1) % 4;
	}
	traj->t = 0;
}

void mhainw_trajectory_updatetraj(Trajectory *traj)
{
	float c0 = traj->c0;
	float c1 = traj->c1;
	float c2 = traj->c2;
	float c3 = traj->c3;
	float c4 = traj->c4;
	float c5 = traj->c5;
	float t = traj->t;
	float t_pow2 = t * t;
	float t_pow3 = t * t * t;
	float t_pow4 = t * t * t * t;
	float t_pow5 = t * t * t * t * t;
	traj->q = c0 + c1*t + c2*t_pow2 + c3*t_pow3 + c4*t_pow4 + c5*t_pow5;
	traj->dq = c1 + 2*c2*t + 3*c3*t_pow2 + 4*c4*t_pow3 + 5*c5*t_pow4;
//	traj->ddq = 2*c2 + 6*c3*t + 12*c4*t_pow2 + 20*c5*t_pow3;
	traj->t += traj->delta_t;
}

