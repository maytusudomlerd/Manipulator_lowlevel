/*
 * mhainw_trajectory.c
 *
 *  Created on: Feb 28, 2022
 *      Author: K. Peerasate & U. Maytus
 */

#include "mhainw_trajectory.h"
#include "stdlib.h"



void mhainw_trajectory_init(Trajectory *traj,float delta_t){
	traj->delta_t = delta_t;
	traj->setpoint = 0;
	traj->havetraj =0;
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

#ifdef nothave_dq_i
void mhainw_trajectory_generatetraj(Trajectory *traj,float *q_i,float *q_f){

	for(int i=0;i<4;i++){
		trajectory_findTk(&traj[i],q_i[i],q_f[i]);
	}

	traj[0].Tk = traj[3].Tk;
	traj[1].Tk = traj[3].Tk;
	traj[2].Tk = traj[3].Tk;

	for(int i=0;i<4;i++){
		trajectory_generateTrajCoef(&traj[i],q_i[i],q_f[i]);
	}
}
void trajectory_generateTrajCoef(Trajectory *traj,float q_i,float q_f){
	float T = traj->Tk;
	float T_pow2 = T * T;
	float T_pow3 = T * T * T;
	float T_pow4 = T * T * T * T;
	float T_pow5 = T * T * T * T * T;
	float dq_i = 0;
	float dq_f = 0;
	float ddq_i = 0;
	float ddq_f = 0;
	traj->c0 = q_i;
	traj->c1 = dq_i;
	traj->c2 = ddq_i / 2.0;
	traj->c3 = -(20 * q_i -20 * q_f + 8 * T * dq_f +12 * T * dq_i - (ddq_f * T_pow2)+(3 * ddq_i * T_pow2))/(2 * T_pow3);
	traj->c4 = (30 * q_i - 30 * q_f + 14 * T * dq_f+ 16 * T * dq_i-(2 * ddq_f * T_pow2) + (3 * ddq_i * T_pow2))/(2 * T_pow4);
	traj->c5 = -(12 * q_i- 12 * q_f+ 6 * T * dq_f + 6 * T * dq_i-(ddq_f * T_pow2)+(ddq_i * T_pow2))/(2 * T_pow5);
	traj->t = 0;
	traj->havetraj =1;
}
void trajectory_findTk(Trajectory *traj,float q_i,float q_f){
	static float T = 4;
	float T_pow2;
	float dq_i = 0;
	float dq_f = 0;
	float ddq_i = 0;
	float ddq_f = 0;
	int flag = 1;
	static int joint = 0;
	float maxspeedofjoint[4] = {1.018,1.333,60.0,1.133};

	while(flag){
		T_pow2 = T * T;
		traj->Vmax = (((((60.0 * q_f - 60.0 * q_i) - (14.0 * T * dq_f)) - 14.0 * T * dq_i) + (ddq_f * T_pow2)) - (ddq_i * T_pow2)) / (32.0 * T);

		if(traj->Vmax < 0){
			if(-1*(traj->Vmax) < maxspeedofjoint[joint]){
				traj->Tk = T;
				flag=0;
			} else{
				T *= 2;
			}
		} else if(traj->Vmax > 0){
			if(traj->Vmax < maxspeedofjoint[joint]){
				traj->Tk = T;
				flag=0;
			} else{
				T *= 2;
			}
		} else{
			flag = 0;
		}
	}
	//reset T for next trajectory
	if(joint == 3){
		T = 4;
	}
	//update joint index
	joint = (joint + 1) % 4;

}
#endif

#ifdef have_dq_i
void mhainw_trajectory_generatetraj(Trajectory *traj,float *q_i,float *q_f,Kalmanfilter *kalman){

	for(int i=0;i<4;i++){
		trajectory_findTk(&traj[i],q_i[i],q_f[i],kalman[i].x2);
	}

	traj[0].Tk = traj[3].Tk;
	traj[1].Tk = traj[3].Tk;
	traj[2].Tk = traj[3].Tk;

	for(int i=0;i<4;i++){
		trajectory_generateTrajCoef(&traj[i],q_i[i],q_f[i],kalman[i].x2);
	}
}
void trajectory_generateTrajCoef(Trajectory *traj,float q_i,float q_f,float dq_i){
	float T = traj->Tk;
	float T_pow2 = T * T;
	float T_pow3 = T * T * T;
	float T_pow4 = T * T * T * T;
	float T_pow5 = T * T * T * T * T;
//	float dq_i = 0;
	float dq_f = 0;
	float ddq_i = 0;
	float ddq_f = 0;
	traj->c0 = q_i;
	traj->c1 = dq_i;
	traj->c2 = ddq_i / 2.0;
	traj->c3 = -(20 * q_i -20 * q_f + 8 * T * dq_f +12 * T * dq_i - (ddq_f * T_pow2)+(3 * ddq_i * T_pow2))/(2 * T_pow3);
	traj->c4 = (30 * q_i - 30 * q_f + 14 * T * dq_f+ 16 * T * dq_i-(2 * ddq_f * T_pow2) + (3 * ddq_i * T_pow2))/(2 * T_pow4);
	traj->c5 = -(12 * q_i- 12 * q_f+ 6 * T * dq_f + 6 * T * dq_i-(ddq_f * T_pow2)+(ddq_i * T_pow2))/(2 * T_pow5);
	traj->t = 0;
	traj->havetraj =1;
}
void trajectory_findTk(Trajectory *traj,float q_i,float q_f,float dq_i){
	static float T = 2.0;
	float T_pow2;
//	float dq_i = 0;
	float dq_f = 0;
	float ddq_i = 0;
	float ddq_f = 0;
	int flag = 1;
	static int joint = 0;
//	float maxspeedofjoint[4] = {0.518,0.733,30.0,0.933};
	float maxspeedofjoint[4] = {1.018,1.333,30.0,1.133};

	while(flag){
		T_pow2 = T * T;
		traj->Vmax = (((((60.0 * q_f - 60.0 * q_i) - (14.0 * T * dq_f)) - 14.0 * T * dq_i) + (ddq_f * T_pow2)) - (ddq_i * T_pow2)) / (32.0 * T);

		if(traj->Vmax < 0){
			if(-1*(traj->Vmax) < maxspeedofjoint[joint]){
				traj->Tk = T;
				flag=0;
			} else{
				T *= 2;
			}
		} else if(traj->Vmax > 0){
			if(traj->Vmax < maxspeedofjoint[joint]){
				traj->Tk = T;
				flag=0;
			} else{
				T *= 2;
			}
		} else{
			flag = 0;
		}
	}
	//reset T for next trajectory
	if(joint == 3){
		T = 2.0;
	}
	//update joint index
	joint = (joint + 1) % 4;

}
#endif


