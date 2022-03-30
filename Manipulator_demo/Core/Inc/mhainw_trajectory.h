/*
 * mhainw_trajectory.h
 *
 *  Created on: Mar 1, 2022
 *      Author: pienapat
 */

#ifndef INC_MHAINW_TRAJECTORY_H_
#define INC_MHAINW_TRAJECTORY_H_

#include "math.h"

typedef struct{
	float havetraj;
	float initial_time;
	float c0,c1,c2,c3,c4,c5;
	float Tk;
	float t;
	float delta_t;
	float q;
	float dq;
	float ddq;
	float Vmax;
	float setpoint;
}Trajectory;

void mhainw_trajectory_init(Trajectory *traj,float delta_t);
void mhainw_trajectory_generatetraj(Trajectory *traj,float T,float q_i,float q_f,float dq_i,float dq_f,float ddq_i,float ddq_f);
void mhainw_trajectory_updatetraj(Trajectory *traj);
#endif /* INC_MHAINW_TRAJECTORY_H_ */
