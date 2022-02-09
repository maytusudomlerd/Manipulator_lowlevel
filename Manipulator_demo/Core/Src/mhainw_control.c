/*
 * mhainw_control.c
 *
 *  Created on: Feb 9, 2022
 *      Author: maytus
 */

#include "mhainw_control.h"

void mhainw_control_init(Controller *pid,float kp, float ki, float kd){
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->output = 0.0;
}

void mhainw_control_positioncontrol(Controller *pid, float setpoint, float jointstate){
	float error = setpoint - jointstate;
	//update proportional term
	pid->p_term = pid->kp * error;

	pid->integrator += error;
	//update integral term
	pid->i_term = pid->ki * pid->integrator;

	pid->d_term = pid->kd * (error - pid->perv_error);
	pid->perv_error = error;

	pid->output = pid->p_term + pid->i_term + pid->d_term;
//	return pid->output;
}





