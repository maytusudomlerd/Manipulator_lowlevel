/*
 * mhainw_control.c
 *
 *  Created on: Feb 9, 2022
 *      Author: maytus
 */

#include "mhainw_control.h"

void mhainw_control_init(Controller *pid,float kp, float ki, float kd,float limit_i,float limit_out){
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->limit_i = limit_i;
	pid->limit_out = limit_out;

	pid->integrator = 0;
	pid->perv_error =0;
	pid->p_term = 0 ;
	pid->d_term = 0;
	pid->i_term = 0;

	pid->output = 0.0;
}

void mhainw_control_setcontrollimit(Controller *pid,float limit_i,float limit_out){
	pid->limit_i = limit_i;
	pid->limit_out = limit_out;
}

void mhainw_control_controllerupdate(Controller *pid, float setpoint, float jointstate){
	float error = setpoint - jointstate;
	//update proportional term
	pid->p_term = pid->kp * error;

	pid->integrator += error;
	//update integral term
	pid->i_term = pid->ki * pid->integrator;
	if(pid->i_term > pid->limit_i){
		pid->i_term = pid->limit_i;
	}

	pid->d_term = pid->kd * (error - pid->perv_error);
	pid->perv_error = error;

	if(pid->output > pid->limit_out){
		pid->output = pid->limit_out;
	} else if( (-1 * pid->output) > pid->limit_out){
		pid->output = -1*pid->limit_out;
	}
	else{
		pid->output = pid->p_term + pid->i_term + pid->d_term;
	}
	pid->output = pid->p_term + pid->i_term + pid->d_term;


}





