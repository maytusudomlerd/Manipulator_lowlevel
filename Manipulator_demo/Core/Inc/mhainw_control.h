/*
 * mhainw_control.h
 *
 *  Created on: Feb 9, 2022
 *      Author: maytus
 */

#ifndef INC_MHAINW_CONTROL_H_
#define INC_MHAINW_CONTROL_H_

typedef struct{
	//control gain
	float kp, ki, kd;

	float integrator;
	float perv_error;

	float p_term, i_term, d_term;
	float output;
}Controller;

void mhainw_control_init(Controller *pid,float kp, float ki, float kd);
void mhainw_control_positioncontrol(Controller *pid, float setpoint, float jointstate);




#endif /* INC_MHAINW_CONTROL_H_ */
