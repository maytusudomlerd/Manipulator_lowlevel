/*
 * mhainw_steppermotor.c
 *
 *  Created on: Feb 4, 2022
 *      Author: maytus
 */

#include"mhainw_steppermotor.h"
#include "tim.h"


float jointworkspace[4] = {J1_WORKSPACE,J2_WORKSPACE,J3_WORKSPACE,J4_WORKSPACE};
float motor_max_frequency[4] = {JOINT1_MAX_FREQUENCY,JOINT2_MAX_FREQUENCY,JOINT3_MAX_FREQUENCY,JOINT4_MAX_FREQUENCY};
float motor_min_frequency[4] = {JOINT1_MIN_FREQUENCY,JOINT2_MIN_FREQUENCY,JOINT3_MIN_FREQUENCY,JOINT4_MIN_FREQUENCY};


void mhainw_stepper_init(Stepper_motor *motor,TIM_HandleTypeDef *timHandle,uint32_t tim_ch,
		GPIO_TypeDef *dir_port,uint16_t dir_pin){

	motor->timHandle = timHandle;
	motor->tim_ch = tim_ch;
	motor->dir_port = dir_port;
	motor->dir_pin = dir_pin;
	motor->freq = 1;

	HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, CCW);

	HAL_TIM_PWM_Start(motor->timHandle, motor->tim_ch);
	mhainw_stepper_setspeed(motor, 0);
}

void mhainw_stepper_setspeed_map(Stepper_motor *motor,float freq){

	static uint8_t joint = 0;
	//map PID value to motor frequency rage
	float freq_mapped = map(freq, 0,jointworkspace[joint], 0, motor_max_frequency[joint]);
	joint = (joint + 1) % 4;

	if(freq > MIN_FREQUENCY){
		HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, CCW);
		mhainw_stepper_setpwm(motor,freq_mapped,0.5);
	} else if(freq < (-1 * MIN_FREQUENCY)) {
		HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, CW);
		mhainw_stepper_setpwm(motor, -1 * freq_mapped,0.5);
	} else {
		mhainw_stepper_setpwm(motor, 100, 0);
	}
	motor->freq = freq_mapped;
}

void mhainw_stepper_setspeed(Stepper_motor *motor,float freq){

	if(freq > MIN_FREQUENCY){
		HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, CCW);
		mhainw_stepper_setpwm(motor,freq,0.5);
	} else if(freq < (-1 * MIN_FREQUENCY)) {
		HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, CW);
		mhainw_stepper_setpwm(motor, -1 * freq,0.5);
	} else {
		mhainw_stepper_setpwm(motor, 100, 0);
	}
	motor->freq = freq;
}

void mhainw_stepper_setpwm(Stepper_motor *motor,float freq,float duty_cycle){

//	if(freq > MAX_FREQUENCY) {
//		freq = MAX_FREQUENCY;
//	}
//
	if(motor->timHandle == &htim13){
		if(freq > JOINT1_MAX_FREQUENCY){
			freq = JOINT1_MAX_FREQUENCY;
		}
	}
	else if(motor->timHandle == &htim16){
		if(freq > JOINT2_MAX_FREQUENCY){
			freq = JOINT2_MAX_FREQUENCY;
		}
	}
	else if(motor->timHandle == &htim14){
		if(freq > JOINT3_MAX_FREQUENCY){
			freq = JOINT3_MAX_FREQUENCY;
		}
	}
	else if(motor->timHandle == &htim17){
		if(freq > JOINT4_MAX_FREQUENCY){
			freq = JOINT4_MAX_FREQUENCY;
		}
	}

	uint16_t period = 1e6 / freq;
	uint16_t pwm = period * duty_cycle;

	__HAL_TIM_SET_AUTORELOAD(motor->timHandle, period);
	__HAL_TIM_SET_COMPARE(motor->timHandle, motor->tim_ch, pwm);

}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

