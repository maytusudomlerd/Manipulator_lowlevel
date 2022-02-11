/*
 * mhainw_steppermotor.c
 *
 *  Created on: Feb 4, 2022
 *      Author: maytus
 */

#include"mhainw_steppermotor.h"


void mhainw_stepper_init(Stepper_motor *motor,TIM_HandleTypeDef *timHandle,uint32_t tim_ch,
		GPIO_TypeDef *dir_port,uint16_t dir_pin){

	motor->timHandle = timHandle;
	motor->tim_ch = tim_ch;
	motor->dir_port = dir_port;
	motor->dir_pin = dir_pin;
	motor->freq = 1;

	HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, 0);

	HAL_TIM_PWM_Start(motor->timHandle, motor->tim_ch);
	mhainw_stepper_setspeed(motor,0);
}

void mhainw_stepper_setspeed(Stepper_motor *motor,float freq){
	if(motor->freq != freq){
		if(freq > 0){
			HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, 1);
			mhainw_stepper_setpwm(motor,freq,0.5);
		} else if(freq < 0) {
			HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, 0);
			mhainw_stepper_setpwm(motor, -1 * freq,0.5);
		} else {
			mhainw_stepper_setpwm(motor, 100, 0);
		}
		motor->freq = freq;
	}
}

void mhainw_stepper_setpwm(Stepper_motor *motor,float freq,float duty_cycle){
	if(freq < 50){
		freq = 50;
	} else if(freq > 1000) {
		freq = 1000;
	}

	uint16_t period = 1e6 / freq;
	uint16_t pwm = period * duty_cycle;

	__HAL_TIM_SET_AUTORELOAD(motor->timHandle, period);
	__HAL_TIM_SET_COMPARE(motor->timHandle, motor->tim_ch, pwm);

}
