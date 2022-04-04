/*
 * mhainw_steppermotor.h
 *
 *  Created on: Feb 4, 2022
 *      Author: maytus
 */

#ifndef INC_MHAINW_STEPPERMOTOR_H_
#define INC_MHAINW_STEPPERMOTOR_H_

#include "stm32h7xx_hal.h"

//motor workspace in pulse
#define J1_WORKSPACE 3436
#define J2_WORKSPACE 97000
#define J3_WORKSPACE 42000
#define J4_WORKSPACE 13096

#define JOINT1_MAX_FREQUENCY 300.0
#define JOINT2_MAX_FREQUENCY 1000.0
#define JOINT3_MAX_FREQUENCY 2500.0
#define JOINT4_MAX_FREQUENCY 300.0
#define JOINT1_MIN_FREQUENCY -300.0
#define JOINT2_MIN_FREQUENCY -1000.0
#define JOINT3_MIN_FREQUENCY -2500.0
#define JOINT4_MIN_FREQUENCY -300.0

#define MIN_FREQUENCY 16

#define CCW 0
#define CW 1

typedef struct{
	TIM_HandleTypeDef *timHandle;
	uint32_t tim_ch;
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;
	float freq;
}Stepper_motor;

void mhainw_stepper_init(Stepper_motor *motor,TIM_HandleTypeDef *timHandle,uint32_t tim_ch,
		GPIO_TypeDef *dir_port,uint16_t dir_pin);
void mhainw_stepper_setspeed_map(Stepper_motor *motor,float freq);
void mhainw_stepper_setspeed(Stepper_motor *motor,float freq);
void mhainw_stepper_setpwm(Stepper_motor *motor,float freq,float duty_cycle);

float map(float x, float in_min, float in_max, float out_min, float out_max);



#endif /* INC_MHAINW_STEPPERMOTOR_H_ */
