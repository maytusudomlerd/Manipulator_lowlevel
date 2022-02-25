/*
 * mhainw_steppermotor.h
 *
 *  Created on: Feb 4, 2022
 *      Author: maytus
 */

#ifndef INC_MHAINW_STEPPERMOTOR_H_
#define INC_MHAINW_STEPPERMOTOR_H_

#include "stm32h7xx_hal.h"

#define JOINT1_MAX_FREQUENCY 500.0
#define JOINT2_MAX_FREQUENCY 700.0
#define JOINT3_MAX_FREQUENCY 2500.0
#define JOINT4_MAX_FREQUENCY 300.0
#define ALLJOINT_MIN_FREQUENCY 0.0

#define MIN_FREQUENCY 15.0

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
void mhainw_stepper_setspeed(Stepper_motor *motor,float freq);
void mhainw_stepper_setpwm(Stepper_motor *motor,float freq,float duty_cycle);


#endif /* INC_MHAINW_STEPPERMOTOR_H_ */
