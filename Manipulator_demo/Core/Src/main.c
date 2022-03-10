/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "mhainw_steppermotor.h"
#include "mhainw_amt10.h"
#include "mhainw_protocol.h"
#include "mhainw_control.h"
#include "mhainw_kinematics.h"
#include "mhainw_kalmanfilter.h"
#include "mhainw_trajectory.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415926
#define kalman_Q 5000
#define kalman_R 0.01

#define J1_ENCODERRESOLUTION 8192
#define J2_ENCODERRESOLUTION 112721.92
#define J3_ENCODERRESOLUTION 8192
#define J4_ENCODERRESOLUTION 8192

#define J1_OFFSETTOHOMECONFIGURATION 1718
#define J2_OFFSETTOHOMECONFIGURATION -48500
#define J3_OFFSETTOHOMECONFIGURATION 0
#define J4_OFFSETTOHOMECONFIGURATION 6548

#define J1_POSITIVE_JOINTLIMIT 105 * DEGTORAD
#define J1_NEGATIVE_JOINTLIMIT -1 * J1_POSITIVE_JOINTLIMIT
#define J2_POSITIVE_JOINTLIMIT 160 * DEGTORAD
#define J2_NEGATIVE_JOINTLIMIT -1 * J2_POSITIVE_JOINTLIMIT
#define J3_UP_JOINTLIMIT 0
#define J3_DOWN_JOINTLIMIT -135.0
#define J4_POSITIVE_JOINTLIMIT 105 * DEGTORAD
#define J4_NEGATIVE_JOINTLIMIT -1 * J4_POSITIVE_JOINTLIMIT


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Protocol user;

Stepper_motor motors[4];

Encoder encoders[4];
Encoder chessboardenc;
int32_t chessboardpos;

float target_position[4] = { 0 };
float jointstate[4]; //real time value from encoder
float setpoint[4];   //target point in encoder unit(pulse)
float taskconfig[4] = {0};  //now position in task space
float jointconfig[4] = {0}; //now position in configuration space
float tasksetpoint[4]; //target point in task space
float jointsetpoint[4]; //target point in configuration space
float jointvelocitysetpoint[4]; //target velocity in configuration space

float temp[4];
int16_t goal;

float radtopulse[4] = {JOINT1_RADTOPULSE,JOINT2_RADTOPULSE,JOINT3_MMTOPULSE,JOINT4_RADTOPULSE};
float positive_jointlimit[4] = {J1_POSITIVE_JOINTLIMIT, J2_POSITIVE_JOINTLIMIT, J3_UP_JOINTLIMIT, J4_POSITIVE_JOINTLIMIT};
float negative_jointlimit[4] = {J1_NEGATIVE_JOINTLIMIT, J2_NEGATIVE_JOINTLIMIT, J3_DOWN_JOINTLIMIT, J4_NEGATIVE_JOINTLIMIT};
uint8_t joint = 0;

uint32_t timestamp = 0;
Controller position_jointcontroller[4];
Controller velocity_jointcontroller[4];
Kalmanfilter kalmanjoint[4];
Trajectory quinticTrajectory[4];

uint8_t control_flag = 1;

uint8_t Proximity_state[4];

uint8_t setpoint_test_indx[4] = {0};
float setpoint_test[2][4] = {{J1_POSITIVE_JOINTLIMIT/2, J2_POSITIVE_JOINTLIMIT/2, J3_UP_JOINTLIMIT, J4_POSITIVE_JOINTLIMIT},
		                     {J1_NEGATIVE_JOINTLIMIT/2, J2_NEGATIVE_JOINTLIMIT/2, -100.0, J4_NEGATIVE_JOINTLIMIT}};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void mhainw_robot_sethome();
void mhainw_command_statemachine(Protocol *uart);
void jogcatesian(Protocol *uart,float *jointsetpoint);
void jogjoint(Protocol *uart,float *jointsetpoint);
void movejoint(Protocol *uart,float *jointsetpoint);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_UART7_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  mhainw_protocol_init(&user, &huart3);

  mhainw_stepper_init(&motors[0], &htim13, TIM_CHANNEL_1 , dir1_GPIO_Port, dir1_Pin);
  mhainw_stepper_init(&motors[1], &htim16, TIM_CHANNEL_1 , dir3_GPIO_Port, dir3_Pin);
  mhainw_stepper_init(&motors[2], &htim14, TIM_CHANNEL_1 , dir2_GPIO_Port, dir2_Pin);
  mhainw_stepper_init(&motors[3], &htim17, TIM_CHANNEL_1 , dir4_GPIO_Port, dir4_Pin);

  mhainw_amt10_init(&encoders[0], &htim5);
  mhainw_amt10_init(&encoders[1], &htim2);
  mhainw_amt10_init(&encoders[2], &htim3);
  mhainw_amt10_init(&encoders[3], &htim4);
  mhainw_amt10_init(&chessboardenc, &htim8);

  mhainw_control_init(&position_jointcontroller[0],500,0.05,0);
  mhainw_control_init(&position_jointcontroller[1],2000,0.5,0);
  mhainw_control_init(&position_jointcontroller[2],100,0.9,0);
  mhainw_control_init(&position_jointcontroller[3],500,0.5,0);

  mhainw_control_init(&velocity_jointcontroller[0],3,0,0);
  mhainw_control_init(&velocity_jointcontroller[1],3,0,0);
  mhainw_control_init(&velocity_jointcontroller[2],3,0,0);
  mhainw_control_init(&velocity_jointcontroller[3],3,0,0);

  mhainw_kalmanfilter_init(&kalmanjoint[0],0,0,0,0,0,0,kalman_Q,kalman_R);
  mhainw_kalmanfilter_init(&kalmanjoint[1],0,0,0,0,0,0,5000,0.01);
  mhainw_kalmanfilter_init(&kalmanjoint[2],0,0,0,0,0,0,kalman_Q,kalman_R);
  mhainw_kalmanfilter_init(&kalmanjoint[3],0,0,0,0,0,0,kalman_Q,kalman_R);

  mhainw_trajectory_init(&quinticTrajectory[0],0.001);
  mhainw_trajectory_init(&quinticTrajectory[1],0.001);
  mhainw_trajectory_init(&quinticTrajectory[2],0.001);
  mhainw_trajectory_init(&quinticTrajectory[3],0.001);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  mhainw_robot_sethome();
/*
  //testtraj

	jointsetpoint[0] = setpoint_test[0][0];
	jointsetpoint[1] = setpoint_test[0][1];
	jointsetpoint[2] = setpoint_test[0][2];
	jointsetpoint[3] = setpoint_test[0][3];
	mhainw_trajectory_generatetraj(&quinticTrajectory[0],5,jointconfig[0],jointsetpoint[0],0,0,0,0);
	quinticTrajectory[0].initial_time = HAL_GetTick();
	mhainw_trajectory_generatetraj(&quinticTrajectory[1],5,jointconfig[1],jointsetpoint[1],0,0,0,0);
	quinticTrajectory[1].initial_time = HAL_GetTick();
	mhainw_trajectory_generatetraj(&quinticTrajectory[2],5,jointconfig[2],jointsetpoint[2],0,0,0,0);
	quinticTrajectory[2].initial_time = HAL_GetTick();
	mhainw_trajectory_generatetraj(&quinticTrajectory[3],5,jointconfig[3],jointsetpoint[3],0,0,0,0);
	quinticTrajectory[3].initial_time = HAL_GetTick();
	quinticTrajectory[0].havetraj = 1;
	quinticTrajectory[1].havetraj = 1;
	quinticTrajectory[2].havetraj = 1;
	quinticTrajectory[3].havetraj = 1;
	//end test traj
*/

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(control_flag ==1 ){
		  if(HAL_GetTick() - timestamp >= 1){
			timestamp = HAL_GetTick();

			for(joint = 0;joint<4;joint++){
				//read encoder
				jointstate[joint] += mhainw_amt10_unwrap(&encoders[joint]);
				//convert pulse to rad
				jointconfig[joint] =  jointstate[joint] / radtopulse[joint];
				mhainw_kalmanfilter_updatekalman(&kalmanjoint[joint],jointconfig[joint]);

				if(quinticTrajectory[joint].havetraj == 1){
					quinticTrajectory[joint].t = (HAL_GetTick() - quinticTrajectory[joint].initial_time) / 1000;
					if(quinticTrajectory[joint].t <= quinticTrajectory[joint].Tk){
						mhainw_trajectory_updatetraj(&quinticTrajectory[joint]); // q da ddq viapoint
						jointsetpoint[joint] = quinticTrajectory[joint].q;
						jointvelocitysetpoint[joint] = quinticTrajectory[joint].dq;
					}
					else{
						quinticTrajectory[joint].havetraj = 0;
						user.goal_reach[joint] = 1;
/*
						//terst traj
						setpoint_test_indx[joint] = (setpoint_test_indx[joint] + 1) % 2;
						jointsetpoint[joint] = setpoint_test[setpoint_test_indx[joint]][joint];
						mhainw_trajectory_generatetraj(&quinticTrajectory[joint],5,jointconfig[joint],jointsetpoint[joint],0,0,0,0);
						quinticTrajectory[joint].initial_time = HAL_GetTick();
						quinticTrajectory[joint].havetraj = 1;
						//end test traj
*/
					}
				}
				//joint limit
				if(jointsetpoint[joint] >= positive_jointlimit[joint]){
							  jointsetpoint[joint] = positive_jointlimit[joint];
				}
			    if(jointsetpoint[joint] <= negative_jointlimit[joint]){
				  jointsetpoint[joint] = negative_jointlimit[joint];
			    }
				//update position control
				mhainw_control_controllerupdate(&position_jointcontroller[joint], jointsetpoint[joint], jointconfig[joint]);
				//update velocity control
				mhainw_control_controllerupdate(&velocity_jointcontroller[joint],
						position_jointcontroller[joint].output + jointvelocitysetpoint[joint] , kalmanjoint[joint].x2);

				mhainw_stepper_setspeed(&motors[joint], velocity_jointcontroller[joint].output);
			}
		  }
	  }
	  if(user.package_verify){
		  mhainw_command_statemachine(&user);
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void mhainw_robot_sethome(){
	uint8_t sethome[3] = {0};
	float jointoffset[4] = {J1_OFFSETTOHOMECONFIGURATION,J2_OFFSETTOHOMECONFIGURATION,J3_OFFSETTOHOMECONFIGURATION,J4_OFFSETTOHOMECONFIGURATION};

	//offset
	mhainw_stepper_setspeed(&motors[0], -100);
	mhainw_stepper_setspeed(&motors[1], 300);
	mhainw_stepper_setspeed(&motors[2], -1000);
	mhainw_stepper_setspeed(&motors[3], -1000);
	HAL_Delay(500);
	mhainw_stepper_setspeed(&motors[0], 0);
	mhainw_stepper_setspeed(&motors[1], 0);
	mhainw_stepper_setspeed(&motors[2], 0);
	mhainw_stepper_setspeed(&motors[3], 0);
	HAL_Delay(500);

	//state 1 set joint 4
	while(Proximity_state[3] != 1){
		mhainw_stepper_setspeed(&motors[3], 300);
	}
	mhainw_stepper_setspeed(&motors[3], 0);

	//state 2 : move to home position
	while(sethome[0] != 1 || sethome[1] !=1 || sethome[2] != 1){

		if(Proximity_state[0] == 1){
			mhainw_stepper_setspeed(&motors[0], 0);
			sethome[0] = 1;
		} else{
			mhainw_stepper_setspeed(&motors[0], 150);
		}
		if(Proximity_state[1] == 1){
			mhainw_stepper_setspeed(&motors[1], 0);
			sethome[1] = 1;
		} else {
			mhainw_stepper_setspeed(&motors[1], -500);
		}
		if(Proximity_state[2] == 1){
			mhainw_stepper_setspeed(&motors[2], 0);
			sethome[2] = 1;
		} else{
			mhainw_stepper_setspeed(&motors[2], 1000);
		}
	}
	HAL_Delay(1000);
	//state 3 : set current position and offset home configuration

	htim5.Instance->CNT = 0;
	htim2.Instance->CNT = 0;
	htim3.Instance->CNT = 0;
	htim4.Instance->CNT = 0;

	for(int i =0;i<4;i++){
		jointstate[i] = jointoffset[i];
		jointconfig[i] = jointstate[i] / radtopulse[i];
		jointsetpoint[i] = jointconfig[i];
	}
}

void mhainw_command_statemachine(Protocol *uart){
	static uint8_t set = 0;
	switch(uart->inst){
		case MHAINW_SETHOME:
			UARTsentACK(uart, MHAINW_SETHOME_ACK);
			break;
		case MHAINW_JOG_CATESIAN:
			if(set == 0){
				UARTsentACK(uart, MHAINW_JOG_ACK);
				jogcatesian(uart,jointsetpoint);
				for(int i=0;i<4;i++){
					mhainw_trajectory_generatetraj(&quinticTrajectory[i],0.3,jointconfig[i],jointsetpoint[i],0,0,0,0);
					quinticTrajectory[i].initial_time = HAL_GetTick();
					//cal Tk
					quinticTrajectory[i].havetraj = 1;
				}
				set = 1;
			}
			if(uart->goal_reach[0] == 1 && uart->goal_reach[1] == 1 && uart->goal_reach[2] == 1 && uart->goal_reach[3] == 1){
				UARTsentACK(uart, MHAINW_JOG_ACK);
				uart->package_verify = 0;
				set = 0;
			}
			break;
		case MHAINW_JOG_JOINT:
			if(set == 0){
				UARTsentACK(uart, MHAINW_JOG_ACK);
				jogjoint(uart,jointsetpoint);
				for(int i=0;i<4;i++){
					mhainw_trajectory_generatetraj(&quinticTrajectory[i],0.5,jointconfig[i],jointsetpoint[i],0,0,0,0);
					quinticTrajectory[i].initial_time = HAL_GetTick();
					quinticTrajectory[i].havetraj = 1;
				}
				set = 1;
			}
			if(uart->goal_reach[0] == 1 && uart->goal_reach[1] == 1 && uart->goal_reach[2] == 1 && uart->goal_reach[3] == 1){
				UARTsentACK(uart, MHAINW_JOG_ACK);
				uart->package_verify = 0;
				set = 0;
			}
			break;
		case MAHINW_MOVE_CATESIAN :
			break;
		case MHAINW_MOVE_JOINT:
			if(set == 0){
				UARTsentACK(uart, MHAINW_MOVE_ACK);
				movejoint(uart,jointsetpoint);
				for(int i=0;i<4;i++){
					mhainw_trajectory_generatetraj(&quinticTrajectory[i],7,jointconfig[i],jointsetpoint[i],0,0,0,0);
					quinticTrajectory[i].initial_time = HAL_GetTick();
					quinticTrajectory[i].havetraj = 1;
				}
				set = 1;
			}
			if(uart->goal_reach[0] == 1 && uart->goal_reach[1] == 1 && uart->goal_reach[2] == 1 && uart->goal_reach[3] == 1){
				UARTsentACK(uart, MHAINW_MOVE_ACK);
				uart->package_verify = 0;
				set = 0;
			}
			uart->package_verify = 0;
			break;
		case MHAINW_MOVE_TASK:
			UARTsentACK(uart, MHAINW_TASKMOVE_ACK);
			break;
	  }
}

void jogcatesian(Protocol *uart,float *jointsetpoint){
	uint8_t axis = uart->data[0];
	float step = uart->data[1];
	float movingstep[4] = {0}; //{ Rz X Y Z}
	float dq[4] = {0};

	if(axis == 8){ //x
		movingstep[1] = step;
	} else if(axis == 4){ //y
		movingstep[2] = step;
	} else if(axis == 2){ //z
		movingstep[3] = step;
	} else if(axis == 1){  //rz
		movingstep[0] = step * DEGTORAD;
	}

	IVK(jointsetpoint,movingstep,dq);

	for(int i=0;i<4;i++){
		jointsetpoint[i] += dq[i];
	}
}

void jogjoint(Protocol *uart,float *jointsetpoint){
	uint8_t axis = uart->data[0];
	float step = (float)(uart->data[1]) * DEGTORAD;

	if(axis == 8){
		jointsetpoint[0] += step;
	} else if(axis == 4){
		jointsetpoint[1] += step;
	} else if(axis == 2){
		jointsetpoint[2] += -1 * uart->data[1];
	} else if(axis == 1){
		jointsetpoint[3] += step;
	}

}

void movejoint(Protocol *uart,float *jointsetpoint){
	if(uart->data[0] == 1){
		jointsetpoint[0] += (int16_t)((uart->data[1] << 8) | (uint8_t)uart->data[2]) * DEGTORAD;
		jointsetpoint[1] += (int16_t)((uart->data[3] << 8) | (uint8_t)uart->data[4]) * DEGTORAD;
		jointsetpoint[2] += -1.0 * (int16_t)((uart->data[5] << 8) | (uint8_t)uart->data[6]);
		jointsetpoint[3] += (int16_t)((uart->data[7] << 8) | (uint8_t)uart->data[8]) * DEGTORAD;

	} else{
		jointsetpoint[0] = (int16_t)((uart->data[1] << 8) | (uint8_t)uart->data[2]) * DEGTORAD;
		jointsetpoint[1] = (int16_t)((uart->data[3] << 8) | (uint8_t)uart->data[4]) * DEGTORAD;
		jointsetpoint[2] = -1.0 * (int16_t)((uart->data[5] << 8) | (uint8_t)uart->data[6]);
		jointsetpoint[3] = (int16_t)((uart->data[7] << 8) | (uint8_t)uart->data[8]) * DEGTORAD;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
