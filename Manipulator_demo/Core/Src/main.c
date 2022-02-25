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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415926
#define kalman_Q 1
#define kalman_R 1

#define J1_ENCODERRESOLUTION 8192
#define J2_ENCODERRESOLUTION 112721.92
#define J3_ENCODERRESOLUTION 8192
#define J4_ENCODERRESOLUTION 8192

#define J1_OFFSETTOHOMECONFIGURATION 1718
#define J2_OFFSETTOHOMECONFIGURATION -48500
#define J3_OFFSETTOHOMECONFIGURATION 0
#define J4_OFFSETTOHOMECONFIGURATION 6548

#define J1_WORKSPACE 3436
#define J2_WORKSPACE 97000
#define J3_WORKSPACE 42000
#define J4_WORKSPACE 13096

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
double jointstate[4]; //real time value from encoder
int32_t setpoint[4];   //target point in encoder unit(pulse)
double taskconfig[4] = {0};  //now position in task space
double jointconfig[4] = {0}; //now position in configuration space
double jointconfig_t[4] = {0};
double tasksetpoint[4]; //target point in task space
double jointsetpoint[4]; //target point in configuration space

float jointworkspace[4] = {J1_WORKSPACE,J2_WORKSPACE,J3_WORKSPACE,J4_WORKSPACE};
uint16_t motor_max_frequency[4] = {JOINT1_MAX_FREQUENCY,JOINT2_MAX_FREQUENCY,JOINT3_MAX_FREQUENCY,JOINT4_MAX_FREQUENCY};
uint32_t radtopulse[4] = {JOINT1_RADTOPULSE,JOINT2_RADTOPULSE,JOINT3_MMTOPULSE,JOINT4_RADTOPULSE};
uint8_t joint = 0;

uint32_t timestamp = 0;
Controller position_jointcontroller[4];
Controller velocity_jointcontroller[4];
Kalmanfilter kalmanjoint[4];

uint8_t Proximity_state[4];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void mhainw_robot_sethome();
float map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

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

  mhainw_control_init(&position_jointcontroller[0],5,1,0);
  mhainw_control_init(&position_jointcontroller[1],7,1,0);
  mhainw_control_init(&position_jointcontroller[2],3.5,0,0);
  mhainw_control_init(&position_jointcontroller[3],3,0,0);

  mhainw_control_init(&velocity_jointcontroller[0],3,0,0);
  mhainw_control_init(&velocity_jointcontroller[1],3,0,0);
  mhainw_control_init(&velocity_jointcontroller[2],3,0,0);
  mhainw_control_init(&velocity_jointcontroller[3],3,0,0);

  mhainw_kalmanfilter_init(&kalmanjoint[0],0,0,0,0,0,0,kalman_Q,kalman_R);
  mhainw_kalmanfilter_init(&kalmanjoint[1],0,0,0,0,0,0,kalman_Q,kalman_R);
  mhainw_kalmanfilter_init(&kalmanjoint[2],0,0,0,0,0,0,kalman_Q,kalman_R);
  mhainw_kalmanfilter_init(&kalmanjoint[3],0,0,0,0,0,0,kalman_Q,kalman_R);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  mhainw_robot_sethome();
//  double dX[4] = {0,0,0,-10};
//  jointconfig_t[0] = 0.7;
//  jointconfig_t[1] = 3.14/2;
//  jointconfig_t[2] = 14;
//  jointconfig_t[3] = -0.7;
//  FPK(jointconfig_t,taskconfig);
//  IVK(jointconfig_t,dX,dq);
//  jointsetpoint[0] = dq[0] + jointconfig_t[0];
//  jointsetpoint[1] = dq[1] + jointconfig_t[1];
//  jointsetpoint[2] = dq[2] + jointconfig_t[2];
//  jointsetpoint[3] = dq[3] + jointconfig_t[3];
//  FPK(jointsetpoint,tasksetpoint);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(HAL_GetTick() - timestamp >= 1){
		timestamp = HAL_GetTick();

		for(joint = 0;joint <4;joint++){
			//read encoder
			jointstate[joint] += mhainw_amt10_unwrap(&encoders[joint]);
			//convert pulse to rad
			jointconfig[joint] =  jointstate[joint] / radtopulse[joint];
			// PID output
			mhainw_control_controllerupdate(&position_jointcontroller[joint], setpoint[joint], jointstate[joint]);
			//map PID output to motor frequency range
			position_jointcontroller[joint].output = map(position_jointcontroller[joint].output, 0,
					jointworkspace[joint], ALLJOINT_MIN_FREQUENCY, motor_max_frequency[joint]);
			//drive motor
			mhainw_stepper_setspeed(&motors[joint], position_jointcontroller[joint].output);
		}
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
			mhainw_stepper_setspeed(&motors[1], -300);
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
	jointstate[0] = J1_OFFSETTOHOMECONFIGURATION;
	setpoint[0] = J1_OFFSETTOHOMECONFIGURATION;
	jointstate[1] = J2_OFFSETTOHOMECONFIGURATION;
	setpoint[1] = J2_OFFSETTOHOMECONFIGURATION;
	jointstate[2] = J3_OFFSETTOHOMECONFIGURATION;
	setpoint[2] = J3_OFFSETTOHOMECONFIGURATION;
	jointstate[3] = J4_OFFSETTOHOMECONFIGURATION;
	setpoint[3] = J4_OFFSETTOHOMECONFIGURATION;
}

float map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
