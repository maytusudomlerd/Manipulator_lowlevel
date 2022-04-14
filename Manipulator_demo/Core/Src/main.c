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
//5000 0.001
#define kalman_Q 500 //2500
#define kalman_R 0.0001

#define CONVERT_CHESSBOARD_PULSETORAD 0
#define PULSEPERROUND 8192

#define JOINT1_DEGTOPULSE            PULSEPERROUND / 360
#define JOINT2_DEGTOPULSE            PULSEPERROUND / 360
#define JOINT3_MMTOPULSE             (PULSEPERROUND * 2.5625) / 8
#define JOINT4_DEGTOPULSE            (PULSEPERROUND * 4) / 360

#define JOINT1_RADTOPULSE            PULSEPERROUND / (PI * 2)
#define JOINT2_RADTOPULSE            PULSEPERROUND / (PI * 2)
#define JOINT4_RADTOPULSE            (PULSEPERROUND * 4) / (PI * 2)

#define DEGTORAD                     0.0174532925

#define J1_OFFSETTOHOMECONFIGURATION 1676
#define J2_OFFSETTOHOMECONFIGURATION -3401
//#define J2_OFFSETTOHOMECONFIGURATION 62890
#define J3_OFFSETTOHOMECONFIGURATION 0
#define J4_OFFSETTOHOMECONFIGURATION 6427

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
float chessboardrad;

float positive_jointlimit[4] = {J1_POSITIVE_JOINTLIMIT, J2_POSITIVE_JOINTLIMIT, J3_UP_JOINTLIMIT, J4_POSITIVE_JOINTLIMIT};
float negative_jointlimit[4] = {J1_NEGATIVE_JOINTLIMIT, J2_NEGATIVE_JOINTLIMIT, J3_DOWN_JOINTLIMIT, J4_NEGATIVE_JOINTLIMIT};
uint8_t joint = 0;
float radtopulse[4] = {JOINT1_RADTOPULSE,JOINT2_RADTOPULSE,JOINT3_MMTOPULSE,JOINT4_RADTOPULSE};

float target_position[4] = { 0 };
float jointstate[4]; //real time value from encoder
float setpoint[4];   //target point in encoder unit(pulse)
float taskconfig[4] = {0};  //now position in task space
float jointconfig[4] = {0}; //now position in configuration space
float tasksetpoint[4]; //target point in task space
float jointreadysetpoint[4] = {1.04,-0.78,0,0}; //target point in configuration space
float jointsetpoint[4] = {0}; //target point in configuration space
float jointsubsetpoint[4] = {0};
float jointvelocitysetpoint[4] = {0}; //target velocity in configuration space

float temp[4];
int16_t goal;


uint32_t timestamp = 0;
uint32_t read_timestamp = 0;
uint32_t feedback_timestamp = 0;
Controller position_jointcontroller[4];
Controller pid_position[4];
Controller velocity_jointcontroller[4];
Kalmanfilter kalmanjoint[4];
Trajectory quinticTrajectory[4];



uint8_t Proximity_state[4];

uint8_t setpoint_test_indx[4] = {0};
float setpoint_test[2][4] = {{0, (J2_NEGATIVE_JOINTLIMIT)/2, (J3_UP_JOINTLIMIT), (J4_NEGATIVE_JOINTLIMIT)/2},
		                     {0,(J2_NEGATIVE_JOINTLIMIT)/2, -80, (J4_POSITIVE_JOINTLIMIT)/2}};

uint8_t robot_feedback = 0;
uint32_t init_t = 0;

uint8_t init_kalman = 1;
uint8_t control_flag = 0;
uint8_t grip = 0;
uint8_t ungrip = 0;

int num = 0;
int map_indx = 1;
float x,y;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void mhainw_robot_sethome();
void mhainw_command_statemachine(Protocol *uart);
void jogcatesian(Protocol *uart,float *jointsetpoint);
void jogjoint(Protocol *uart,float *jointsetpoint);
void movejoint(Protocol *uart,float *jointsetpoint);
void movecartesian(Protocol *uart,float *jointsetpoint);

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
  MX_TIM8_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  mhainw_protocol_init(&user, &huart3);

  mhainw_stepper_init(&motors[0], &htim13, TIM_CHANNEL_1 , dir1_GPIO_Port, dir1_Pin);
  mhainw_stepper_init(&motors[1], &htim16, TIM_CHANNEL_1 , dir3_GPIO_Port, dir3_Pin);
  mhainw_stepper_init(&motors[2], &htim14, TIM_CHANNEL_1 , dir2_GPIO_Port, dir2_Pin);
  mhainw_stepper_init(&motors[3], &htim17, TIM_CHANNEL_1 , dir4_GPIO_Port, dir4_Pin);

  mhainw_amt10_init(&encoders[0], &htim1);
  mhainw_amt10_init(&encoders[1], &htim2);
  mhainw_amt10_init(&encoders[2], &htim3);
  mhainw_amt10_init(&encoders[3], &htim4);
  mhainw_amt10_init(&chessboardenc, &htim8);

  //initial controller for only position control
  mhainw_control_init(&pid_position[0],3500,0.01,100,600,600);
  mhainw_control_init(&pid_position[1],1500,0.001,50,1000,1000);
  mhainw_control_init(&pid_position[2],1000,0.01,0,300,2500);
  mhainw_control_init(&pid_position[3],1000,0,0,300,600);

  //initial controller for case-cade control
  mhainw_control_init(&position_jointcontroller[0],20,0,0,50,300); //kp = 20
  mhainw_control_init(&position_jointcontroller[1],50,0,0,50,1000);  //kp=50
  mhainw_control_init(&position_jointcontroller[2],5,0,0,50,2500);
  mhainw_control_init(&position_jointcontroller[3],40,0,0,50,300);
  mhainw_control_init(&velocity_jointcontroller[0],100,0.001,50,300,300); //kp = 100 ki=0.001 kd =100
  mhainw_control_init(&velocity_jointcontroller[1],50,0.001,300,1000,1000); //kp= 50 ki=0.001
  mhainw_control_init(&velocity_jointcontroller[2],50,0,1,2500,2500); //ki = 0.01
  mhainw_control_init(&velocity_jointcontroller[3],200,0.01,50,300,300); //ki=0.01

  mhainw_kalmanfilter_init(&kalmanjoint[0],0,-1,0,1,0,1,3000,0.0001);
  mhainw_kalmanfilter_init(&kalmanjoint[1],0,0,0,0,0,0.0147,kalman_Q,kalman_R);
  mhainw_kalmanfilter_init(&kalmanjoint[2],0,0,0,0,0,0.0147,kalman_Q,kalman_R);
  mhainw_kalmanfilter_init(&kalmanjoint[3],0,0,0,0,0,0.0147,kalman_Q,kalman_R);

  mhainw_trajectory_init(&quinticTrajectory[0],0.001);
  mhainw_trajectory_init(&quinticTrajectory[1],0.001);
  mhainw_trajectory_init(&quinticTrajectory[2],0.001);
  mhainw_trajectory_init(&quinticTrajectory[3],0.001);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //manipulator set zero
  mhainw_robot_sethome();

//  initial kalman filter
  if(init_kalman){
	  control_flag = 0;
	  init_t = HAL_GetTick();
	  while(init_kalman){
		  if(!(HAL_GetTick() - init_t >= 2000)){
			  for(joint =0;joint<4;joint++){
				//read encoder
				jointstate[joint] += mhainw_amt10_unwrap(&encoders[joint]);
				//convert pulse to rad
				jointconfig[joint] =  jointstate[joint] / radtopulse[joint];
				mhainw_kalmanfilter_updatekalman(&kalmanjoint[joint],jointconfig[joint]);
			  }
		  } else{
			  init_kalman = 0;
			  control_flag = 1;
		  }
	  }
  }
  mhainw_trajectory_generatetraj(quinticTrajectory,jointconfig,jointsetpoint);

/* test trarj
  for(int i=0;i<4;i++){
	  jointsetpoint[i] = setpoint_test[0][i];
  }
  mhainw_trajectory_generatetraj(quinticTrajectory,jointconfig,jointsetpoint);
 */

 /*test mapping position
  chessboardtemptochessboard(num,&x,&y);
  chessboardtorobot(x,y,0,tasksetpoint);
  IPK(tasksetpoint,-1,jointsetpoint);
  jointsetpoint[2] = 0;
  mhainw_trajectory_generatetraj(quinticTrajectory,jointconfig,jointsetpoint);
  */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(grip){
		  UARTsentGripper(&user,MHAINW_GRIPPER_GRIP);
		  grip=0;
	  }
	  if(ungrip){
		  UARTsentGripper(&user,MHAINW_GRIPPER_UNGRIP);
		  ungrip=0;
	  }

	  /*test mapping position
		if(num > 0){
			pointinchessboardtomanipulator(num,jointsetpoint);
			mhainw_trajectory_generatetraj(quinticTrajectory,jointconfig,jointsetpoint);
			if(num == 64){
				num = 0;
			}
		}
	  */

	  //update robot position 2000 hz
	  if(HAL_GetTick() - read_timestamp >= 0.5){
		  read_timestamp = HAL_GetTick();
		  for(int joint=0;joint<4;joint++){
			//read encoder
			jointstate[joint] += mhainw_amt10_unwrap(&encoders[joint]);
			//convert pulse to rad
			jointconfig[joint] =  jointstate[joint] / radtopulse[joint];
			//update kalman x1,x2
			mhainw_kalmanfilter_updatekalman(&kalmanjoint[joint],jointconfig[joint]);
		  }
		  chessboardpos += mhainw_amt10_unwrap(&chessboardenc) ;
		  chessboardrad = chessboardpos * CONVERT_CHESSBOARD_PULSETORAD;
	  }

	  if(control_flag ==1 ){

		  //update robot position 2000 hz
		  if(HAL_GetTick() - read_timestamp >= 0.5){
			  read_timestamp = HAL_GetTick();
			  for(int joint=0;joint<4;joint++){
				//read encoder
				jointstate[joint] += mhainw_amt10_unwrap(&encoders[joint]);
				//convert pulse to rad
				jointconfig[joint] =  jointstate[joint] / radtopulse[joint];
				//update kalman x1,x2
				mhainw_kalmanfilter_updatekalman(&kalmanjoint[joint],jointconfig[joint]);
			  }
			  chessboardpos += mhainw_amt10_unwrap(&chessboardenc) ;
			  chessboardrad = chessboardpos * CONVERT_CHESSBOARD_PULSETORAD;
		  }

		  //update control loop 1000 hz
		  if(HAL_GetTick() - timestamp >= 1){
			timestamp = HAL_GetTick();
			for(joint =0;joint<4;joint++){
				//update trajectory
				if(quinticTrajectory[joint].havetraj == 1){
//					quinticTrajectory[joint].t = (HAL_GetTick() - quinticTrajectory[joint].initial_time) / 1000;
					if(quinticTrajectory[joint].t <= quinticTrajectory[joint].Tk){
						mhainw_trajectory_updatetraj(&quinticTrajectory[joint]); // q da ddq viapoint
						jointsetpoint[joint] = quinticTrajectory[joint].q;
						jointvelocitysetpoint[joint] = quinticTrajectory[joint].dq;
					}
					else{
						quinticTrajectory[joint].havetraj = 0;
						user.goal_reach[joint] = 1;

						/* terst traj
						for(int i=0;i<4;i++){
							setpoint_test_indx[i] = (setpoint_test_indx[i] + 1) % 2;
							jointsetpoint[i] = setpoint_test[setpoint_test_indx[i]][i];
						}
						mhainw_trajectory_generatetraj(quinticTrajectory,jointconfig,jointsetpoint);
						 */

						/* map test
						num = (num + 1) % 64;
						pointinchessboardtomanipulator(num,jointsetpoint);
						mhainw_trajectory_generatetraj(quinticTrajectory,jointconfig,jointsetpoint);
						*/

					}
				}

				//check joint limit
				if(jointsetpoint[joint] >= positive_jointlimit[joint] * 0.8){
					jointsetpoint[joint] = positive_jointlimit[joint] * 0.8;
				}
			    if(jointsetpoint[joint] <= negative_jointlimit[joint] * 0.8){
				  jointsetpoint[joint] = negative_jointlimit[joint] * 0.8;
			    }

			    //control
			    if(quinticTrajectory[joint].havetraj == 1){
				//update position control
				mhainw_control_controllerupdate(&position_jointcontroller[joint], jointsetpoint[joint], kalmanjoint[joint].x1);

				//controller with tune parameter for only position control
//				mhainw_control_controllerupdate(&pid_position[joint], jointsetpoint[joint], kalmanjoint[joint].x1);

				//update velocity control
				mhainw_control_controllerupdate(&velocity_jointcontroller[joint],jointvelocitysetpoint[joint] + position_jointcontroller[joint].output , kalmanjoint[joint].x2);

				mhainw_stepper_setspeed(&motors[joint], velocity_jointcontroller[joint].output);
//				mhainw_stepper_setspeed(&motors[joint], pid_position[joint].output);
			    }

			    else {
			    	mhainw_control_controllerupdate(&pid_position[joint], jointsetpoint[joint], kalmanjoint[joint].x1);
					mhainw_stepper_setspeed(&motors[joint], pid_position[joint].output);
			    }

			}

		  }
		  if(user.package_verify){
			  mhainw_command_statemachine(&user);
			}
	  }

//	  robot feedback
//	  if(robot_feedback){
//		  if(HAL_GetTick() - feedback_timestamp >= 500){
//			  UARTsentFeedback(&user, MHAINW_JOINT_FEEDBACK,jointconfig,4);
//		  }
//	  }
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

	while(Proximity_state[2] != 1){
			mhainw_stepper_setspeed(&motors[2], 1000);
	}
	mhainw_stepper_setspeed(&motors[2], 0);

	//state 2 : move to home position
	while(sethome[0] != 1 || sethome[1] !=1 ){

		if(Proximity_state[0] == 1){
			mhainw_stepper_setspeed(&motors[0], 0);
			sethome[0] = 1;
		} else{
			mhainw_stepper_setspeed(&motors[0], 100);
		}
		if(Proximity_state[1] == 1){
			mhainw_stepper_setspeed(&motors[1], 0);
			sethome[1] = 1;
		} else{
			mhainw_stepper_setspeed(&motors[1], -250);
		}
	}
	HAL_Delay(1000);

	//state 3 : set current position and offset home configuration
	htim1.Instance->CNT = 0;
	htim2.Instance->CNT = 0;
	htim3.Instance->CNT = 0;
	htim4.Instance->CNT = 0;
	HAL_Delay(500);

	for(int i=0;i<4;i++){
		jointstate[i] = jointoffset[i];
		jointconfig[i] = jointstate[i] / radtopulse[i];
		jointsetpoint[i] = jointreadysetpoint[i];
	}

}

void mhainw_robot_movetohome(){
	uint8_t sethome[3] = {0};
	mhainw_stepper_setspeed(&motors[2], -1000);
	HAL_Delay(500);
	mhainw_stepper_setspeed(&motors[2], 0);
	//state 2 : move to home position
	while(sethome[0] != 1 || sethome[1] !=1 || sethome[2] != 1 || sethome[3] != 1){

		if(Proximity_state[0] == 1){
			mhainw_stepper_setspeed(&motors[0], 0);
			sethome[0] = 1;
		} else{
			mhainw_stepper_setspeed(&motors[0], 200);
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
		if(Proximity_state[2] == 1){
			mhainw_stepper_setspeed(&motors[3], 0);
			sethome[3] = 1;
		} else{
			mhainw_stepper_setspeed(&motors[3], 500);
		}
	}
}

void mhainw_command_statemachine(Protocol *uart){
	static uint8_t set = 1;
	static uint8_t inst;
	static uint8_t state = MHAINW_WAIT;

	//task move
	static uint8_t end_path = 0;
	static float path_setpoint[24];
	static uint8_t action;
	static uint8_t path_no = 0;

	if(set){
		state = uart->inst;
		set=0;
	}
	switch(state){
		case MHAINW_SETHOME:
			UARTsentACK(uart, MHAINW_SETHOME_ACK);
			for(int i=0;i<4;i++){
				jointsetpoint[i] = jointreadysetpoint[i];
			}
			state = MHAINW_TRAJGEN;
			break;

		case MHAINW_JOG_CATESIAN:
			UARTsentACK(uart, MHAINW_JOG_ACK);
			jogcatesian(uart,jointsetpoint);
			inst = MHAINW_JOG_ACK;
			state = MHAINW_TRAJGEN;
			break;

		case MHAINW_JOG_JOINT:
			UARTsentACK(uart, MHAINW_JOG_ACK);
			jogjoint(uart,jointsetpoint);
			inst = MHAINW_JOG_ACK;
			state = MHAINW_TRAJGEN;
			break;

		case MAHINW_MOVE_CATESIAN :
			UARTsentACK(uart, MHAINW_MOVE_ACK);
			movecartesian(uart,jointsetpoint);
			inst = MHAINW_MOVE_ACK;
			state = MHAINW_TRAJGEN;
			break;

		case MHAINW_MOVE_JOINT:
			UARTsentACK(uart, MHAINW_MOVE_ACK);
			movejoint(uart,jointsetpoint);
			inst = MHAINW_MOVE_ACK;
			state = MHAINW_TRAJGEN;
			break;

		case MHAINW_MOVE_TASK:
			UARTsentACK(uart, MHAINW_TASKMOVE_ACK);
			state = MAHINW_MOVE_SETPOINTGEN;

		case MAHINW_MOVE_SETPOINTGEN:
			action = uart->data[2];
			if(action == 1){
				float temp_to[4];
				float temp_from[4];
				uint8_t from = uart->data[0];
				uint8_t to = uart->data[1];
				pointinchessboardtomanipulator(uart->data[0],temp_from); //from
				pointinchessboardtomanipulator(uart->data[1],temp_to); //to
				//set x y position of each via point
				//from
				memcpy(&path_setpoint[0],temp_from,sizeof(temp_from));
				memcpy(&path_setpoint[4],temp_from,sizeof(temp_from));
				memcpy(&path_setpoint[8],temp_from,sizeof(temp_from));
				//to
				memcpy(&path_setpoint[12],temp_to,sizeof(temp_to));
				memcpy(&path_setpoint[16],temp_to,sizeof(temp_to));
				memcpy(&path_setpoint[20],temp_to,sizeof(temp_to));
				//set z distance in each via point
				path_setpoint[2] = -50;
				path_setpoint[6] = -100;
				path_setpoint[10] = -50;
				path_setpoint[14] = -50;
				path_setpoint[18] = -100;
				path_setpoint[22] = -50;
			}

//			else if(action == 2){
//				//check
//			}
			state = MHAINW_MOVE_SETPOINTUPDATE;

		case MHAINW_MOVE_SETPOINTUPDATE:
			memcpy(jointsetpoint,&path_setpoint[path_no * 4],sizeof(jointsetpoint));
			path_no = (path_no + 1) % 6;
			if(path_no == 0){
				end_path = 1;
				memcpy(jointsetpoint,jointreadysetpoint,sizeof(jointsetpoint));
			}
			state = MHAINW_TRAJGEN;
			inst = MHAINW_TASKMOVE_ACK;
			break;

		case MHAINW_POSITION_CHESSBOARD:
			UARTsentFeedback(uart, MHAINW_POSITION_ACK,&chessboardrad,1);
			state = MHAINW_WAIT;
			set = 1;
			uart->package_verify = 0;
			break;

		case MHAINW_POSITION_MANIPULATOR:
			UARTsentFeedback(uart, MHAINW_POSITION_ACK,jointconfig,4);
			state = MHAINW_WAIT;
			set = 1;
			uart->package_verify = 0;
			break;

		case MHAINW_CHESSBOARD_SETZERO:
			chessboardenc.Timehandle->Instance->CNT = 0;
			UARTsentFeedback(uart, MHAINW_POSITION_ACK,&chessboardrad,1);
			break;

		case MHAINW_WAIT:
			if(uart->goal_reach[0] == 1 && uart->goal_reach[1] == 1 && uart->goal_reach[2] == 1 && uart->goal_reach[3] == 1){
				uart->package_verify = 0;

				if(inst == MHAINW_JOG_ACK){
					UARTsentFeedback(uart, MHAINW_JOG_ACK,jointconfig,4);
				} else if(inst  == MHAINW_MOVE_ACK){
					UARTsentFeedback(uart, MHAINW_MOVE_ACK,jointconfig,4);
				} else if(inst == MHAINW_TASKMOVE_ACK){
					state = MHAINW_MOVE_SETPOINTUPDATE;
					uart->package_verify = 1;
					if(end_path){
						end_path = 0;
						set=1;
						inst = 0;
						UARTsentFeedback(uart, MHAINW_TASKMOVE_ACK,jointconfig,4);
						uart->package_verify = 0;
					}
				}
				//.....
				if(inst != MHAINW_TASKMOVE_ACK){
					set=1;
					inst = 0;
				}
			}

			break;
	  }
	if(state == MHAINW_TRAJGEN){
		for(int i=0;i<4;i++){
			uart->goal_reach[i] = 0;
			mhainw_trajectory_generatetraj(quinticTrajectory,jointconfig,jointsetpoint);
		}
		state = MHAINW_WAIT;
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
void movecartesian(Protocol *uart,float *jointsetpoint){
	float taskspace_setpoint[4] = {0};
	float temp_jointsetpoint[4] = {0};
	float temp_taskconfig[4] = {0};
	float temp_homeconfiguration[4] = {J1_OFFSETTOHOMECONFIGURATION, J2_OFFSETTOHOMECONFIGURATION, J3_OFFSETTOHOMECONFIGURATION, J4_OFFSETTOHOMECONFIGURATION};
	float temp_jointconfig[4] = {jointsetpoint[0],jointsetpoint[1],jointsetpoint[2],jointsetpoint[3]};
	uint8_t move_ref = uart->data[0] & 0x01;
	robot_feedback = uart->data[0] & 0x02;
	if(move_ref == 1){
		FPK(temp_jointconfig,temp_taskconfig);
	} else {
		FPK(temp_homeconfiguration,temp_taskconfig);
	}

	taskspace_setpoint[1] = temp_taskconfig[1] + (int16_t)((uart->data[1] << 8) | (uint8_t)uart->data[2]);
	taskspace_setpoint[2] = temp_taskconfig[2] + (int16_t)((uart->data[3] << 8) | (uint8_t)uart->data[4]);
	taskspace_setpoint[3] = temp_taskconfig[3] + (int16_t)((uart->data[5] << 8) | (uint8_t)uart->data[6]);
	taskspace_setpoint[0] = temp_taskconfig[0] + (int16_t)((uart->data[7] << 8) | (uint8_t)uart->data[8]);
	IPK(taskspace_setpoint,-1,temp_jointsetpoint);

	for(int i=0;i<4;i++){
		jointsetpoint[i] = temp_jointsetpoint[i];
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
