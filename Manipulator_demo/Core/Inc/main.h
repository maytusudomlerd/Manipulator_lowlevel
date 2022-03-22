/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define prox1_Pin GPIO_PIN_2
#define prox1_GPIO_Port GPIOE
#define prox1_EXTI_IRQn EXTI2_IRQn
#define prox2_Pin GPIO_PIN_3
#define prox2_GPIO_Port GPIOE
#define prox2_EXTI_IRQn EXTI3_IRQn
#define prox3_Pin GPIO_PIN_4
#define prox3_GPIO_Port GPIOE
#define prox3_EXTI_IRQn EXTI4_IRQn
#define prox4_Pin GPIO_PIN_5
#define prox4_GPIO_Port GPIOE
#define prox4_EXTI_IRQn EXTI9_5_IRQn
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define com485_Rx_Pin GPIO_PIN_6
#define com485_Rx_GPIO_Port GPIOF
#define com485_Tx_Pin GPIO_PIN_7
#define com485_Tx_GPIO_Port GPIOF
#define pulse2_Pin GPIO_PIN_9
#define pulse2_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define pulse1_Pin GPIO_PIN_6
#define pulse1_GPIO_Port GPIOA
#define dir1_Pin GPIO_PIN_1
#define dir1_GPIO_Port GPIOB
#define dir2_Pin GPIO_PIN_2
#define dir2_GPIO_Port GPIOB
#define dir3_Pin GPIO_PIN_11
#define dir3_GPIO_Port GPIOF
#define dir4_Pin GPIO_PIN_15
#define dir4_GPIO_Port GPIOF
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define ENC_A4_Pin GPIO_PIN_12
#define ENC_A4_GPIO_Port GPIOD
#define ENC_B4_Pin GPIO_PIN_13
#define ENC_B4_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define chessboard_A_Pin GPIO_PIN_6
#define chessboard_A_GPIO_Port GPIOC
#define chessboard_B_Pin GPIO_PIN_7
#define chessboard_B_GPIO_Port GPIOC
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define ENC_A2_Pin GPIO_PIN_15
#define ENC_A2_GPIO_Port GPIOA
#define dynamixel_Tx_Pin GPIO_PIN_10
#define dynamixel_Tx_GPIO_Port GPIOC
#define dynamixel_Rx_Pin GPIO_PIN_11
#define dynamixel_Rx_GPIO_Port GPIOC
#define dynamixel_control_Pin GPIO_PIN_12
#define dynamixel_control_GPIO_Port GPIOC
#define ENC_B2_Pin GPIO_PIN_3
#define ENC_B2_GPIO_Port GPIOB
#define ENC_A3_Pin GPIO_PIN_4
#define ENC_A3_GPIO_Port GPIOB
#define ENC_B3_Pin GPIO_PIN_5
#define ENC_B3_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define pulse3_Pin GPIO_PIN_8
#define pulse3_GPIO_Port GPIOB
#define pulse4_Pin GPIO_PIN_9
#define pulse4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
