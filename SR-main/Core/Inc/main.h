/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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
#define IRQ_HW_L_Pin GPIO_PIN_5
#define IRQ_HW_L_GPIO_Port GPIOE
#define IRQ_HW_L_EXTI_IRQn EXTI9_5_IRQn
#define XC_Pin GPIO_PIN_15
#define XC_GPIO_Port GPIOC
#define MOTOR_R_IN2_Pin GPIO_PIN_7
#define MOTOR_R_IN2_GPIO_Port GPIOE
#define MOTOR_R_IN1N_Pin GPIO_PIN_8
#define MOTOR_R_IN1N_GPIO_Port GPIOE
#define XCE9_Pin GPIO_PIN_9
#define XCE9_GPIO_Port GPIOE
#define MOTOR_L_IN2_Pin GPIO_PIN_12
#define MOTOR_L_IN2_GPIO_Port GPIOE
#define MOTOR_L_IN1_Pin GPIO_PIN_13
#define MOTOR_L_IN1_GPIO_Port GPIOE
#define KEY_L_Pin GPIO_PIN_15
#define KEY_L_GPIO_Port GPIOE
#define KEY_L_EXTI_IRQn EXTI15_10_IRQn
#define GS_Pin GPIO_PIN_12
#define GS_GPIO_Port GPIOB
#define RQS_Pin GPIO_PIN_13
#define RQS_GPIO_Port GPIOB
#define LQS_Pin GPIO_PIN_15
#define LQS_GPIO_Port GPIOB
#define KEY_R_Pin GPIO_PIN_11
#define KEY_R_GPIO_Port GPIOD
#define KEY_R_EXTI_IRQn EXTI15_10_IRQn
#define BEEP_Pin GPIO_PIN_7
#define BEEP_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOD
#define IRQ_24L01_Pin GPIO_PIN_2
#define IRQ_24L01_GPIO_Port GPIOD
#define IRQ_24L01_EXTI_IRQn EXTI2_IRQn
#define CSN_24L01_Pin GPIO_PIN_3
#define CSN_24L01_GPIO_Port GPIOD
#define CE_24L01_Pin GPIO_PIN_4
#define CE_24L01_GPIO_Port GPIOD
#define IRQ_HW_R_Pin GPIO_PIN_8
#define IRQ_HW_R_GPIO_Port GPIOB
#define IRQ_HW_R_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
