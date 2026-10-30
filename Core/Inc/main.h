/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define Motor_B1_Pin GPIO_PIN_5
#define Motor_B1_GPIO_Port GPIOE
#define Motor_B2_Pin GPIO_PIN_6
#define Motor_B2_GPIO_Port GPIOE
#define Battery_Volt_Pin GPIO_PIN_0
#define Battery_Volt_GPIO_Port GPIOB
#define PS2_CLK_Pin GPIO_PIN_8
#define PS2_CLK_GPIO_Port GPIOE
#define Motor_C2_Pin GPIO_PIN_9
#define Motor_C2_GPIO_Port GPIOE
#define PS2_CS_Pin GPIO_PIN_10
#define PS2_CS_GPIO_Port GPIOE
#define Motor_C1_Pin GPIO_PIN_11
#define Motor_C1_GPIO_Port GPIOE
#define PS2_DO_Pin GPIO_PIN_12
#define PS2_DO_GPIO_Port GPIOE
#define Motor_D2_Pin GPIO_PIN_13
#define Motor_D2_GPIO_Port GPIOE
#define Motor_D1_Pin GPIO_PIN_14
#define Motor_D1_GPIO_Port GPIOE
#define PS2_DI_Pin GPIO_PIN_15
#define PS2_DI_GPIO_Port GPIOE
#define MPU6050_SCL_Pin GPIO_PIN_10
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_11
#define MPU6050_SDA_GPIO_Port GPIOB
#define Servo_PWM_Pin GPIO_PIN_15
#define Servo_PWM_GPIO_Port GPIOB
#define OLED_RS_Pin GPIO_PIN_11
#define OLED_RS_GPIO_Port GPIOD
#define OLED_RST_Pin GPIO_PIN_12
#define OLED_RST_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_13
#define OLED_SDA_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_14
#define OLED_SCL_GPIO_Port GPIOD
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define Motor_EN_Pin GPIO_PIN_3
#define Motor_EN_GPIO_Port GPIOD
#define Motor_A1_Pin GPIO_PIN_8
#define Motor_A1_GPIO_Port GPIOB
#define Motor_A2_Pin GPIO_PIN_9
#define Motor_A2_GPIO_Port GPIOB
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
