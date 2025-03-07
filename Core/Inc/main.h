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
#include "stm32g0xx_hal.h"

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
#define C_SGPIO_SClock_Pin GPIO_PIN_2
#define C_SGPIO_SClock_GPIO_Port GPIOC
#define C_SGPIO_SClock_EXTI_IRQn EXTI2_3_IRQn
#define A_SGPIO_SClock_Pin GPIO_PIN_0
#define A_SGPIO_SClock_GPIO_Port GPIOB
#define A_SGPIO_SClock_EXTI_IRQn EXTI0_1_IRQn
#define A_SGPIO_SLoad_Pin GPIO_PIN_1
#define A_SGPIO_SLoad_GPIO_Port GPIOB
#define A_SGPIO_SLoad_EXTI_IRQn EXTI0_1_IRQn
#define A_SGPIO_SData_Pin GPIO_PIN_2
#define A_SGPIO_SData_GPIO_Port GPIOB
#define B_SGPIO_SClock_Pin GPIO_PIN_11
#define B_SGPIO_SClock_GPIO_Port GPIOB
#define B_SGPIO_SClock_EXTI_IRQn EXTI4_15_IRQn
#define B_SGPIO_SLoad_Pin GPIO_PIN_12
#define B_SGPIO_SLoad_GPIO_Port GPIOB
#define B_SGPIO_SLoad_EXTI_IRQn EXTI4_15_IRQn
#define B_SGPIO_SData_Pin GPIO_PIN_13
#define B_SGPIO_SData_GPIO_Port GPIOB
#define E_SGPIO_SClock_Pin GPIO_PIN_9
#define E_SGPIO_SClock_GPIO_Port GPIOA
#define E_SGPIO_SClock_EXTI_IRQn EXTI4_15_IRQn
#define E_SGPIO_SLoad_Pin GPIO_PIN_6
#define E_SGPIO_SLoad_GPIO_Port GPIOC
#define E_SGPIO_SLoad_EXTI_IRQn EXTI4_15_IRQn
#define C_SGPIO_SLoad_Pin GPIO_PIN_7
#define C_SGPIO_SLoad_GPIO_Port GPIOC
#define C_SGPIO_SLoad_EXTI_IRQn EXTI4_15_IRQn
#define C_SGPIO_SData_Pin GPIO_PIN_8
#define C_SGPIO_SData_GPIO_Port GPIOD
#define F_SGPIO_SClock_Pin GPIO_PIN_10
#define F_SGPIO_SClock_GPIO_Port GPIOA
#define F_SGPIO_SClock_EXTI_IRQn EXTI4_15_IRQn
#define RED_I2C2_SCL_Pin GPIO_PIN_11
#define RED_I2C2_SCL_GPIO_Port GPIOA
#define RED_I2C2_SDA_Pin GPIO_PIN_12
#define RED_I2C2_SDA_GPIO_Port GPIOA
#define F_SGPIO_SLoad_Pin GPIO_PIN_15
#define F_SGPIO_SLoad_GPIO_Port GPIOA
#define F_SGPIO_SLoad_EXTI_IRQn EXTI4_15_IRQn
#define F_SGPIO_SData_Pin GPIO_PIN_8
#define F_SGPIO_SData_GPIO_Port GPIOC
#define E_SGPIO_SData_Pin GPIO_PIN_2
#define E_SGPIO_SData_GPIO_Port GPIOD
#define D_SGPIO_SLoad_Pin GPIO_PIN_5
#define D_SGPIO_SLoad_GPIO_Port GPIOD
#define D_SGPIO_SLoad_EXTI_IRQn EXTI4_15_IRQn
#define D_SGPIO_SData_Pin GPIO_PIN_6
#define D_SGPIO_SData_GPIO_Port GPIOD
#define D_SGPIO_SClock_Pin GPIO_PIN_3
#define D_SGPIO_SClock_GPIO_Port GPIOB
#define D_SGPIO_SClock_EXTI_IRQn EXTI2_3_IRQn
#define TEMP_I2C1_SCL_Pin GPIO_PIN_6
#define TEMP_I2C1_SCL_GPIO_Port GPIOB
#define TEMP_I2C_SDA_Pin GPIO_PIN_7
#define TEMP_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
