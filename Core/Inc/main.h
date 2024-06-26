/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_gpio.h"

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
#define USE_CAN
#define USE_GPIO_LL
#define USE_UART
void SystemClock_Config(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IO9_Pin LL_GPIO_PIN_1
#define IO9_GPIO_Port GPIOA
#define IO8_Pin LL_GPIO_PIN_2
#define IO8_GPIO_Port GPIOA
#define IO7_Pin LL_GPIO_PIN_3
#define IO7_GPIO_Port GPIOA
#define IO6_Pin LL_GPIO_PIN_4
#define IO6_GPIO_Port GPIOA
#define IO5_Pin LL_GPIO_PIN_5
#define IO5_GPIO_Port GPIOA
#define IO4_Pin LL_GPIO_PIN_6
#define IO4_GPIO_Port GPIOA
#define IO3_Pin LL_GPIO_PIN_7
#define IO3_GPIO_Port GPIOA
#define IO2_Pin LL_GPIO_PIN_0
#define IO2_GPIO_Port GPIOB
#define IO1_Pin LL_GPIO_PIN_1
#define IO1_GPIO_Port GPIOB
#define LED_B_Pin LL_GPIO_PIN_8
#define LED_B_GPIO_Port GPIOA
#define LED_G_Pin LL_GPIO_PIN_9
#define LED_G_GPIO_Port GPIOA
#define LED_R_Pin LL_GPIO_PIN_10
#define LED_R_GPIO_Port GPIOA
#define ID0_Pin LL_GPIO_PIN_4
#define ID0_GPIO_Port GPIOB
#define ID2_Pin LL_GPIO_PIN_5
#define ID2_GPIO_Port GPIOB
#define ID1_Pin LL_GPIO_PIN_6
#define ID1_GPIO_Port GPIOB
#define ID3_Pin LL_GPIO_PIN_7
#define ID3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
