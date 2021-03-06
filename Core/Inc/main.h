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
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LATCH0_Pin GPIO_PIN_0
#define LATCH0_GPIO_Port GPIOA
#define LATCH1_Pin GPIO_PIN_1
#define LATCH1_GPIO_Port GPIOA
#define LATCH2_Pin GPIO_PIN_2
#define LATCH2_GPIO_Port GPIOA
#define LATCH3_Pin GPIO_PIN_3
#define LATCH3_GPIO_Port GPIOA
#define LATCH4_Pin GPIO_PIN_4
#define LATCH4_GPIO_Port GPIOA
#define OE_Pin GPIO_PIN_5
#define OE_GPIO_Port GPIOA
#define SW0_Pin GPIO_PIN_6
#define SW0_GPIO_Port GPIOA
#define SW0_EXTI_IRQn EXTI9_5_IRQn
#define SW1_Pin GPIO_PIN_7
#define SW1_GPIO_Port GPIOA
#define SW1_EXTI_IRQn EXTI9_5_IRQn
#define DB0_Pin GPIO_PIN_0
#define DB0_GPIO_Port GPIOB
#define DB1_Pin GPIO_PIN_1
#define DB1_GPIO_Port GPIOB
#define DB2_Pin GPIO_PIN_2
#define DB2_GPIO_Port GPIOB
#define DB10_Pin GPIO_PIN_10
#define DB10_GPIO_Port GPIOB
#define DB11_Pin GPIO_PIN_11
#define DB11_GPIO_Port GPIOB
#define DB12_Pin GPIO_PIN_12
#define DB12_GPIO_Port GPIOB
#define DB13_Pin GPIO_PIN_13
#define DB13_GPIO_Port GPIOB
#define DB14_Pin GPIO_PIN_14
#define DB14_GPIO_Port GPIOB
#define DB15_Pin GPIO_PIN_15
#define DB15_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_8
#define SW2_GPIO_Port GPIOA
#define SW2_EXTI_IRQn EXTI9_5_IRQn
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_15
#define SW3_GPIO_Port GPIOA
#define SW3_EXTI_IRQn EXTI15_10_IRQn
#define DB3_Pin GPIO_PIN_3
#define DB3_GPIO_Port GPIOB
#define DB4_Pin GPIO_PIN_4
#define DB4_GPIO_Port GPIOB
#define DB5_Pin GPIO_PIN_5
#define DB5_GPIO_Port GPIOB
#define DB6_Pin GPIO_PIN_6
#define DB6_GPIO_Port GPIOB
#define DB7_Pin GPIO_PIN_7
#define DB7_GPIO_Port GPIOB
#define DB8_Pin GPIO_PIN_8
#define DB8_GPIO_Port GPIOB
#define DB9_Pin GPIO_PIN_9
#define DB9_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
