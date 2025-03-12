/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define Button_GPIO_Port GPIOF
#define Button_Pin GPIO_PIN_0
#define TOTAL_ADC_CH 8
#define TOTAL_RGB 8

#define Steer_0_Pin GPIO_PIN_0
#define Steer_0_GPIO_Port GPIOA
#define Steer_1_Pin GPIO_PIN_1
#define Steer_1_GPIO_Port GPIOA
#define Steer_2_Pin GPIO_PIN_2
#define Steer_2_GPIO_Port GPIOA
#define Steer_3_Pin GPIO_PIN_3
#define Steer_3_GPIO_Port GPIOA
#define Steer_4_Pin GPIO_PIN_4
#define Steer_4_GPIO_Port GPIOA
#define Steer_5_Pin GPIO_PIN_5
#define Steer_5_GPIO_Port GPIOA
#define Steer_6_Pin GPIO_PIN_6
#define Steer_6_GPIO_Port GPIOA
#define Steer_7_Pin GPIO_PIN_7
#define Steer_7_GPIO_Port GPIOA

extern volatile uint16_t neopixel_pin;
extern volatile uint8_t color_index;
extern uint32_t *color_buf;

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
/* USER CODE BEGIN Private defines */
extern __IO uint8_t rgb_state;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
