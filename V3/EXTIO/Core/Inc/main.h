/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
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

extern uint8_t i2c_address[1];
extern volatile uint16_t neopixel_pin;
extern volatile uint8_t color_index;
extern uint32_t *color_buf;
extern volatile uint8_t freq_set_index;
extern uint8_t pwm_correct[5];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IO1_Pin GPIO_PIN_1
#define IO1_GPIO_Port GPIOA
#define IO2_Pin GPIO_PIN_2
#define IO2_GPIO_Port GPIOA
#define IO3_Pin GPIO_PIN_3
#define IO3_GPIO_Port GPIOA
#define IO4_Pin GPIO_PIN_4
#define IO4_GPIO_Port GPIOA
#define IO5_Pin GPIO_PIN_5
#define IO5_GPIO_Port GPIOA
#define IO6_Pin GPIO_PIN_6
#define IO6_GPIO_Port GPIOA
#define IO7_Pin GPIO_PIN_7
#define IO7_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
