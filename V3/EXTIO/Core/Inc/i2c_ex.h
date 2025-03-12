/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c_ex.h
  * @brief   This file contains all the function prototypes for
  *          the i2c_ex.c file
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
#ifndef __I2C_EX_H__
#define __I2C_EX_H__
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern volatile uint32_t i2c_timeout_counter;
extern volatile uint32_t i2c_stop_timeout_flag;
extern volatile uint32_t i2c_stop_timeout_counter;

extern void i2c1_it_enable(void);
extern void i2c1_it_disable(void);
extern void i2c1_set_send_data(uint8_t *tx_ptr, uint16_t len);
extern void set_i2c_slave_address(uint8_t addr);
#ifdef __cplusplus
}
#endif
#endif
