/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
//#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_ex.h"
//#include "ws2812.h"
//#include "sk6812.h"
#include "arm_math.h"
#include "steer.h"
#include "ws2812.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define abs(x) (((x) > 0) ? (x) : -(x))
#define OFFSET 0.707106

#define INPUT_MODE  0
#define OUTPUT_MODE 1
#define ADC_MODE    2
#define SERVO_MODE  3
#define LED_MODE    4

#define IO0_GPIO GPIOA, GPIO_PIN_0
#define IO1_GPIO GPIOA, GPIO_PIN_1
#define IO2_GPIO GPIOA, GPIO_PIN_2
#define IO3_GPIO GPIOA, GPIO_PIN_3

#define IO4_GPIO GPIOA, GPIO_PIN_4
#define IO5_GPIO GPIOA, GPIO_PIN_5
#define IO6_GPIO GPIOA, GPIO_PIN_6
#define IO7_GPIO GPIOA, GPIO_PIN_7

#define FIRMWARE_VERSION 2
#define I2C_ADDRESS      0x45

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
volatile uint16_t steer_pulse[STEER_NUMBER];
volatile uint8_t steer_angle[STEER_NUMBER];
volatile uint8_t steer_disable[STEER_NUMBER];
volatile uint16_t neopixel_pin = 0;
volatile uint8_t color_index   = 0;
uint8_t debug_flag             = 0;
uint8_t io_mode_save[8]        = {0};
uint8_t i2c_temp[4]            = {0};
uint8_t i2c_address[1]         = {0};
uint8_t flag_timer_start       = 0;
volatile uint8_t fm_version    = FIRMWARE_VERSION;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t input_state = 0;

void user_i2c_init(void) {
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x2000090E;
    hi2c1.Init.OwnAddress1      = i2c_address[0] << 1;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) !=
        HAL_OK) {
        Error_Handler();
    }
    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
}

void i2c_address_write_to_flash(void) {
    writeMessageToFlash(i2c_address, 1);
}

void i2c_address_read_from_flash(void) {
    if (!(readPackedMessageFromFlash(i2c_address, 1))) {
        i2c_address[0] = I2C_ADDRESS;
        i2c_address_write_to_flash();
    }
}

void switchInputAndOutput(uint16_t pin, uint32_t type, uint32_t pull_type,
                          uint32_t freq_type) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (type == GPIO_MODE_ANALOG) __HAL_RCC_ADC1_CLK_ENABLE();
    /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                            PAPin PAPin PAPin PAPin */
    GPIO_InitStruct.Pin   = pin;
    GPIO_InitStruct.Mode  = type;
    GPIO_InitStruct.Pull  = pull_type;
    GPIO_InitStruct.Speed = freq_type;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    if (type == GPIO_MODE_ANALOG) {
        hadc.Instance                   = ADC1;
        hadc.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
        hadc.Init.Resolution            = ADC_RESOLUTION_12B;
        hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
        hadc.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
        hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
        hadc.Init.LowPowerAutoWait      = DISABLE;
        hadc.Init.LowPowerAutoPowerOff  = DISABLE;
        hadc.Init.ContinuousConvMode    = DISABLE;
        hadc.Init.DiscontinuousConvMode = DISABLE;
        hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
        hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
        hadc.Init.DMAContinuousRequests = DISABLE;
        hadc.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
        HAL_ADC_Init(&hadc);
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readADCState(uint32_t channel, uint8_t bit_mode) {
    uint32_t AD_Value              = 0;
    uint32_t Value[22]             = {0};
    uint8_t tmp_buff[16]           = {0};
    uint16_t tmp_buff_len          = 0;
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc.Instance->CHSELR          = 0;
    uint32_t max                   = 0;
    uint32_t min                   = 0;

    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    HAL_ADCEx_Calibration_Start(&hadc);

    for (int n = 0; n < 22; n++) {
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 10);
        if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc), HAL_ADC_STATE_REG_EOC)) {
            Value[n] = HAL_ADC_GetValue(&hadc);
            AD_Value += Value[n];
        }
    }
    max = Value[0];
    min = Value[0];

    for (char n = 0; n < 22; n++)  //取最大值、最小值
    {
        max = (Value[n] < max) ? max : Value[n];
        min = (min < Value[n]) ? min : Value[n];
    }

    AD_Value = (AD_Value - max - min) / 20;

    if (bit_mode == 12) {
        tmp_buff[0]  = (uint8_t)AD_Value;
        tmp_buff[1]  = (uint8_t)(AD_Value >> 8);
        tmp_buff_len = 2;
    } else if (bit_mode == 8) {
        tmp_buff[0]  = map(AD_Value, 0, 4095, 0, 255);
        tmp_buff_len = 1;
    }
    i2c1_set_send_data(tmp_buff, tmp_buff_len);
}

void i2c1_receive_callback(uint8_t *rx_data, uint16_t len) {
    if (len > 1 && (rx_data[0] >= 0x00 & rx_data[0] <= 0x07)) {
        for (int i = 0; i < len - 1; i++) {
            io_mode_save[rx_data[0] + i] = rx_data[1 + i];
            switch (rx_data[1 + i]) {
                case INPUT_MODE:
                    switchInputAndOutput((uint16_t)(1 << rx_data[0] + i),
                                         GPIO_MODE_INPUT, GPIO_PULLUP,
                                         GPIO_SPEED_FREQ_LOW);
                    HAL_TIM_Base_Stop_IT(&htim16);
                    flag_timer_start              = 0;
                    steer_disable[rx_data[0] + i] = 1;
                    neopixel_pin                  = 0;
                    break;
                case OUTPUT_MODE:
                    switchInputAndOutput((uint16_t)(1 << rx_data[0] + i),
                                         GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                                         GPIO_SPEED_FREQ_LOW);
                    HAL_TIM_Base_Stop_IT(&htim16);
                    flag_timer_start              = 0;
                    steer_disable[rx_data[0] + i] = 1;
                    neopixel_pin                  = 0;
                    break;
                case ADC_MODE:
                    switchInputAndOutput((uint16_t)(1 << rx_data[0] + i),
                                         GPIO_MODE_ANALOG, GPIO_NOPULL,
                                         GPIO_SPEED_FREQ_LOW);
                    HAL_TIM_Base_Stop_IT(&htim16);
                    flag_timer_start              = 0;
                    steer_disable[rx_data[0] + i] = 1;
                    neopixel_pin                  = 0;
                    break;
                case SERVO_MODE:
                    switchInputAndOutput((uint16_t)(1 << rx_data[0] + i),
                                         GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                                         GPIO_SPEED_FREQ_LOW);
                    HAL_TIM_Base_Start_IT(&htim16);
                    flag_timer_start              = 1;
                    steer_disable[rx_data[0] + i] = 0;
                    neopixel_pin                  = 0;
                    break;
                case LED_MODE:
                    HAL_TIM_Base_Stop_IT(&htim16);
                    flag_timer_start              = 0;
                    steer_disable[rx_data[0] + i] = 1;
                    switchInputAndOutput((uint16_t)(1 << rx_data[0] + i),
                                         GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                                         GPIO_SPEED_FREQ_MEDIUM);
                    break;

                default:
                    break;
            }
        }
    } else if (len == 1 && (rx_data[0] >= 0x00 & rx_data[0] <= 0x07)) {
        i2c1_set_send_data((uint8_t *)&io_mode_save[rx_data[0]], 1);
    } else if (len > 1 && (rx_data[0] >= 0x10 & rx_data[0] <= 0x17)) {
        for (int i = 0; i < len - 1; i++) {
            if(io_mode_save[i] == OUTPUT_MODE) {
                switch (rx_data[1 + i]) {
                    case 1:
                        HAL_GPIO_WritePin(GPIOA,
                                          (uint16_t)(1 << (rx_data[0] + i - 0x10)),
                                          GPIO_PIN_SET);
                        break;
                    case 0:
                        HAL_GPIO_WritePin(GPIOA,
                                          (uint16_t)(1 << (rx_data[0] + i - 0x10)),
                                          GPIO_PIN_RESET);
                        break;

                    default:
                        break;
                }
            }
        }
    } else if (len > 1 && (rx_data[0] == 0x18)) {
		for (int i = 0; i < 8; i++) {
			if(io_mode_save[i] == OUTPUT_MODE) {
				if ((rx_data[1]>>i)&0x01) {
					HAL_GPIO_WritePin(GPIOA,
									  (uint16_t)(1 << (i)),
									  GPIO_PIN_SET);
				}
				else {
					HAL_GPIO_WritePin(GPIOA,
									  (uint16_t)(1 << (i)),
									  GPIO_PIN_RESET);
				}
			}
		}
    } else if (len == 1 && (rx_data[0] >= 0x20 & rx_data[0] <= 0x27)) {
        if(io_mode_save[rx_data[0] - 0x20] == INPUT_MODE) {
            input_state =
                HAL_GPIO_ReadPin(GPIOA, (uint16_t)(1 << (rx_data[0] - 0x20)));
        }
        else {
            input_state = 0;
        }
        i2c1_set_send_data((uint8_t *)&input_state, 1);
    } else if (len > 1 && (rx_data[0] == 0x28)) {
		input_state = 0;
		for (int i = 0; i < 8; i++) {
			if(io_mode_save[i] == INPUT_MODE) {
				input_state |=
					HAL_GPIO_ReadPin(GPIOA, (uint16_t)(1 << (i)));
			}
		}
		i2c1_set_send_data((uint8_t *)&input_state, 1);
    } else if (len == 1 && (rx_data[0] >= 0x30 & rx_data[0] <= 0x37)) {
        readADCState((rx_data[0] - 0x30), 8);
    } else if (len == 1 && (rx_data[0] >= 0x40 & rx_data[0] <= 0x4F)) {
        readADCState(((rx_data[0] - 0x40) / 2), 12);
    } else if (len > 1 && (rx_data[0] >= 0x50 & rx_data[0] <= 0x57)) {
        for (int i = 0; i < len - 1; i++) {
            SetSteerAngle(rx_data[0] + i - 0x50, rx_data[1 + i]);
        }
    } else if (len == 1 && (rx_data[0] >= 0x50 & rx_data[0] <= 0x57)) {
        i2c1_set_send_data((uint8_t *)&steer_angle[(rx_data[0] - 0x50)], 1);
    } else if (len > 1 && (rx_data[0] >= 0x60 & rx_data[0] <= 0x6F)) {
        for (int i = 0; i < (len - 1) / 2; i++) {
            SetSteerPulse(((rx_data[0] + i * 2 - 0x60) / 2),
                          ((rx_data[2 + i * 2] << 8) | (rx_data[1 + i * 2])));
        }
    } else if (len == 1 && (rx_data[0] >= 0x60 & rx_data[0] <= 0x6F)) {
        i2c_temp[0] = (steer_pulse[((rx_data[0] - 0x60) / 2)] * 10) & 0xff;
        i2c_temp[1] =
            ((steer_pulse[((rx_data[0] - 0x60) / 2)] * 10) >> 8) & 0xff;
        i2c1_set_send_data(i2c_temp, 2);
    } else if (len > 1 && (rx_data[0] >= 0x70 & rx_data[0] <= 0x8F)) {
        for (int i = 0; i < (len - 1) / 3; i++) {
            neopixel_pin = (uint16_t)(1 << ((rx_data[0] - 0x70) / 3));
            neopixel_set_color(
                ((rx_data[0] + i * 3 - 0x70) / 3),
                ((rx_data[1 + i * 3] << 24) | (rx_data[2 + i * 3] << 16) |
                 (rx_data[3 + i * 3] << 8)));
            color_index = (rx_data[0] + i * 3 - 0x70) / 3;
            neopixel_show();
        }
    } else if (len == 1 && (rx_data[0] >= 0x70 & rx_data[0] <= 0x8F)) {
        i2c_temp[0] = (color_buf[((rx_data[0] - 0x70) / 3)] >> 8) & 0xff;
        i2c_temp[1] = (color_buf[((rx_data[0] - 0x70) / 3)] >> 16) & 0xff;
        i2c_temp[2] = (color_buf[((rx_data[0] - 0x70) / 3)]) & 0xff;
        i2c1_set_send_data(i2c_temp, 3);
    } else if (len > 1 && (rx_data[0] == 0xFF)) {
        if (len == 2) {
            if (rx_data[1] >= 0 & rx_data[1] < 128) {
                i2c_address[0] = rx_data[1];
                i2c_address_write_to_flash();
                user_i2c_init();
            }
        }
    } else if (len == 1 && (rx_data[0] == 0xFF)) {
        if (i2c_address != 0) i2c1_set_send_data(i2c_address, 1);
    } else if (len == 1 && (rx_data[0] == 0xFE)) {
        i2c1_set_send_data((uint8_t *)&fm_version, 1);
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    // MX_I2C1_Init();
    MX_TIM16_Init();
    /* USER CODE BEGIN 2 */
    i2c_address_read_from_flash();
    user_i2c_init();
    SteerInit();
    HAL_I2C_EnableListen_IT(&hi2c1);
    sk6812_init(TOTAL_RGB);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct   = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct   = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV          = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
