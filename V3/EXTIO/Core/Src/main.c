/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_ex.h"
#include "flash.h"
#include "Steer.h"
#include "ws2812.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin;
} PWM_GPIO;

PWM_GPIO steering_gear[STEER_NUMBER]={
	
	{Steer_0_GPIO_Port,Steer_0_Pin},	
	{Steer_1_GPIO_Port,Steer_1_Pin},
	{Steer_2_GPIO_Port,Steer_2_Pin},
	{Steer_3_GPIO_Port,Steer_3_Pin},
	{Steer_4_GPIO_Port,Steer_4_Pin},
	{Steer_5_GPIO_Port,Steer_5_Pin},
	{Steer_6_GPIO_Port,Steer_6_Pin},
	{Steer_7_GPIO_Port,Steer_7_Pin},
		
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS 0x45
#define FIRMWARE_VERSION 3
#define FLASH_DATA_SIZE 32

#define APPLICATION_ADDRESS     ((uint32_t)0x08001000)

#define INPUT_MODE 0
#define OUTPUT_MODE 1
#define ADC_MODE 2
#define SERVO_MODE 3
#define LED_MODE 4
#define PWM_MODE 5

#define PWM_DISABLE 0
#define PWM_SERVO_MODE 1
#define PWM_PWM_MODE 2

#define IO0_GPIO GPIOA,GPIO_PIN_0
#define IO1_GPIO GPIOA,GPIO_PIN_1
#define IO2_GPIO GPIOA,GPIO_PIN_2
#define IO3_GPIO GPIOA,GPIO_PIN_3

#define IO4_GPIO GPIOA,GPIO_PIN_4
#define IO5_GPIO GPIOA,GPIO_PIN_5
#define IO6_GPIO GPIOA,GPIO_PIN_6
#define IO7_GPIO GPIOA,GPIO_PIN_7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// i2c address
uint8_t i2c_address[1] = {0};

// flash data
uint8_t flash_data[FLASH_DATA_SIZE] = {0};

volatile uint16_t steer_pulse[STEER_NUMBER];
volatile uint8_t steer_angle[STEER_NUMBER];
volatile uint8_t steer_disable[STEER_NUMBER];
uint16_t pwm_pulse[STEER_NUMBER];
uint8_t pwm_disable[STEER_NUMBER];
volatile uint16_t neopixel_pin = 0;
volatile uint8_t color_index = 0;
uint8_t io_mode_save[8] = {0};
volatile uint8_t flag_pwm_mode = 0;
volatile uint8_t flag_servo_mode = 0;
static char Switch[STEER_NUMBER]={0};
volatile uint8_t fm_version = FIRMWARE_VERSION;

volatile uint8_t input_state = 0;
uint8_t input_state_set[8] = {0};

static uint8_t i2c_address_change_flag = 0;

static uint8_t led_change_flag = 0;

// i2c stop timeout delay
static volatile uint32_t i2c_stop_timeout_delay = 0;

// pwm freq, 0:2KHz,1:1KHz,2:500Hz,3:250Hz,4:125Hz
uint16_t freq_set[5] = {500, 1000, 2000, 4000, 8000};
uint8_t pwm_correct[5] = {5, 10, 20, 40, 80};

volatile uint8_t freq_set_index = 1;

volatile uint8_t iap_jump_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//????????SRAM???
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //??? SRAM ??? 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

static void init_flash_data(void) 
{   
  uint32_t flash_write_timeout = 0;

  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    i2c_address[0] = I2C_ADDRESS;
    flash_data[0] = i2c_address[0];

    // retry 20 times, if still fail, fast flash 10 times, reset system
    while(!writeMessageToFlash(flash_data , FLASH_DATA_SIZE)) {
      flash_write_timeout++;
      if (flash_write_timeout > 20) {        
        HAL_NVIC_SystemReset();
      }
    }
  } else {
    i2c_address[0] = flash_data[0];
  }
}

uint8_t flash_data_write_back(void)
{
  uint32_t flash_write_timeout = 0;

  // if read falsh ok
  if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
    flash_data[0] = i2c_address[0];
    // retry 20 times, if still fail, return 0
    while(!writeMessageToFlash(flash_data , FLASH_DATA_SIZE)) {
      flash_write_timeout++;
      if (flash_write_timeout > 20) {
        flash_write_timeout = 0;
        return 0;
      }
    }
    // write success, return 1
    return 1;
  }
  else {
    return 0;
  }     
}

void switchInputAndOutput(uint16_t pin, uint32_t type, uint32_t pull_type, uint32_t freq_type)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (type == GPIO_MODE_ANALOG)
      __HAL_RCC_ADC1_CLK_ENABLE();    
    /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                            PAPin PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = type;
    GPIO_InitStruct.Pull = pull_type;
    GPIO_InitStruct.Speed = freq_type;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    if (type == GPIO_MODE_ANALOG) {
      hadc.Instance = ADC1;
      hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
      hadc.Init.Resolution = ADC_RESOLUTION_12B;
      hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
      hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
      hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
      hadc.Init.LowPowerAutoWait = DISABLE;
      hadc.Init.LowPowerAutoPowerOff = DISABLE;
      hadc.Init.ContinuousConvMode = DISABLE;
      hadc.Init.DiscontinuousConvMode = DISABLE;
      hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
      hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
      hadc.Init.DMAContinuousRequests = DISABLE;
      hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
      HAL_ADC_Init(&hadc);      
    }    
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;  
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readADCState(uint32_t channel, uint8_t bit_mode)
{
    uint32_t AD_Value = 0;
    uint32_t Value[22] = {0};
    uint8_t tmp_buff[16] = {0};
    uint16_t tmp_buff_len = 0;
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc.Instance->CHSELR = 0;
    uint32_t max = 0;
    uint32_t min = 0;

    sConfig.Channel = channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    HAL_ADCEx_Calibration_Start(&hadc);

    for(int n=0;n<22;n++)
    {
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 10);
        if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc), HAL_ADC_STATE_REG_EOC))
        {
        	Value[n] = HAL_ADC_GetValue(&hadc);
          AD_Value += Value[n];
        }
    }
    max=Value[0];
    min=Value[0];

    for(uint8_t n=0;n<22;n++)//取最大值、最小值
    {
        max=(Value[n]<max)?max:Value[n];    
        min=(min<Value[n])?min:Value[n];
    }     

    AD_Value = (AD_Value - max - min) / 20;
    
    if (bit_mode == 12) {
      tmp_buff[0] = (uint8_t)AD_Value;
      tmp_buff[1] = (uint8_t)(AD_Value>>8);
      tmp_buff_len = 2;
    } else if (bit_mode == 8) {
      tmp_buff[0] = map(AD_Value,0,4095,0,255);
      tmp_buff_len = 1;
    }
    i2c1_set_send_data(tmp_buff, tmp_buff_len);
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len)
{
  uint8_t buf[32] = {0};

  if(len > 1 && (rx_data[0] <= 0x07))
	{
    uint8_t max_pins = 8 - rx_data[0]; // 剩余可用引脚数
    uint8_t valid_len = MIN(len-1, max_pins);    

    for(int i = 0; i < valid_len; i++) {
      if (rx_data[1+i] <= PWM_MODE) {
        io_mode_save[rx_data[0]+i] = rx_data[1+i];
        switch (rx_data[1+i])
        {
        case INPUT_MODE:
          switchInputAndOutput((uint16_t)(1 << (rx_data[0]+i)), GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
          flag_pwm_mode &= (~(1 << (rx_data[0]+i)));
          flag_servo_mode &= (~(1 << (rx_data[0]+i)));
          // HAL_TIM_Base_Stop_IT(&htim16);
          steer_disable[rx_data[0]+i] = 1;
          pwm_disable[rx_data[0]+i] = 1;
          neopixel_pin = 0;
          break;
        case OUTPUT_MODE:
          switchInputAndOutput((uint16_t)(1 << (rx_data[0]+i)), GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
          flag_pwm_mode &= (~(1 << (rx_data[0]+i)));
          flag_servo_mode &= (~(1 << (rx_data[0]+i)));
          // HAL_TIM_Base_Stop_IT(&htim16);
          steer_disable[rx_data[0]+i] = 1;
          pwm_disable[rx_data[0]+i] = 1;
          neopixel_pin = 0;
          break;
        case ADC_MODE:
          switchInputAndOutput((uint16_t)(1 << (rx_data[0]+i)), GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
          flag_pwm_mode &= (~(1 << (rx_data[0]+i)));
          flag_servo_mode &= (~(1 << (rx_data[0]+i)));
          // HAL_TIM_Base_Stop_IT(&htim16);
          steer_disable[rx_data[0]+i] = 1;
          pwm_disable[rx_data[0]+i] = 1;
          neopixel_pin = 0;
          break;        
        case SERVO_MODE:
          switchInputAndOutput((uint16_t)(1 << (rx_data[0]+i)), GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
          flag_pwm_mode &= (~(1 << (rx_data[0]+i)));
          flag_servo_mode |= (1 << (rx_data[0]+i));
          steer_disable[rx_data[0]+i] = 0;
          pwm_disable[rx_data[0]+i] = 1;
          neopixel_pin = 0;
          break;        
        case LED_MODE:
          // HAL_TIM_Base_Stop_IT(&htim16);
          // flag_pwm_mode = PWM_DISABLE;
          flag_pwm_mode &= (~(1 << (rx_data[0]+i)));
          flag_servo_mode &= (~(1 << (rx_data[0]+i)));
          steer_disable[rx_data[0]+i] = 1;
          pwm_disable[rx_data[0]+i] = 1;
          switchInputAndOutput((uint16_t)(1 << (rx_data[0]+i)), GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);
          break;        
        case PWM_MODE:
          flag_pwm_mode |= (1 << (rx_data[0]+i));
          flag_servo_mode &= (~(1 << (rx_data[0]+i)));
          steer_disable[rx_data[0]+i] = 1;
          pwm_disable[rx_data[0]+i] = 0;
          neopixel_pin = 0;
          switchInputAndOutput((uint16_t)(1 << (rx_data[0]+i)), GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
          break;       
        default:
          break;
        } 
      }      
    }
	}
  else if(len == 1 && ((rx_data[0] >= 0x00) && (rx_data[0] <= 0x07))) {
    memcpy(buf, io_mode_save, 8);
    i2c1_set_send_data((uint8_t *)&buf[rx_data[0]-0x00], 0x07-rx_data[0]+1);
  }
	else if(len > 1 && ((rx_data[0] >= 0x10) && (rx_data[0] <= 0x17))) 
	{
    uint8_t base_pin = rx_data[0] - 0x10;  // 转换为0-7
    
    uint8_t max_pins = 8 - base_pin;       // 剩余可用引脚数
    uint8_t valid_len = MIN(len - 1, max_pins);

    for(int i = 0; i < valid_len; i++) {
      if(io_mode_save[rx_data[0]-0x10+i] == OUTPUT_MODE) {
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
  }
  else if (len > 1 && (rx_data[0] == 0x18)) {
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
  } 
	else if(len == 1 && ((rx_data[0] >= 0x20) && (rx_data[0] <= 0x28))) 
	{
    input_state = 0;
    for (int i = 0; i < 8; i++) {
      input_state_set[i] = HAL_GPIO_ReadPin(GPIOA, (uint16_t)(1 << i));
      input_state |= (input_state_set[i] << i);
    }
    memcpy(buf, input_state_set, 8);
    buf[8] = input_state;
    i2c1_set_send_data((uint8_t *)&buf[rx_data[0]-0x20], 0x28-rx_data[0]+1);
  } 
	else if(len == 1 && ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x37))) 
	{
    uint8_t index = rx_data[0]-0x30;
    if(io_mode_save[index] == ADC_MODE) {
      readADCState(index, 8);
    }
  }
	else if(len == 1 && ((rx_data[0] >= 0x40) && (rx_data[0] <= 0x4F))) 
	{
    uint8_t index = (rx_data[0]-0x40) / 2;
    if (io_mode_save[index] == ADC_MODE) {
      readADCState(index, 12);
    }
  }
	else if(len > 1 && ((rx_data[0] >= 0x50) && (rx_data[0] <= 0x57))) 
	{
    uint8_t base_steer = rx_data[0] - 0x50;   // 转换为0-7的基地址
    uint8_t max_steers = 8 - base_steer;      // 剩余可用舵机数
    uint8_t valid_len = MIN(len - 1, max_steers); // 双重限制  

    for(int i = 0; i < valid_len; i++) {
      SetSteerAngle(rx_data[0]+i - 0x50, rx_data[1+i]);
    }
  }
	else if(len == 1 && ((rx_data[0] >= 0x50) && (rx_data[0] <= 0x57))) 
	{
    memcpy(buf, steer_angle, 8);
    i2c1_set_send_data((uint8_t *)&buf[rx_data[0]-0x50], 0x57-rx_data[0]+1);
  }
	else if(len > 1 && ((rx_data[0] >= 0x60) && (rx_data[0] <= 0x6F))) 
	{
    // 将地址转换为基索引（0x60→0, 0x62→1...）
    uint8_t base_pulse = (rx_data[0] - 0x60) / 2;  
    // 剩余可用舵机数（假设共8个舵机）
    uint8_t max_pulses = 8 - base_pulse;           
    // 有效数据对数（每对数据占2字节）
    uint8_t valid_pairs = (len - 1) / 2;          
    // 双重限制：数据对数 vs 剩余舵机数
    uint8_t valid_len = MIN(valid_pairs, max_pulses);     

    for (int i = 0; i < valid_len; i++) {
      SetSteerPulse(((rx_data[0]+i*2 - 0x60) / 2), ((rx_data[2+i*2] << 8) | (rx_data[1+i*2])));
    }
  }
	else if(len == 1 && ((rx_data[0] >= 0x60) && (rx_data[0] <= 0x6F))) 
	{
    memcpy(buf, (uint8_t *)steer_pulse, 16);
    i2c1_set_send_data((uint8_t *)&buf[rx_data[0]-0x60], 0x6F-rx_data[0]+1);
  }
	else if (len > 1 && ((rx_data[0] >= 0x70) && (rx_data[0] <= 0x87))) 
	{
    uint8_t offset = rx_data[0] - 0x70;
    uint8_t base_led = offset / 3; // 地址转换保护（每3个地址对应1个LED）
    base_led = base_led % 8;                     // 强制映射到8个LED
    
    /* 有效数据包计算 */
    uint8_t max_leds = 8 - base_led;            // 剩余可操作LED数
    uint8_t valid_pairs = (len - 1) / 3;       // 完整RGB数据包数量
    uint8_t valid_len = MIN(valid_pairs, max_leds);

    for (int i = 0; i < valid_len; i++) {
			neopixel_set_color(((rx_data[0]+i*3 - 0x70)/3), ((rx_data[1+i*3] << 24) | (rx_data[2+i*3] << 16) | (rx_data[3+i*3] << 8)));
      led_change_flag = 1;     
    }
  }  
	else if (len == 1 && ((rx_data[0] >= 0x70) && (rx_data[0] <= 0x8F))) 
	{
    for (int i = 0; i < 24; i += 3) {
      buf[i+0] = (color_buf[i/3] >> 8) & 0xff;
      buf[i+1] = (color_buf[i/3] >> 16) & 0xff;
      buf[i+2] = (color_buf[i/3]) & 0xff;
    }
    i2c1_set_send_data((uint8_t *)&buf[rx_data[0]-0x70], 0x8F-rx_data[0]+1);
  }  
	else if (len > 1 && ((rx_data[0] >= 0x90) && (rx_data[0] <= 0x97))) 
	{
    uint8_t base_pwm = rx_data[0] - 0x90;       // 转换为0-7的基索引
    uint8_t max_pins = 8 - base_pwm;            // 剩余可用PWM通道数
    uint8_t valid_len = MIN(len - 1, max_pins); // 双重限制    

    for(int i = 0; i < valid_len; i++) {    
      SetPWMPulse(rx_data[0]+i - 0x90, rx_data[1+i]);
    }    
  }
	else if (len == 1 && ((rx_data[0] >= 0x90) && (rx_data[0] <= 0x97))) 
	{
    for (int i = 0; i < 8; i++) {
      buf[i] = pwm_pulse[i] / pwm_correct[freq_set_index];
    }
    i2c1_set_send_data((uint8_t *)&buf[rx_data[0]-0x90], 0x97-rx_data[0]+1);
  }    
	else if (len > 1 && (rx_data[0] == 0xA0)) 
	{
    if (rx_data[1] < 5) {
      uint8_t pwm_pulse_temp[8] = {0};
      for (int i = 0; i < 8; i++) {
        pwm_pulse_temp[i] = pwm_pulse[i] / pwm_correct[freq_set_index];
      }
      freq_set_index = rx_data[1];
      for (int i = 0; i < 8; i++) {
        pwm_pulse[i] = pwm_pulse_temp[i] * pwm_correct[freq_set_index];
      }
    }
  }       
	else if (len == 1 && (rx_data[0] == 0xA0)) 
	{
    i2c1_set_send_data((uint8_t *)&freq_set_index, 1);
  }       
	else if (len > 1 && (rx_data[0] == 0xFF)) 
	{
    if (len == 2) {
      if (rx_data[1] && (rx_data[1] < 128)) {
        i2c_address[0] = rx_data[1];
        i2c_address_change_flag = 1;
      }
    }
  }  
  else if (len > 1 && rx_data[0] == 0xFD)
  {
    if (rx_data[1] == 1) {
      iap_jump_flag = 1;
    }
  }  
	else if (len == 1 && (rx_data[0] == 0xFF)) 
	{
    if (i2c_address != 0)
      i2c1_set_send_data(i2c_address, 1);
  }  
  else if (len == 1 && (rx_data[0] == 0xFE))
  {
    i2c1_set_send_data((uint8_t *)&fm_version, 1);
  }         
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
  uint32_t flash_writeback_timeout = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  init_flash_data();
  SteerInit();
  sk6812_init(TOTAL_RGB);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);  
  user_i2c_init();
  i2c1_it_enable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (iap_jump_flag) {
      LL_I2C_DeInit(I2C1);
      LL_I2C_DisableAutoEndMode(I2C1);
      LL_I2C_Disable(I2C1);
      LL_I2C_DisableIT_ADDR(I2C1);
      HAL_TIM_Base_DeInit(&htim16);
      HAL_TIM_Base_DeInit(&htim17);
      NVIC_SystemReset();      
      iap_jump_flag = 0;
    }

    i2c_timeout_counter = 0;

    if (flag_pwm_mode != 0) {
      if(TIM16->CNT >= freq_set[freq_set_index])	
      {
        TIM16->CNT = 0;  
      } 
      for(int i=0; i<8; i++)
      {
        i2c_timeout_counter = 0;
        if(pwm_pulse[i] == 0 && pwm_disable[i] == 0)
        {
          GPIOA->BRR = steering_gear[i].GPIO_Pin;
          Switch[i]=0;          
        }
        else if(TIM16->CNT <= pwm_pulse[i] && Switch[i]==0 && pwm_disable[i] == 0)
        {
          GPIOA->BSRR = steering_gear[i].GPIO_Pin;
          Switch[i]=1;
        }
        else if(TIM16->CNT > pwm_pulse[i] && Switch[i]==1 && pwm_disable[i] == 0 && TIM16->CNT <= freq_set[freq_set_index])
        {
          GPIOA->BRR = steering_gear[i].GPIO_Pin;
          Switch[i]=0;
        }
      }
    }
    if (flag_servo_mode != 0) {
      if(TIM17->CNT >= 20000)	
      {
        TIM17->CNT = 0;  
      } 
      for(int i=0; i<8; i++)
      {
        i2c_timeout_counter = 0;
        if(TIM17->CNT <= steer_pulse[i] && Switch[i]==0 && steer_disable[i] == 0)
        {
          GPIOA->BSRR = steering_gear[i].GPIO_Pin;
          Switch[i]=1;
        }
        else if(TIM17->CNT > steer_pulse[i] && Switch[i]==1 && steer_disable[i] == 0)
        {
          GPIOA->BRR = steering_gear[i].GPIO_Pin;
          Switch[i]=0;
        }
      }
    }  
    
    if (i2c_stop_timeout_flag) {
      if (i2c_stop_timeout_delay < HAL_GetTick()) {
        i2c_stop_timeout_counter++;
        i2c_stop_timeout_delay = HAL_GetTick() + 10;
      }
    }

    if (i2c_stop_timeout_counter > 50) {
      LL_I2C_DeInit(I2C1);
      LL_I2C_DisableAutoEndMode(I2C1);
      LL_I2C_Disable(I2C1);
      LL_I2C_DisableIT_ADDR(I2C1);     
      user_i2c_init();    
      i2c1_it_enable();
      HAL_Delay(500);
      i2c_stop_timeout_flag = 0;
      i2c_stop_timeout_counter = 0;
    } 

    if (led_change_flag) {
      for (int i = 0; i < 8; i++) {
        i2c_timeout_counter = 0;
        if(io_mode_save[i] == LED_MODE) {
          neopixel_pin = (uint16_t)(1 << i);
          color_index = i;
          neopixel_show();         
        }
      }      
      led_change_flag = 0;
    }

    if (i2c_address_change_flag) {
      __disable_irq();

      user_i2c_init();

      // retry 20 times, if still fail, reset system
      flash_writeback_timeout = 0;
      while(!flash_data_write_back()) {
        flash_writeback_timeout++;
        if (flash_writeback_timeout > 20) {
          flash_writeback_timeout = 0;        
          HAL_NVIC_SystemReset();
        }
      } 

      __enable_irq();      
      i2c_address_change_flag = 0;
    }       
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  uint32_t clock_setup_timeout = 0;

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
    clock_setup_timeout++;
    if (clock_setup_timeout > 32000) {
      HAL_NVIC_SystemReset();
    }
  }
  LL_RCC_HSI_Enable();

  clock_setup_timeout = 0;
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    clock_setup_timeout++;
    if (clock_setup_timeout > 32000) {
      HAL_NVIC_SystemReset();
    }
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

  clock_setup_timeout = 0;
   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {
    clock_setup_timeout++;
    if (clock_setup_timeout > 32000) {
      HAL_NVIC_SystemReset();
    }
  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

  clock_setup_timeout = 0;
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    clock_setup_timeout++;
    if (clock_setup_timeout > 32000) {
      HAL_NVIC_SystemReset();
    }
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  clock_setup_timeout = 0;
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
    clock_setup_timeout++;
    if (clock_setup_timeout > 32000) {
      HAL_NVIC_SystemReset();
    }
  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
