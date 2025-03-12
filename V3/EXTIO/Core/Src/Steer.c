#include "Steer.h"
#include "tim.h"
#include "i2c_ex.h"


#define ANGLE_TO_PULSE(angle) ((uint16_t)(500 + (angle * 2000 / 180)))//Angle convert to pulse
#define PULSE_TO_COUNT(pulse) ((uint8_t)(pulse)) // Pulse convert to percent
#define PULSE_TO_ANGLE(pulse) ((uint8_t)((pulse - 500) * 180 / 2000)) // Pulse convert to angle

//volatile uint8_t steer_tim_count[STEER_NUMBER];
extern volatile uint16_t steer_pulse[];
extern volatile uint8_t steer_angle[];
extern volatile uint8_t steer_disable[];
extern uint16_t steer_count;
extern uint16_t pwm_pulse[];
extern uint8_t pwm_disable[];
uint16_t steer_count = 0;//1us count

uint8_t steer_angle_last[STEER_NUMBER];
uint16_t steer_pulse_last[STEER_NUMBER];



/**
  * @brief  Update duty cycle.
  * @param  Which output.
  * @param  Set duty cycle.
  */
// static void UpdateTimPulse(uint8_t pos, uint16_t pulse)
// {
//   steer_pulse[pos] = pulse;
// }
/**
  * @brief  Use the GPIO port output to simulate PWM.
  * @param  PWM port.
  */
// void Analog_PWM(void)
// {
// 	static char Switch[STEER_NUMBER]={0};
// 	for(int i=0; i<STEER_NUMBER; i++)
// 	{
// 		if(steer_count <= steer_pulse[i] && Switch[i]==0 && steer_disable[i] == 0)
// 		{
//       GPIOA->BSRR = steering_gear[i].GPIO_Pin;
// 			Switch[i]=1;
// 		}
// 		else if(steer_count > steer_pulse[i] && Switch[i]==1)
// 		{
//       GPIOA->BRR = steering_gear[i].GPIO_Pin;
// 			Switch[i]=0;
// 		}
// 	}
// }
/**
  * @brief  Steer initialization
  */
void SteerInit(void)
{
  for(uint8_t i = 0; i < STEER_NUMBER; i++)
  {
		steer_angle[i] = 0xff;
    steer_angle_last[i] = 0xff;
		steer_pulse[i] = 0xffff;
		steer_pulse_last[i] = 0xffff;
		steer_disable[i] = 0xff;
  }
}
/**
  * @brief  Set Steer angle.
  * @param  Which output.
  */
void SetSteerAngle(uint8_t pos, uint8_t angle) 
{
  if(angle > 180) return;
  steer_angle[pos] = angle;
  steer_pulse[pos] = ANGLE_TO_PULSE(angle);
}
/**
  * @brief  Set Steer Pulse.
  * @param  Which output.
  */
void SetSteerPulse(uint8_t pos, uint16_t pulse)
{
  if(pulse > 2500) return;
  if(pulse < 500) return;
  steer_pulse[pos] = pulse;  
}
/**
  * @brief  Set Steer Pulse.
  * @param  Which output.
  */
void SetPWMPulse(uint8_t pos, uint16_t pulse)
{
  if(pulse > 100) return;
  pwm_pulse[pos] = pulse*pwm_correct[freq_set_index];
}
/**
  * @brief  Update related parameters.
  */
void SteerUpdate(void)
{
  for(uint8_t i = 0; i < STEER_NUMBER; i++) {
    if(steer_angle[i] != steer_angle_last[i]) {
      SetSteerAngle(i, steer_angle[i]);
      steer_angle_last[i] = steer_angle[i];
      steer_pulse_last[i] = steer_pulse[i];
    }

    if(steer_pulse[i] != steer_pulse_last[i]) {
      SetSteerPulse(i, steer_pulse[i]);
      steer_angle_last[i] = steer_angle[i];
      steer_pulse_last[i] = steer_pulse[i];
    }
  }
}
/**
  * @brief  Rewrite the callback function.
  * @param  Callback function parameters.
  */

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
// 	if(htim->Instance==htim16.Instance)
// 	{
// 		if(++steer_count==2000)steer_count=0;
// 		Analog_PWM();
// 	}
// }
