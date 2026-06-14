#ifndef __TIM_H
#define __TIM_H

#include "stm32f4xx_hal.h"
#include "control.h"

// TIM4 初始化函数
// arr: 自动重装值
// psc: 预分频系数
void TIM4_Init(uint32_t arr, uint16_t psc);

// TIM4 中断服务函数
void TIM4_IRQHandler(void);

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
