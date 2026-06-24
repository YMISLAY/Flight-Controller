/**
 * @file    timer.c
 * @brief   TIM4 — 200 Hz interrupt for cascaded flight-control loops.
 */

#include "timer.h"
#include "pwm.h"
#include "control.h"
#include "usart.h"

TIM_HandleTypeDef TIM4_Handler;  // 定时器4句柄
/*******************************************************************************
* 函 数 名         : TIM4_Init
* 函数功能         : TIM4初始化，开启周期定时中断
* 输    入         : per:重装载值
*                   psc:分频系数
* 输    出         : 无
*******************************************************************************/
void TIM4_Init(uint32_t arr, uint16_t psc)
{
    TIM4_Handler.Instance = TIM4;                   // 定时器4
    TIM4_Handler.Init.Prescaler = psc;              // 定时器分频
    TIM4_Handler.Init.CounterMode = TIM_COUNTERMODE_UP; // 向上计数模式
    TIM4_Handler.Init.Period = arr;                 // 自动重装载值
    TIM4_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM4_Handler);               // 初始化定时器

    // 开启定时器中断
    HAL_TIM_Base_Start_IT(&TIM4_Handler);
}

/*******************************************************************************
* 函 数 名         : HAL_TIM_Base_MspInit
* 函数功能         : 定时器底层驱动，时钟使能，引脚配置
*                   此函数会被HAL_TIM_Base_Init()调用
* 输    入         : htim:定时器句柄
* 输    出         : 无
*******************************************************************************/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();                // 使能TIM4时钟
        // 配置NVIC优先级分组
        HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
        // 设置中断优先级：抢占优先级1，响应优先级1
        HAL_NVIC_SetPriority(TIM4_IRQn, 1, 1);
        // 使能TIM4中断
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }
}
