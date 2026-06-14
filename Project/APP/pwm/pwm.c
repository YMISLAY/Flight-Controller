#include "pwm.h"

TIM_HandleTypeDef TIM5_Handler;          // 定时器5句柄
TIM_OC_InitTypeDef TIM5_CH1Handler;      // 定时器5通道1配置
TIM_OC_InitTypeDef TIM5_CH2Handler;      // 定时器5通道2配置
TIM_OC_InitTypeDef TIM5_CH3Handler;      // 定时器5通道3配置
TIM_OC_InitTypeDef TIM5_CH4Handler;      // 定时器5通道4配置

/*******************************************************************************
* 函 数 名         : TIM5_CH1_PWM_Init
* 函数功能         : TIM5通道1-4 PWM初始化函数
* 输    入         : per:重装载值
                     psc:分频系数
* 输    出         : 无
*******************************************************************************/
void TIM5_CH1_PWM_Init(u32 per, u16 psc)
{
    TIM5_Handler.Instance = TIM5;                   // 定时器5
    TIM5_Handler.Init.Prescaler = psc;              // 定时器分频
    TIM5_Handler.Init.CounterMode = TIM_COUNTERMODE_UP; // 向上计数模式
    TIM5_Handler.Init.Period = per;                 // 自动重装载值
    TIM5_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM5_Handler);                // 初始化PWM

    TIM5_CH1Handler.OCMode = TIM_OCMODE_PWM1;       // 模式选择PWM1
    TIM5_CH1Handler.Pulse = 0;                      // 设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM5_CH1Handler.OCPolarity = TIM_OCPOLARITY_HIGH; // 输出比较极性为高

    TIM5_CH2Handler.OCMode = TIM_OCMODE_PWM1;       // 模式选择PWM1
    TIM5_CH2Handler.Pulse = 0;                      // 设置比较值
    TIM5_CH2Handler.OCPolarity = TIM_OCPOLARITY_HIGH;

    TIM5_CH3Handler.OCMode = TIM_OCMODE_PWM1;       // 模式选择PWM1
    TIM5_CH3Handler.Pulse = 0;                      // 设置比较值
    TIM5_CH3Handler.OCPolarity = TIM_OCPOLARITY_HIGH;

    TIM5_CH4Handler.OCMode = TIM_OCMODE_PWM1;       // 模式选择PWM1
    TIM5_CH4Handler.Pulse = 0;                      // 设置比较值
    TIM5_CH4Handler.OCPolarity = TIM_OCPOLARITY_HIGH;

    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler, &TIM5_CH1Handler, TIM_CHANNEL_1); // 配置TIM5通道1
    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler, &TIM5_CH2Handler, TIM_CHANNEL_2); // 配置TIM5通道2
    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler, &TIM5_CH3Handler, TIM_CHANNEL_3); // 配置TIM5通道3
    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler, &TIM5_CH4Handler, TIM_CHANNEL_4); // 配置TIM5通道4

    HAL_TIM_PWM_Start(&TIM5_Handler, TIM_CHANNEL_1); // 开启PWM通道1
    HAL_TIM_PWM_Start(&TIM5_Handler, TIM_CHANNEL_2); // 开启PWM通道2
    HAL_TIM_PWM_Start(&TIM5_Handler, TIM_CHANNEL_3); // 开启PWM通道3
    HAL_TIM_PWM_Start(&TIM5_Handler, TIM_CHANNEL_4); // 开启PWM通道4
}

// 定时器底层驱动，时钟使能，引脚配置
// 此函数会被HAL_TIM_PWM_Init()调用
// htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_TIM5_CLK_ENABLE();            // 使能定时器5
    __HAL_RCC_GPIOA_CLK_ENABLE();           // 开启GPIOA时钟

    GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;   // PA0-3
    GPIO_Initure.Mode = GPIO_MODE_AF_PP;    // 复用推挽输出
    GPIO_Initure.Pull = GPIO_NOPULL;        // 浮空
    GPIO_Initure.Speed = GPIO_SPEED_LOW;    // 低速
    GPIO_Initure.Alternate = GPIO_AF2_TIM5; // PA0-3复用为TIM5_CH1-4
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);
}

// 设置TIM通道的占空比
void TIM5_SetCompare1(u32 compare)
{
    TIM5->CCR1 = compare;
}

void TIM5_SetCompare2(u32 compare)
{
    TIM5->CCR2 = compare;
}

void TIM5_SetCompare3(u32 compare)
{
    TIM5->CCR3 = compare;
}

void TIM5_SetCompare4(u32 compare)
{
    TIM5->CCR4 = compare;
}
