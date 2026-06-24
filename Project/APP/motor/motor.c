/**
 * @file    motor.c
 * @brief   Maps 0–1000 motor outputs to ESC PWM (500–2000 µs) via TIM5 CCR.
 */

#include "motor.h"
#include "usart.h"

// PWM参数定义
#define PWM_PERIOD  19999

// 电机实际映射到的PWM范围
#define PWM_MIN     500     // 电机停止（校准最小值）
#define PWM_MAX     1999    // 电机最大值（校准最大值）
void Motor_Init(void)
{
    // 初始时所有电机为停止状态
    float initial_outputs[4] = {0, 0, 0, 0};
    Motor_SetOutput(initial_outputs);
}

//uint32_t print_count = 0;
void Motor_SetOutput(float motor_outputs[4])
{
    // 将0-1000的输入值映射到PWM占空比
    // 使用线性映射：0->PWM_MIN, 1000->PWM_MAX
    uint32_t pwm1 = (uint32_t)(PWM_MIN + (motor_outputs[0] / 1000.0f) * (PWM_MAX - PWM_MIN));
    uint32_t pwm2 = (uint32_t)(PWM_MIN + (motor_outputs[1] / 1000.0f) * (PWM_MAX - PWM_MIN));
    uint32_t pwm3 = (uint32_t)(PWM_MIN + (motor_outputs[2] / 1000.0f) * (PWM_MAX - PWM_MIN));
    uint32_t pwm4 = (uint32_t)(PWM_MIN + (motor_outputs[3] / 1000.0f) * (PWM_MAX - PWM_MIN));
    // 限制PWM值在有效范围内
    pwm1 = (pwm1 > PWM_MAX) ? PWM_MAX : pwm1;
    pwm2 = (pwm2 > PWM_MAX) ? PWM_MAX : pwm2;
    pwm3 = (pwm3 > PWM_MAX) ? PWM_MAX : pwm3;
    pwm4 = (pwm4 > PWM_MAX) ? PWM_MAX : pwm4;
	
    pwm1 = (pwm1 < PWM_MIN) ? PWM_MIN : pwm1;
    pwm2 = (pwm2 < PWM_MIN) ? PWM_MIN : pwm2;
    pwm3 = (pwm3 < PWM_MIN) ? PWM_MIN : pwm3;
    pwm4 = (pwm4 < PWM_MIN) ? PWM_MIN : pwm4;

    // 直接设置比较寄存器值
    TIM5->CCR1 = pwm1;
    TIM5->CCR2 = pwm2;
    TIM5->CCR3 = pwm3;
    TIM5->CCR4 = pwm4;
}

void Motor_SetSingleOutput(uint8_t motor_id, float output)
{
    // 将0-1000的输入值映射到PWM占空比
    uint32_t pwm = (uint32_t)(PWM_MIN + (output / 1000.0f) * (PWM_MAX - PWM_MIN));
    // 限制PWM值在有效范围内
    pwm = (pwm > PWM_MAX) ? PWM_MAX : pwm;
    pwm = (pwm < PWM_MIN) ? PWM_MIN : pwm;

    // 设置指定电机的PWM值
    switch (motor_id) {
        case 0:
            TIM5->CCR1 = pwm;
            break;
        case 1:
            TIM5->CCR2 = pwm;
            break;
        case 2:
            TIM5->CCR3 = pwm;
            break;
        case 3:
            TIM5->CCR4 = pwm;
            break;
        default:
            break;
    }
}
