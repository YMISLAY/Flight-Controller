#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

// 电机初始化
void Motor_Init(void);

// 设置四个电机输出值 (0-1000)
void Motor_SetOutput(float motor_outputs[4]);

// 设置单个电机输出值 (0-1000)
void Motor_SetSingleOutput(uint8_t motor_id, float output);

#endif
