/**
 * @file    main.h
 * @brief   Flight controller global constants and shared declarations.
 */
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

void esc_init_sequence(void);
void esc_start_signal(void);
void MPU6050_Calibrate_Gyro(void);

#define GYRO_SCALE_FACTOR       16.4f                   // 对应±2000dps量程

#define DEG_TO_RAD              0.017453292519943f      // 度到弧度的转换系数

#define INNER_LOOP_FREQ         200                     // 角速度内环频率 200Hz
#define OUTER_LOOP_FREQ         100                     // 角度外环频率 100Hz
#define INNER_LOOP_DT           (1.0f / INNER_LOOP_FREQ) // dt = 0.005s
#define OUTER_LOOP_DT           (1.0f / OUTER_LOOP_FREQ) // dt = 0.01s

// 初始化陀螺仪低通滤波器参数 = MPU6050 输出频率 500 Hz
#define GYRO_LPF_CUTOFF_HZ      35.0f                   // F330 机架推荐值
#define GYRO_SAMPLE_RATE        500.0f                  // 与 MPU6050_Set_Rate(500) 对应

// 滤波后数据缓冲区（供全局变量，在 main.c 中定义）
extern float gyro_x_dps;
extern float gyro_y_dps;
extern float gyro_z_dps;

#define PITCH_ANGLE_FAILSAFE_DEG    45.0f
#define PITCH_RATE_FAILSAFE_DPS     150.0f



#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
