#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

typedef struct {
    float kp;                 // 比例系数
    float ki;                 // 积分系数
    float kd;                 // 微分系数
    float integral;           // 积分累积
    float prev_error;         // 上一次误差
    float prev_measurement;   // 上一次测量值（用于 D on measurement）
    uint8_t initialized;      // 首次初始化标志，防止第一次 D 跳变
    float output;             // PID 输出值
    float integral_limit;     // 积分限幅
    float output_limit;       // 输出限幅

    // 调试字段
    float p_out;              // P 项输出
    float i_out;              // I 项输出
    float d_out;              // D 项输出

    float d_filtered;         // D 项滤波值
    float mes;
    float premes;
    float max_output;         // 峰值输出
    float max_error;          // 峰值误差
    uint32_t reset_time;      // 预留
} PID_Controller;

void PID_Init(PID_Controller* pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_limit);

/*
 * 普通 PID
 * D 对 error 微分
 * 适用于角度外环
 */
float PID_Update(PID_Controller* pid,
                 float setpoint,
                 float measurement,
                 float dt);

/*
 * D on measurement PID
 * D 对 measurement 微分
 * 推荐用于角速度内环，避免目标角速度阶跃时 D 跳变
 */
float PID_Update_D_On_Measurement(PID_Controller* pid,
                                  float setpoint,
                                  float measurement,
                                  float dt);

/*
 * 复位所有 PID
 * 停机、解锁或模式切换时使用
 */
void PID_Reset(PID_Controller* pid);

/*
 * 打印并重置峰值记录
 */
void PID_ResetAndPrint(PID_Controller* pid, const char* name);

#endif
