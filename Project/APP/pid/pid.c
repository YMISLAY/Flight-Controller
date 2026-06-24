/**
 * @file    pid.c
 * @brief   PID controller implementation with two D-term strategies.
 */
 
#include "pid.h"
#include <math.h>
#include "usart.h"

void PID_Init(PID_Controller* pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = 0;
    pid->output = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;

    pid->d_filtered = 0.0f;   // 滤波
    pid->max_output = 0.0f;
    pid->max_error = 0.0f;
    pid->reset_time = 0;
}

/*
 * 普通 PID，D 对 error 微分
 * 用于角度外环即可
 */
float PID_Update(PID_Controller* pid,
                 float setpoint,
                 float measurement,
                 float dt)
{
    if (dt <= 0.0f) {
        return pid->output;
    }
    float error = setpoint - measurement;
    // P
    float proportional = pid->kp * error;
    pid->p_out = proportional;
    // I
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float integral = pid->ki * pid->integral;
    pid->i_out = integral;

    // D on error
    float derivative = 0.0f;
    if (!pid->initialized) {
        pid->prev_error = error;
        pid->prev_measurement = measurement;
        pid->initialized = 1;
        derivative = 0.0f;
    } else {
        derivative = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->d_out = derivative;
    pid->mes = error;
    pid->premes = pid->prev_error;
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    // Output
    pid->output = proportional + integral + derivative;
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    // 记录峰值
    if (fabsf(pid->output) > fabsf(pid->max_output)) {
        pid->max_output = pid->output;
    }
    if (fabsf(error) > fabsf(pid->max_error)) {
        pid->max_error = error;
    }

    return pid->output;
}

/*
 * D on measurement PID
 * error = setpoint - measurement
 * P = kp * error
 * I = ki * integral(error)
 * D = -kd * d(measurement)/dt
 * 好处：
 * 目标值 setpoint 阶跃变化时，不会直接冲击 D 输出
 * 推荐用于角速度环
 */
float PID_Update_D_On_Measurement(PID_Controller* pid,
                                  float setpoint,
                                  float measurement,
                                  float dt)
{
    if (dt <= 0.0f) {
        return pid->output;
    }

    float error = setpoint - measurement;

    // P
    float proportional = pid->kp * error;
    pid->p_out = proportional;

    // I
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float integral = pid->ki * pid->integral;
    pid->i_out = integral;

    // D on measurement（跳过第一次初始化跳变）
    float derivative = 0.0f;
    if (!pid->initialized) {
        pid->prev_measurement = measurement;
        pid->prev_error = error;
        pid->initialized = 1;
        derivative = 0.0f;
    } else {
        derivative = -pid->kd * (measurement - pid->prev_measurement) / dt;
    // D 一阶低通滤波，截止约20Hz@100Hz
    pid->d_filtered = 0.12f * derivative + 0.88f * pid->d_filtered;
    derivative = pid->d_filtered;
    }
    pid->d_out = derivative;
    pid->prev_measurement = measurement;
    pid->prev_error = error;
    // Output
    pid->output = proportional + integral + derivative;
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    // 记录峰值
    if (fabsf(pid->output) > fabsf(pid->max_output)) {
        pid->max_output = pid->output;
    }
    if (fabsf(error) > fabsf(pid->max_error)) {
        pid->max_error = error;
    }
    return pid->output;
}

/*
 * 复位所有 PID
 * 停机、解锁下降或切换模式时使用
 */
void PID_Reset(PID_Controller* pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = 0;
    pid->output = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;

    pid->d_filtered = 0.0f;   // 滤波
    pid->max_output = 0.0f;
    pid->max_error = 0.0f;
}

/*
 * 打印并重置峰值记录
 */
void PID_ResetAndPrint(PID_Controller* pid, const char* name)
{
    printf("%s - MaxOutput: %.1f, MaxError: %.1f\n",
           name,
           pid->max_output,
           pid->max_error);
    pid->max_output = 0.0f;
    pid->max_error = 0.0f;
}
