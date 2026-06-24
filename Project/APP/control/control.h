/**
 * @file    control.h
 * @brief   Flight controller — cascaded angle/rate loops for a quadcopter.
 *
 * Architecture:
 *   Outer loop (100 Hz):  target angle  →  target angular rate
 *   Inner loop (200 Hz):  target angular rate  →  motor outputs
 *   Mixer: X-configuration quadcopter
 *
 * Motor layout (X quad):
 *         1       2
 *           \   /
 *            \ /
 *            / \
 *           /   \
 *         3       4
 *   1: front-right (motor_outputs[0])   2: front-left (motor_outputs[1])
 *   3: rear-right  (motor_outputs[2])   4: rear-left  (motor_outputs[3])
 */


#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include <stddef.h>
#include "pid.h"
#include "mpu6050.h"

/*
 * 调试模式开关
 * 1: Pitch 角速度内环直驱模式
 *    摇杆的 target_pitch 乘以 ANGLE_TO_RATE_COEFF，
 *    直接变成 Pitch 目标角速度。
 * 0: 标准角度外环模式
 *    摇杆的 target_pitch 作为 Pitch 目标角度，
 *    Pitch 角度外环算出 Pitch 目标角速度。
 */
#define PITCH_RATE_TEST_MODE    0

/*
 * 角速度直驱模式（调试）使用：
 * 例如：
 * 摇杆的 target_pitch 约 ±30°，
 * ANGLE_TO_RATE_COEFF = 3.0，
 * 则目标角速度约 ±90°/s。
 * 在角度外环模式下不直接使用该系数。
 */
#define ANGLE_TO_RATE_COEFF     3.0f
/*
 * Pitch 外环输出的目标角速度限幅，单位：deg/s
 * 建议在至 ±75～±90。
 */
#define MAX_PITCH_RATE_DPS      75.0f

/*
 * Pitch 目标角速度斜率限制，单位：deg/s^2
 * 作用：
 * 限制 Pitch 进入角速度内环的目标角速度变化率，
 * 避免外环输出突然阶跃造成小台架甩尾或机械振荡。
 * 例如：
 * 假设 dt = 0.005s，
 * PITCH_RATE_TARGET_SLEW_DPS2 = 400 deg/s^2，
 * 则每周期目标角速度最多变化 2 deg/s。
 */
#define PITCH_RATE_TARGET_SLEW_DPS2   400.0f
// 无人机姿态数据结构
typedef struct {
    float roll;          // 滚转角，单位：度
    float pitch;         // 俯仰角，单位：度
    float yaw;           // 偏航角，单位：度
    float roll_rate;     // 滚转角速度，单位：度/秒
    float pitch_rate;    // 俯仰角速度，单位：度/秒
    float yaw_rate;      // 偏航角速度，单位：度/秒
} DroneAttitude;

// 飞控器结构
typedef struct FlightController {
    PID_Controller roll_angle_pid;
    PID_Controller pitch_angle_pid;
    PID_Controller yaw_angle_pid;
    PID_Controller roll_rate_pid;
    PID_Controller pitch_rate_pid;
    PID_Controller yaw_rate_pid;
    float throttle;
    float motor_outputs[4];
    /*
     * Pitch 外环 / 内环调试变量
     * 这些变量主要用于 main.c 中发送调试数据。
     */
    // Pitch 角度误差：target_pitch - current_pitch，单位：deg
    float pitch_angle_error;
    // Pitch 外环原始输出的目标角速度，限幅前，单位：deg/s
    float pitch_rate_target_raw;
    // Pitch 外环限幅后的目标角速度，斜率限制前，单位：deg/s
    float pitch_rate_target_limited;
    // 最终进入 Pitch 角速度内环的目标角速度，斜率限制后，单位：deg/s
    float pitch_rate_target_out;
    // Pitch 角速度内环输出
    float pitch_rate_output;
} FlightController;
// Pitch 前后电机差动值（用于调试）
extern int16_t motor_pitch_diff;
void FlightController_Init(FlightController* fc);
/*
 * 复位飞控所有 PID
 * 停机或解锁下降时调用。
 */
void FlightController_ResetPID(FlightController* fc);
/*
 * 停机控制
 * 复位 PID，清空输出。
 * 注意：此处负责写 500us 停止 PWM，由 main.c 中 TIM5_SetCompare 控制。
 */
void FlightController_Stop(FlightController* fc);
/*
 * 外环更新，100Hz，角度误差 → 角速度目标
 */
void FlightController_Update_Outer(FlightController* fc,
                                   DroneAttitude* attitude,
                                   float target_roll,
                                   float target_pitch,
                                   float target_yaw,
                                   float dt);
/*
 * 内环更新，200Hz，角速度误差 → 电机输出
 */
void FlightController_Update_Inner(FlightController* fc,
                                   DroneAttitude* attitude,
                                   float dt);

#endif
