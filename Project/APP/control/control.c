/**
 * @file    control.c
 * @brief   Flight controller — cascaded angle/rate PID loops and X-mixer.
 *
 * Loop rates:
 *   Outer (angle → rate target):  100 Hz  (every other TIM4 interrupt)
 *   Inner (rate  → motor output): 200 Hz  (every TIM4 interrupt)
 */

#include "control.h"
#include "motor.h"
#include "usart.h"

// ==================== PID 参数 ====================

//PID参数
#define PITCH_ANGLE_KP  2.0f
#define PITCH_ANGLE_KI  0.0f
#define PITCH_ANGLE_KD  0.0f

#define PITCH_RATE_KP   0.4f
#define PITCH_RATE_KI   0.08f
#define PITCH_RATE_KD   0.003f

#define ROLL_ANGLE_KP   2.0f
#define ROLL_ANGLE_KI   0.0f
#define ROLL_ANGLE_KD   0.0f

#define ROLL_RATE_KP    0.4f
#define ROLL_RATE_KI    0.08f
#define ROLL_RATE_KD    0.003f

#define YAW_ANGLE_KP    2.0f
#define YAW_ANGLE_KI    0.0f
#define YAW_ANGLE_KD    0.0f

#define YAW_RATE_KP     0.8f
#define YAW_RATE_KI     0.15f
#define YAW_RATE_KD     0.0f

// Pitch 前后电机差动值（用于调试）
int16_t motor_pitch_diff = 0;

// ==================== 内部工具函数 ====================

// 数值限幅函数
static float constrain_float(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

/*
 * 斜率限制函数
 * target:    期望目标值
 * current:   当前已经逼近的目标值
 * max_step:  单次调节允许的最大变化步长
 * 返回：     限制变化率后的目标值
 * 用途：
 *     限制 Pitch 目标角速度变化率，
 *     避免外环输出角速度目标突然阶跃。
 */
static float slew_limit_float(float target, float current, float max_step)
{
    float diff = target - current;
    if (diff > max_step) {
        diff = max_step;
    } else if (diff < -max_step) {
        diff = -max_step;
    }
    return current + diff;
}

// ==================== 飞控初始化 ====================
void FlightController_Init(FlightController* fc)
{
    if (fc == NULL) {
        return;
    }
    /*
     * 初始化角度环 PID
     * Pitch 角度环输出，单位是 deg/s，
     * Pitch 角度环输出限幅建议先小一些，
     * 逐步放开，上限用 MAX_PITCH_RATE_DPS 控制。
     * 变量 pitch_rate_target_raw 能反映外环原始输出值。
     */
    PID_Init(&fc->roll_angle_pid,
             ROLL_ANGLE_KP,
             ROLL_ANGLE_KI,
             ROLL_ANGLE_KD,
             100.0f,
             80.0f);

    PID_Init(&fc->pitch_angle_pid,
             PITCH_ANGLE_KP,
             PITCH_ANGLE_KI,
             PITCH_ANGLE_KD,
             100.0f,
             80.0f);

    PID_Init(&fc->yaw_angle_pid,
             YAW_ANGLE_KP,
             YAW_ANGLE_KI,
             YAW_ANGLE_KD,
             100.0f,
             30.0f);

    /*
     * 初始化角速度环 PID
     */
    PID_Init(&fc->roll_rate_pid,
             ROLL_RATE_KP,
             ROLL_RATE_KI,
             ROLL_RATE_KD,
             100.0f,
             160.0f);

    PID_Init(&fc->pitch_rate_pid,
             PITCH_RATE_KP,
             PITCH_RATE_KI,
             PITCH_RATE_KD,
             100.0f,
             160.0f);

    PID_Init(&fc->yaw_rate_pid,
             YAW_RATE_KP,
             YAW_RATE_KI,
             YAW_RATE_KD,
             100.0f,
             100.0f);

    fc->throttle = 0.0f;
    fc->pitch_angle_error = 0.0f;
    fc->pitch_rate_target_raw = 0.0f;
    fc->pitch_rate_target_limited = 0.0f;
    fc->pitch_rate_target_out = 0.0f;
    fc->pitch_rate_output = 0.0f;
    for (int i = 0; i < 4; i++) {
        fc->motor_outputs[i] = 0.0f;
    }
    motor_pitch_diff = 0;
}

// ==================== PID 复位 ====================
void FlightController_ResetPID(FlightController* fc)
{
    if (fc == NULL) {
        return;
    }
    PID_Reset(&fc->roll_angle_pid);
    PID_Reset(&fc->pitch_angle_pid);
    PID_Reset(&fc->yaw_angle_pid);
    PID_Reset(&fc->roll_rate_pid);
    PID_Reset(&fc->pitch_rate_pid);
    PID_Reset(&fc->yaw_rate_pid);
    /*
     * 复位外环 / 斜率 / 内环状态，
     * 主要是防止：
     * 停机再解锁定高时，残留 pitch_rate_target_out，导致斜率状态被保持成目标角速度。
     */
    fc->pitch_angle_error = 0.0f;
    fc->pitch_rate_target_raw = 0.0f;
    fc->pitch_rate_target_limited = 0.0f;
    fc->pitch_rate_target_out = 0.0f;
    fc->pitch_rate_output = 0.0f;
    motor_pitch_diff = 0;
}

// ==================== 停机控制 ====================
void FlightController_Stop(FlightController* fc)
{
    if (fc == NULL) {
        return;
    }
    FlightController_ResetPID(fc);
    fc->throttle = 0.0f;
    for (int i = 0; i < 4; i++) {
        fc->motor_outputs[i] = 0.0f;
    }
    motor_pitch_diff = 0;
    /*
     * 此处仅 Motor_SetOutput 清空软件变量，
     * main.c 停机分支仍然自己调用 TIM5_SetCompareX(500)
     * 以确保 PWM 输出硬件停止值。
     */
    Motor_SetOutput(fc->motor_outputs);
}

// ========== 外环：角度 → 角速度目标（100Hz） ==========
void FlightController_Update_Outer(FlightController* fc, DroneAttitude* attitude,
                                   float target_roll,
                                   float target_pitch, float target_yaw, float dt)
{
    if (fc == NULL || attitude == 0 || dt <= 0.0f) return;

#if PITCH_RATE_TEST_MODE
    fc->pitch_angle_error = 0.0f;
    fc->pitch_rate_target_raw = target_pitch * ANGLE_TO_RATE_COEFF;
#else
    fc->pitch_angle_error = target_pitch - attitude->pitch;
    fc->pitch_rate_target_raw = PID_Update(&fc->pitch_angle_pid,
                                           target_pitch,
                                           attitude->pitch,
                                           dt);
#endif
    // 限幅
    fc->pitch_rate_target_limited = constrain_float(fc->pitch_rate_target_raw,
                                                    -MAX_PITCH_RATE_DPS,
                                                     MAX_PITCH_RATE_DPS);
    // 斜率限制
    {
        float max_step = PITCH_RATE_TARGET_SLEW_DPS2 * dt;
        fc->pitch_rate_target_out = slew_limit_float(fc->pitch_rate_target_limited,
                                                     fc->pitch_rate_target_out,
                                                     max_step);
    }
    // ===== Roll 外环（暂未调试）=====
    PID_Update(&fc->roll_angle_pid, target_roll, attitude->roll, dt);
    // Yaw 外环（目前输出为0）
    PID_Update(&fc->yaw_angle_pid, target_yaw, attitude->yaw, dt);
}

// ========== 内环：角速度 → 电机输出（200Hz） ==========
void FlightController_Update_Inner(FlightController* fc, DroneAttitude* attitude, float dt)
{
    if (fc == NULL || attitude == 0 || dt <= 0.0f) return;

    // Roll 角速度目标 = Roll 外环 PID 输出
    float roll_rate_target = fc->roll_angle_pid.output;
    // Yaw 外环输出（暂时用了外环不要动）
    float yaw_rate_target = fc->yaw_angle_pid.output;
    // Roll / Pitch / Yaw 角速度内环
    float roll_output = PID_Update_D_On_Measurement(&fc->roll_rate_pid,
                                                    roll_rate_target,
                                                    attitude->roll_rate,
                                                    dt);
    float pitch_output = PID_Update_D_On_Measurement(&fc->pitch_rate_pid,
                                                     fc->pitch_rate_target_out,
                                                     attitude->pitch_rate,
                                                     dt);
    float yaw_output = PID_Update_D_On_Measurement(&fc->yaw_rate_pid,
                                                   yaw_rate_target,
                                                   attitude->yaw_rate,
                                                   dt);
    fc->pitch_rate_output = pitch_output;
    // X 型混控
    fc->motor_outputs[0] = fc->throttle + pitch_output - roll_output + yaw_output;
    fc->motor_outputs[1] = fc->throttle + pitch_output + roll_output - yaw_output;
    fc->motor_outputs[2] = fc->throttle - pitch_output - roll_output - yaw_output;
    fc->motor_outputs[3] = fc->throttle - pitch_output + roll_output + yaw_output;

    for (int i = 0; i < 4; i++) {
        fc->motor_outputs[i] = constrain_float(fc->motor_outputs[i], 0.0f, 1000.0f);
    }
    motor_pitch_diff = (int16_t)((fc->motor_outputs[0] + fc->motor_outputs[1]
                                  - fc->motor_outputs[2] - fc->motor_outputs[3]) / 2.0f);
    Motor_SetOutput(fc->motor_outputs);
}
