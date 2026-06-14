#include "system.h"
#include "SysTick.h"
#include "usart.h"
#include "biquad_filter.h"
#include "led.h"
#include "key.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "control.h"
#include "pid.h"
#include "motor.h"
#include "timer.h"
#include "pwm.h"
#include "main.h"
#include "math.h"
#include "ibus.h"
#include <stdio.h>

volatile uint8_t diantiao_flag = 0;        // diantiao_flag为0时停止控制更新，不输出电机信号
volatile uint8_t flight_failsafe = 0;      // pitch角度/角速度超限触发保护
volatile uint8_t esc_output_enable = 0;    // 电调输出使能，MPU校准完成前不向中断写PWM

extern TIM_HandleTypeDef TIM4_Handler;

FlightController flight_controller;     // 飞控器结构实例
DroneAttitude attitude_buffer;          // 无人机姿态数据缓冲区
DroneAttitude current_attitude;         // 无人机当前姿态（飞控中断使用）

BiquadFilter gyro_filter_x, gyro_filter_y, gyro_filter_z; // 陀螺仪低通滤波器实例

// 无人机姿态数据
float pitch, roll, yaw;
short aacx, aacy, aacz;
short gyrox, gyroy, gyroz;

// MPU校准结果
short gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
short acc_bias_x = 0, acc_bias_y = 0, acc_bias_z = 0;
u8 gyro_calibrated = 0;

// MPU陀螺仪滤波后数据，单位 deg/s
float gyro_x_dps;
float gyro_y_dps;
float gyro_z_dps;

/*
 * ==================== ASCII 调试数据发送 ====================
 * 配合 Serial Plot
 * 发送顺序：
 * 1  target_pitch              摇杆 Pitch 目标角度，单位 deg
 * 2  current_pitch             当前 Pitch 实际角度，单位 deg
 * 3  pitch_angle_error         外环角度误差，单位 deg
 * 4  pitch_rate_target_raw     外环原始输出角速度，单位 deg/s
 * 5  pitch_rate_target_out     斜率限制后最终角速度目标，单位 deg/s
 * 6  current_pitch_rate        当前 Pitch 角速度，单位 deg/s
 * 7  pitch_rate_p_out          Pitch 角速度内环 P 项输出
 * 8  pitch_rate_output         Pitch 角速度内环总输出
 * 9  motor_pitch_diff          Pitch 前后差动值
 * 10 throttle                  油门
 *
 * 说明：
 * 配合串口绘图器使用，观察各环节动态变化过程。
 */
#define DEBUG_TX_BUFFER_SIZE     128
#define DEBUG_SEND_INTERVAL_MS   20

typedef struct {
    float target_pitch;              // 摇杆 Pitch 目标角度，单位 deg
    float current_pitch;             // 当前 Pitch 实际角度，单位 deg
    float pitch_angle_error;         // 外环角度误差，单位 deg
    float pitch_rate_target_raw;     // 外环原始输出角速度，单位 deg/s
    float pitch_rate_target_out;     // 斜率限制后的目标角速度，单位 deg/s
    float current_pitch_rate;        // 当前 Pitch 角速度，单位 deg/s
    float pitch_rate_p_out;          // Pitch 角速度内环 P 项输出
    float pitch_rate_output;         // Pitch 角速度内环总输出
    int16_t motor_pitch_diff;        // Pitch 前后差动值
    float throttle;                  // 油门
    float pitch_rate_d_out;          // 内环 D 项输出
    float premes;
    float mes;
    float pitch_rate_i_out;          // 内环 I 项输出
} DebugFrame;

DebugFrame debug_buffer[DEBUG_TX_BUFFER_SIZE];
volatile uint16_t debug_head = 0;
volatile uint16_t debug_tail = 0;

/*
 * 环形缓冲区入队
 * 目前由 main while 中调用
 * 队满时覆盖最旧数据
 */
static void enqueue_debug_frame(DebugFrame *frame)
{
    uint16_t next_head = (debug_head + 1) % DEBUG_TX_BUFFER_SIZE;
    if (next_head == debug_tail) {
        // 队满，覆盖最旧数据
        debug_tail = (debug_tail + 1) % DEBUG_TX_BUFFER_SIZE;
    }
    debug_buffer[debug_head] = *frame;
    debug_head = next_head;
}

/*
 * 环形缓冲区出队
 * 目前由 main while 中调用
 */
static int dequeue_debug_frame(DebugFrame *frame)
{
    if (debug_head == debug_tail) {
        return 0;
    }
    *frame = debug_buffer[debug_tail];
    debug_tail = (debug_tail + 1) % DEBUG_TX_BUFFER_SIZE;
    return 1;
}

// 摇杆数据缓冲区
float target_roll = 0;
float target_pitch = 0;
float target_yaw = 0;
float pitch_rate_target = 0;
u16 len = 0;
float throttle = 0;
u16 safe_flag = 0;
u8 usart6_receive_buffer[256];
u8 usart6_buffer_index = 0;

int main()
{
    HAL_Init();
    SystemClock_Init(8, 336, 2, 7);
    SysTick_Init(168);

    USART1_Init(115200);   // 串口1：printf
    USART3_Init(115200);   // 串口3：蓝牙 / Serial Plot
    USART6_Init(115200);   // 串口6：遥控器 IBUS

    usart1_send_char(1);

    IBUS_Init();
    LED_Init();
    KEY_Init();

    TIM5_CH1_PWM_Init(19999, 168 - 1);    // 定时器5输出PWM
    TIM4_Init(5000 - 1, 84 - 1);          // 定时器4控制中断

    FlightController_Init(&flight_controller);

    MPU6050_Init();
    MPU6050_Verify_Config();           // 验证MPU6050配置

    // 初始化陀螺仪低通滤波器
    if (Biquad_LowPass_Init(&gyro_filter_x, GYRO_LPF_CUTOFF_HZ, GYRO_SAMPLE_RATE) != 0 ||
        Biquad_LowPass_Init(&gyro_filter_y, GYRO_LPF_CUTOFF_HZ, GYRO_SAMPLE_RATE) != 0 ||
        Biquad_LowPass_Init(&gyro_filter_z, GYRO_LPF_CUTOFF_HZ, GYRO_SAMPLE_RATE) != 0)
    {
        printf("Gyro LPF init failed!\r\n");
        while (1);
    }

    printf("b0=%.6f, b1=%.6f, b2=%.6f, a1=%.6f, a2=%.6f\r\n",
           gyro_filter_x.b0, gyro_filter_x.b1, gyro_filter_x.b2,
           gyro_filter_x.a1, gyro_filter_x.a2);

    // 初始化 DMP
    while (mpu_dmp_init()) {
        printf("MPU6050 Error!\r\n");
        delay_ms(200);
    }
    printf("MPU6050 OK!\r\n");

    // MPU 稳定后校准
    MPU6050_Calibrate_Gyro();

    diantiao_start();
    esc_output_enable = 1;
    diantiao_flag = 1;

    printf("LPF init: cutoff=%.1f Hz, fs=%.1f Hz\r\n",
           GYRO_LPF_CUTOFF_HZ,
           GYRO_SAMPLE_RATE);

    while (1)
    {
        /*
         * DMP 姿态更新成功后，读取陀螺仪和遥控器并采集调试数据
         */
        if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
        {
            // 读取角速度和加速度原始值
            MPU6050_Get_Accelerometer(&aacx, &aacy, &aacz);
            MPU6050_Get_Gyroscope(&gyrox, &gyroy, &gyroz);

            // 去除校准偏置
            if (gyro_calibrated) {
                gyrox -= gyro_bias_x;
                gyroy -= gyro_bias_y;
                gyroz -= gyro_bias_z;
                aacx -= acc_bias_x;
                aacy -= acc_bias_y;
                aacz -= acc_bias_z;
            }

            // 填充 DMP 姿态角
            attitude_buffer.pitch = pitch;
            attitude_buffer.roll = roll;
            attitude_buffer.yaw = yaw;

            // 陀螺仪原始值转换为 deg/s
            gyro_x_dps = (float)gyrox / GYRO_SCALE_FACTOR;
            gyro_y_dps = (float)gyroy / GYRO_SCALE_FACTOR;
            gyro_z_dps = (float)gyroz / GYRO_SCALE_FACTOR;

            // 应用二阶 Butterworth 低通滤波
            gyro_x_dps = Biquad_Update(&gyro_filter_x, gyro_x_dps);
            gyro_y_dps = Biquad_Update(&gyro_filter_y, gyro_y_dps);
            gyro_z_dps = Biquad_Update(&gyro_filter_z, gyro_z_dps);

            /*
             * 填充姿态角速度
             * 目前 Pitch 环使用 gyro_y_dps，
             * 如外环或内环异常，先检查此处映射及电机序号
             */
            attitude_buffer.roll_rate  = gyro_x_dps;
            attitude_buffer.pitch_rate = gyro_y_dps;
            attitude_buffer.yaw_rate   = gyro_z_dps;

            /*
             * 读取遥控器数据
             * 当前保持原有方式：
             * target_pitch、throttle 等变量在此循环更新，
             * TIM4 中断中直接读取
             */
            USART6_Process_IBUS_Data();
            IBUS_Map_To_Control(&throttle, &target_roll, &target_pitch, &target_yaw);
            safe_flag = ibus_data.channels[4];

            // 摇杆安全通道<1020时，没有故障且角度正常时，才会开启飞控更新控制
            if ((safe_flag < 1020) && (flight_failsafe == 0)) {
                diantiao_flag = 1;
            } else {
                diantiao_flag = 0;
            }

            /*
             * 更新姿态到 current_attitude
             * current_attitude 是控制中断实际使用的姿态数据
             * 关中断时拷贝给中断，避免中断读取到一半更新的数据
             */
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            current_attitude = attitude_buffer;
            __set_PRIMASK(primask);

            /*
             * 采集调试数据并放入环形缓冲区
             * 注意：
             * flight_controller 中的外环/内环变量在 TIM4 中断中更新
             * 需关中断拷贝一份，避免读到更新过程中的中间状态
             */
            DebugFrame frame;
            uint32_t primask_dbg = __get_PRIMASK();
            __disable_irq();
            frame.target_pitch          = target_pitch;
            frame.current_pitch         = current_attitude.pitch;
            frame.pitch_angle_error     = flight_controller.pitch_angle_error;
            frame.pitch_rate_target_raw = flight_controller.pitch_rate_target_raw;
            frame.pitch_rate_target_out = flight_controller.pitch_rate_target_out;
            frame.current_pitch_rate    = current_attitude.pitch_rate;
            frame.pitch_rate_p_out      = flight_controller.pitch_rate_pid.p_out;
            frame.pitch_rate_output     = flight_controller.pitch_rate_output;
            frame.motor_pitch_diff      = motor_pitch_diff;
            frame.throttle              = throttle;
            frame.pitch_rate_d_out      = flight_controller.pitch_rate_pid.d_out;
            frame.mes                   = flight_controller.pitch_rate_pid.mes;
            frame.premes                = flight_controller.pitch_rate_pid.premes;
            frame.pitch_rate_i_out      = flight_controller.pitch_rate_pid.i_out;

            __set_PRIMASK(primask_dbg);
            enqueue_debug_frame(&frame);
        }

        /* ========== ASCII 调试数据发送（配合 Serial Plot）==========
         * 配合串口绘图器使用。
         * 每 DEBUG_SEND_INTERVAL_MS 发送一帧。
         * 发送顺序：
         * target_pitch,
         * current_pitch,
         * pitch_angle_error,
         * pitch_rate_target_raw,
         * pitch_rate_target_out,
         * current_pitch_rate,
         * pitch_rate_p_out,
         * pitch_rate_output,
         * motor_pitch_diff,
         * throttle
         */
        static uint32_t last_debug_send_time = 0;
        uint32_t now2 = HAL_GetTick();
        if (now2 - last_debug_send_time >= DEBUG_SEND_INTERVAL_MS) {
            last_debug_send_time = now2;
            DebugFrame frame;
            if (dequeue_debug_frame(&frame)) {
                char buf[160];
                int len = sprintf(buf,
                    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f\r\n",
                    frame.target_pitch,
                    frame.current_pitch,
                    frame.pitch_angle_error,
                    frame.pitch_rate_target_raw,
                    frame.pitch_rate_target_out,
                    frame.current_pitch_rate,
                    frame.pitch_rate_p_out,
                    frame.pitch_rate_output,
                    frame.motor_pitch_diff,
                    frame.throttle,
                    frame.pitch_rate_d_out,
                    frame.pitch_rate_i_out);
                // 使用 DMA 发送字符串
                usart3_send_dma((uint8_t*)buf, len);
            }
        }
    }
}

void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM4_Handler);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t stopped = 1;

    if (htim->Instance == TIM4)
    {
        // MPU校准完成并执行diantiao_start之前，不在TIM4中断写电机PWM
        if (esc_output_enable == 0) {
            return;
        }

        // 故障保护。一旦 flight_failsafe 变为 1，飞控在解锁前强制保持停机
        if (flight_failsafe) {
            diantiao_flag = 0;
        }

        // 检查 Pitch 安全检查（同步检查 Roll 安全）
        float roll_err = target_roll - current_attitude.roll;
        // Pitch 安全检查
        float pitch_err = target_pitch - current_attitude.pitch;

        if (diantiao_flag == 1) {
            if (fabsf(pitch_err) >= PITCH_ANGLE_FAILSAFE_DEG) {
                flight_failsafe = 1;
                diantiao_flag = 0;
            }
            if (fabsf(current_attitude.pitch_rate) >= PITCH_RATE_FAILSAFE_DPS) {
                flight_failsafe = 1;
                diantiao_flag = 0;
            }
            if (fabsf(roll_err) >= PITCH_ANGLE_FAILSAFE_DEG) {
                flight_failsafe = 1;
                diantiao_flag = 0;
            }
            if (fabsf(current_attitude.roll_rate) >= PITCH_RATE_FAILSAFE_DPS) {
                flight_failsafe = 1;
                diantiao_flag = 0;
            }
        }

        // diantiao_flag 为 0 时停止控制更新，不输出电机信号
        if (diantiao_flag == 0) {
            if (!stopped) {
                FlightController_Stop(&flight_controller);
                stopped = 1;
            }
            TIM5_SetCompare1(500);
            TIM5_SetCompare2(500);
            TIM5_SetCompare3(500);
            TIM5_SetCompare4(500);
            return;
        }

        if (diantiao_flag == 1) {
            static uint8_t loop_cnt = 0;
            stopped = 0;

            flight_controller.throttle = throttle;

            // 外环 100Hz，每两次中断跑一次
            if ((loop_cnt & 1) == 0) {
                FlightController_Update_Outer(&flight_controller, &current_attitude,
                               target_roll, target_pitch, target_yaw, OUTER_LOOP_DT);
            }

            // 内环 200Hz，每次都跑
            FlightController_Update_Inner(&flight_controller, &current_attitude, INNER_LOOP_DT);

            loop_cnt++;
            if (loop_cnt > 200)
                loop_cnt = 1;
        }
    }
}

void USART6_Receive_Callback(u8 data)
{
    usart6_send_char(data);
    if (data == '\r') {
        usart6_send_char('\n');
    }
}

void diantiao_init(void)
{
    TIM5_SetCompare1(1999);
    TIM5_SetCompare2(1999);
    TIM5_SetCompare3(1999);
    TIM5_SetCompare4(1999);
    HAL_Delay(4000);

    TIM5_SetCompare1(1000);
    TIM5_SetCompare2(1000);
    TIM5_SetCompare3(1000);
    TIM5_SetCompare4(1000);
    HAL_Delay(5000);

    printf("diaotiao_init success\n");
}

void diantiao_start(void)
{
    TIM5_SetCompare1(500);
    TIM5_SetCompare2(500);
    TIM5_SetCompare3(500);
    TIM5_SetCompare4(500);

    printf("diaotiao_start success\n");
    HAL_Delay(2000);
}

void MPU6050_Calibrate_Gyro(void)
{
    short gx, gy, gz;
    short ax, ay, az;
    long sum_gx = 0;
    long sum_gy = 0;
    long sum_gz = 0;
    long sum_ax = 0;
    long sum_ay = 0;
    long sum_az = 0;

    printf("Calibrating gyro, please keep MPU6050 stationary...\n");

    for (int i = 0; i < 1000; i++) {
        MPU6050_Get_Gyroscope(&gx, &gy, &gz);
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;

        MPU6050_Get_Accelerometer(&ax, &ay, &az);
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az - 16384;

        delay_ms(5);
    }

    gyro_bias_x = sum_gx / 1000;
    gyro_bias_y = sum_gy / 1000;
    gyro_bias_z = sum_gz / 1000;
    acc_bias_x = sum_ax / 1000;
    acc_bias_y = sum_ay / 1000;
    acc_bias_z = sum_az / 1000;
    gyro_calibrated = 1;

    printf("Gyro calibration complete: AX=%d, AY=%d, AZ=%d, GX=%d, GY=%d, GZ=%d\n",
           acc_bias_x,
           acc_bias_y,
           acc_bias_z,
           gyro_bias_x,
           gyro_bias_y,
           gyro_bias_z);
}
