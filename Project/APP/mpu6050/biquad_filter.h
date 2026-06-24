#ifndef __BIQUAD_FILTER_H
#define __BIQUAD_FILTER_H

#include <stdint.h>

/* 双二阶滤波器实现（转置直接Ⅱ型结构） */
typedef struct {
    float b0, b1, b2;   // 前馈系数
    float a1, a2;       // 反馈系数（a0 已归一化为 1）
    float w1, w2;       // 延迟状态
} BiquadFilter;

/*
 * @brief 初始化二阶 Butterworth 低通滤波器
 * @param filt   滤波器实例指针
 * @param cutoff 截止频率（Hz），推荐值：30~40 Hz（F330 + 8寸桨）
 * @param fs     采样频率（Hz），需与实际数据更新频率一致。
 *               与 mpu6050.c 中 MPU6050_Set_Rate(500) 调用对应，此处应为 500.0f
 * @return 0:成功，-1:参数无效
 */
int Biquad_LowPass_Init(BiquadFilter *filt, float cutoff, float fs);

/**
 * @brief 送入单个采样值（标量版本）
 * @param filt  滤波器实例指针
 * @param input 输入采样
 * @return      滤波输出
 */
float Biquad_Update(BiquadFilter *filt, float input);

/**
 * @brief 批量处理三维向量（陀螺仪三轴）
 * @param filt_x, filt_y, filt_z 三个滤波器实例指针
 * @param in_x, in_y, in_z       输入数组指针
 * @param out_x, out_y, out_z    输出数组指针（可与输入相同）
 * @param len                    数据点个数
 */
void Biquad_Update_Vec3(BiquadFilter *filt_x, BiquadFilter *filt_y, BiquadFilter *filt_z,
                        const float *in_x, const float *in_y, const float *in_z,
                        float *out_x, float *out_y, float *out_z, uint32_t len);

#endif /* __BIQUAD_FILTER_H */
