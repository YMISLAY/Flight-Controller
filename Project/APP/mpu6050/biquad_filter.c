/**
 * @file    biquad_filter.c
 * @brief   Butterworth LPF via bilinear transform — transposed DF-II.
 */

#include "biquad_filter.h"
#include <stddef.h>  // 提供 NULL 标识
#include <math.h>

#ifndef M_PI
#define M_PI 3.141592653589793f
#endif

/* 计算二阶 Butterworth 低通滤波器系数（双线性变换法） */
static void calc_butterworth_lpf_coeffs(float cutoff, float fs,
                                        float *b0, float *b1, float *b2,
                                        float *a1, float *a2)
{
    float omega = 2.0f * M_PI * cutoff / fs;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.7071067811865475f); // Q = 1/√2

    float a0 = 1.0f + alpha;
    *b0 = (1.0f - cs) / 2.0f / a0;
    *b1 = (1.0f - cs) / a0;
    *b2 = (1.0f - cs) / 2.0f / a0;
    *a1 = -2.0f * cs / a0;
    *a2 = (1.0f - alpha) / a0;
}

int Biquad_LowPass_Init(BiquadFilter *filt, float cutoff, float fs)
{
    if (filt == NULL || cutoff <= 0.0f || fs <= 0.0f || cutoff >= fs / 2.0f)
        return -1;
    calc_butterworth_lpf_coeffs(cutoff, fs,
                                &filt->b0, &filt->b1, &filt->b2,
                                &filt->a1, &filt->a2);
    filt->w1 = 0.0f;
    filt->w2 = 0.0f;
    return 0;
}
float Biquad_Update(BiquadFilter *filt, float input)
{
    // 转置直接Ⅱ型结构，数值稳定性好，仅存两个状态变量
    float out = filt->b0 * input + filt->w1;
    filt->w1 = filt->b1 * input - filt->a1 * out + filt->w2;
    filt->w2 = filt->b2 * input - filt->a2 * out;
    return out;
}

void Biquad_Update_Vec3(BiquadFilter *filt_x, BiquadFilter *filt_y, BiquadFilter *filt_z,
                        const float *in_x, const float *in_y, const float *in_z,
                        float *out_x, float *out_y, float *out_z, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        out_x[i] = Biquad_Update(filt_x, in_x[i]);
        out_y[i] = Biquad_Update(filt_y, in_y[i]);
        out_z[i] = Biquad_Update(filt_z, in_z[i]);
    }
}
