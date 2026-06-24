/**
 * @file    pwm.h
 * @brief   TIM5 4-channel PWM for ESC (50 Hz on PA0–PA3).
 */

#ifndef _pwm_H
#define _pwm_H

#include "system.h"

void TIM5_PWM_Init(u32 per, u16 psc);
void TIM5_SetCompare1(u32 compare);
void TIM5_SetCompare2(u32 compare);
void TIM5_SetCompare3(u32 compare);
void TIM5_SetCompare4(u32 compare);

#endif
