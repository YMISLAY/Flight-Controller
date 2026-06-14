#ifndef _pwm_H
#define _pwm_H

#include "system.h"

void TIM5_CH1_PWM_Init(u32 per, u16 psc);
void TIM5_SetCompare1(u32 compare);
void TIM5_SetCompare2(u32 compare);
void TIM5_SetCompare3(u32 compare);
void TIM5_SetCompare4(u32 compare);

#endif
