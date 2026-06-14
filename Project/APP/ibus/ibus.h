#ifndef __IBUS_H
#define __IBUS_H

#include "system.h"
#include "main.h"

// IBUS协议定义
#define IBUS_BUFFER_SIZE         32
#define IBUS_FRAME_HEADER        0x4020  // 小端模式：0x20 0x40
#define IBUS_CHANNEL_COUNT       14
#define IBUS_TIMEOUT_MS          100

// IBUS数据结构
typedef struct {
    uint16_t channels[IBUS_CHANNEL_COUNT];  // 14个通道数据
    uint8_t valid;                          // 数据有效标志
    uint32_t last_update;                   // 最后更新时间
    uint8_t lost_flag;                      // 信号丢失标志
} IBUS_Data;

// 全局变量声明
extern IBUS_Data ibus_data;

// 函数声明
void IBUS_Init(void);
void IBUS_Process_Data(uint8_t *buffer);
uint8_t IBUS_Is_Valid(void);
uint8_t IBUS_Is_Lost(void);
uint16_t IBUS_Get_Channel(uint8_t channel);
float IBUS_Get_Channel_Normalized(uint8_t channel);
void IBUS_Map_To_Control(float *throttle, float *roll, float *pitch, float *yaw);
void IBUS_Debug_Print(void);

#endif /* __IBUS_H */
