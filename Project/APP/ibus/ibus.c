#include "ibus.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

// IBUS全局变量
IBUS_Data ibus_data;

/**
  * @brief  IBUS初始化
  * @param  无
  * @retval 无
  */
void IBUS_Init(void)
{
    // 初始化IBUS数据结构
    memset(&ibus_data, 0, sizeof(IBUS_Data));

    // 设置通道默认值（中立位置）
    for (int i = 0; i < IBUS_CHANNEL_COUNT; i++) {
        ibus_data.channels[i] = 1500;  // 中立位置
    }

    ibus_data.valid = 0;
    ibus_data.lost_flag = 1;  // 初始状态为信号丢失
    ibus_data.last_update = 0;

    printf("IBUS Protocol Initialized\r\n");
}

/**
  * @brief  处理IBUS数据
  * @param  buffer: 接收到的数据缓冲区
  * @retval 无
  */
void IBUS_Process_Data(uint8_t *buffer)
{
    uint16_t frame_header = (buffer[1] << 8) | buffer[0];

    // 检查帧头
    if (frame_header != IBUS_FRAME_HEADER) {
        return; // 帧头错误
    }

    // 计算校验和
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < 30; i++) {
        checksum -= buffer[i];
    }

    uint16_t received_checksum = (buffer[31] << 8) | buffer[30];
    if (checksum != received_checksum) {
        return; // 校验和错误
    }

    // 解析通道数据（小端模式）
    for (int i = 0; i < IBUS_CHANNEL_COUNT; i++) {
        uint8_t low_byte = buffer[2 + i * 2];
        uint8_t high_byte = buffer[2 + i * 2 + 1];
        ibus_data.channels[i] = (high_byte << 8) | low_byte;
    }

    ibus_data.valid = 1;
    ibus_data.lost_flag = 0;
    ibus_data.last_update = HAL_GetTick();

    // 调试打印（可选）
    // IBUS_Debug_Print();
}

/**
  * @brief  检查IBUS数据是否有效
  * @param  无
  * @retval 1:有效, 0:无效
  */
uint8_t IBUS_Is_Valid(void)
{
    // 检查数据是否超时
    if (ibus_data.valid && (HAL_GetTick() - ibus_data.last_update > IBUS_TIMEOUT_MS)) {
        ibus_data.valid = 0;
        ibus_data.lost_flag = 1;
    }

    return ibus_data.valid;
}

/**
  * @brief  检查IBUS信号是否丢失
  * @param  无
  * @retval 1:丢失, 0:正常
  */
uint8_t IBUS_Is_Lost(void)
{
    IBUS_Is_Valid();  // 更新状态
    return ibus_data.lost_flag;
}

/**
  * @brief  读取指定通道的原始值
  * @param  channel: 通道号 (0-13)
  * @retval 通道原始值 (1000-2000)
  */
uint16_t IBUS_Get_Channel(uint8_t channel)
{
    if (channel >= IBUS_CHANNEL_COUNT) {
        return 1500;  // 默认中立值
    }

    return ibus_data.channels[channel];
}

/**
  * @brief  读取指定通道的归一化值
  * @param  channel: 通道号 (0-13)
  * @retval 归一化值 (-1.0 到 +1.0)
  */
float IBUS_Get_Channel_Normalized(uint8_t channel)
{
    uint16_t raw_value = IBUS_Get_Channel(channel);
    return (raw_value - 1500.0f) / 500.0f;  // 转换为 -1.0 到 +1.0
}

/**
  * @brief  将IBUS通道映射到飞控控制量
  * @param  throttle: 油门指针
  * @param  roll: 滚转指针
  * @param  pitch: 俯仰指针
  * @param  yaw: 偏航指针
  * @retval 无
  */
void IBUS_Map_To_Control(float *throttle, float *roll, float *pitch, float *yaw)
{
    if (!IBUS_Is_Valid()) {
        // 信号丢失，设置安全值
        *throttle = 0;
        *roll = 0;
        *pitch = 0;
        *yaw = 0;
        return;
    }

    // 标准通道映射（根据富斯i6默认设置）
    // 通道2: 油门 (1000-2000) -> (0-150)
    *throttle = (ibus_data.channels[2] - 1000.0f) * 1.5f / 10.0f;

    // 通道0: 横滚 (1000-2000) -> (-30° to +30°)
    *roll = ((ibus_data.channels[0] - 1500.0f) * 3.0f / 50.0f) + 0.06f;

    // 通道1: 俯仰 (1000-2000) -> (-30° to +30°)
    *pitch = (ibus_data.channels[1] - 1500.0f) * 3.0f / 50.0f;

    // 通道3: 偏航 (1000-2000) -> (-180° to +180°)
    *yaw = (ibus_data.channels[3] - 1500.0f) * 18.0f / 50.0f;

    // 限制范围
    *throttle = (*throttle < 0) ? 0 : (*throttle > 150) ? 150 : *throttle;
    *roll = (*roll < -30) ? -30 : (*roll > 30) ? 30 : *roll;
    *pitch = (*pitch < -30) ? -30 : (*pitch > 30) ? 30 : *pitch;
    *yaw = (*yaw < -180) ? -180 : (*yaw > 180) ? 180 : *yaw;
}

/**
  * @brief  IBUS调试信息打印
  * @param  无
  * @retval 无
  */
void IBUS_Debug_Print(void)
{
    if (IBUS_Is_Valid()) {
        printf("IBUS: ");
        for (int i = 0; i < 6; i++) {  // 只打印前6个通道
            printf("CH%d:%4d ", i, ibus_data.channels[i]);
        }
        printf("\r\n");
    } else {
        printf("IBUS: Signal Lost\r\n");
    }
}
