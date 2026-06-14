#include "usart.h"

UART_HandleTypeDef UART1_Handler;
UART_HandleTypeDef UART3_Handler;
UART_HandleTypeDef UART6_Handler;

DMA_HandleTypeDef USART3_RxDMA_Handler;
DMA_HandleTypeDef USART3_TxDMA_Handler;  // 串口3 TX DMA 句柄
DMA_HandleTypeDef USART6_RxDMA_Handler;

u8 USART1_RX_BUF[USART1_REC_LEN];
u16 USART1_RX_STA = 0;
u8 aRxBuffer[RXBUFFERSIZE];

// USART3 DMA接收
u8 USART3_Rx_DMA_Buffer[USART3_DMA_BUFFER_SIZE];
volatile u8 USART3_DMA_Ready = 0;   // 0:空闲, 1:有数据待处理

u8 USART6_RX_BUF[USART6_REC_LEN];
u16 USART6_RX_STA = 0;
u8 aRxBuffer6[RXBUFFERSIZE];
u8 USART6_Rx_DMA_Buffer[USART6_DMA_BUFFER_SIZE];
volatile u8 USART6_DMA_Ready = 0;

int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = (u8)ch;
    return ch;
}

//==================== 串口1 ====================

void USART1_Init(u32 bound)
{
    UART1_Handler.Instance = USART1;
    UART1_Handler.Init.BaudRate = bound;
    UART1_Handler.Init.WordLength = UART_WORDLENGTH_8B;
    UART1_Handler.Init.StopBits = UART_STOPBITS_1;
    UART1_Handler.Init.Parity = UART_PARITY_NONE;
    UART1_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART1_Handler.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&UART1_Handler);
    HAL_UART_Receive_IT(&UART1_Handler, aRxBuffer, RXBUFFERSIZE);
}

void usart1_send_char(u8 c)
{
    while (__HAL_UART_GET_FLAG(&UART1_Handler, UART_FLAG_TC) != SET);
    HAL_UART_Transmit(&UART1_Handler, &c, 1, 1000);
}

void usart1_niming_report(u8 fun, u8* data, u8 len)
{
    u8 send_buf[32];
    u8 i;
    if (len > 28) return;
    send_buf[len + 3] = 0;
    send_buf[0] = 0x88;
    send_buf[1] = fun;
    send_buf[2] = len;
    for (i = 0; i < len; i++) send_buf[3 + i] = data[i];
    for (i = 0; i < len + 3; i++) send_buf[len + 3] += send_buf[i];
    for (i = 0; i < len + 4; i++) usart1_send_char(send_buf[i]);
}

void mpu6050_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
    u8 tbuf[12];
    tbuf[0] = (aacx >> 8) & 0xFF;
    tbuf[1] = aacx & 0xFF;
    tbuf[2] = (aacy >> 8) & 0xFF;
    tbuf[3] = aacy & 0xFF;
    tbuf[4] = (aacz >> 8) & 0xFF;
    tbuf[5] = aacz & 0xFF;
    tbuf[6] = (gyrox >> 8) & 0xFF;
    tbuf[7] = gyrox & 0xFF;
    tbuf[8] = (gyroy >> 8) & 0xFF;
    tbuf[9] = gyroy & 0xFF;
    tbuf[10] = (gyroz >> 8) & 0xFF;
    tbuf[11] = gyroz & 0xFF;
    usart1_niming_report(0xA1, tbuf, 12);
}

void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
    u8 tbuf[28];
    u8 i;
    for (i = 0; i < 28; i++) tbuf[i] = 0;
    tbuf[0] = (aacx >> 8) & 0xFF;
    tbuf[1] = aacx & 0xFF;
    tbuf[2] = (aacy >> 8) & 0xFF;
    tbuf[3] = aacy & 0xFF;
    tbuf[4] = (aacz >> 8) & 0xFF;
    tbuf[5] = aacz & 0xFF;
    tbuf[6] = (gyrox >> 8) & 0xFF;
    tbuf[7] = gyrox & 0xFF;
    tbuf[8] = (gyroy >> 8) & 0xFF;
    tbuf[9] = gyroy & 0xFF;
    tbuf[10] = (gyroz >> 8) & 0xFF;
    tbuf[11] = gyroz & 0xFF;
    tbuf[18] = (roll >> 8) & 0xFF;
    tbuf[19] = roll & 0xFF;
    tbuf[20] = (pitch >> 8) & 0xFF;
    tbuf[21] = pitch & 0xFF;
    tbuf[22] = (yaw >> 8) & 0xFF;
    tbuf[23] = yaw & 0xFF;
    usart1_niming_report(0xAF, tbuf, 28);
}

//==================== 串口3（DMA）====================

// USART3 TX DMA 初始化
void USART3_TxDMA_Init(void)
{
    // 使能 DMA1 时钟（如尚未使能）
    __HAL_RCC_DMA1_CLK_ENABLE();

    // USART3_TX 通道使用 DMA1_Stream3, Channel4 (参考手册)
    USART3_TxDMA_Handler.Instance = DMA1_Stream3;
    USART3_TxDMA_Handler.Init.Channel = DMA_CHANNEL_4;
    USART3_TxDMA_Handler.Init.Direction = DMA_MEMORY_TO_PERIPH;
    USART3_TxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    USART3_TxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    USART3_TxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    USART3_TxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    USART3_TxDMA_Handler.Init.Mode = DMA_NORMAL;       // 普通模式，发完即停
    USART3_TxDMA_Handler.Init.Priority = DMA_PRIORITY_LOW; // 优先级低于接收
    USART3_TxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&USART3_TxDMA_Handler);

    // 将 DMA 绑定到 UART 句柄（用于发送）
    __HAL_LINKDMA(&UART3_Handler, hdmatx, USART3_TxDMA_Handler);

    // 使能发送完成中断（可选，用于等待发送完成）
    __HAL_DMA_ENABLE_IT(&USART3_TxDMA_Handler, DMA_IT_TC);
}

// 非阻塞DMA 发送函数（内部等待完成，避免缓冲区冲突）
void usart3_send_dma(uint8_t *data, uint16_t len)
{
    // 等待前一次 DMA 发送完成
    while (HAL_DMA_GetState(&USART3_TxDMA_Handler) == HAL_DMA_STATE_BUSY) {
        // 可加入超时处理
    }
    // 启动 DMA 发送
    HAL_UART_Transmit_DMA(&UART3_Handler, data, len);
}

void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&USART3_TxDMA_Handler);
}

void USART3_Init(u32 bound)
{
    UART3_Handler.Instance = USART3;
    UART3_Handler.Init.BaudRate = bound;
    UART3_Handler.Init.WordLength = UART_WORDLENGTH_8B;
    UART3_Handler.Init.StopBits = UART_STOPBITS_1;
    UART3_Handler.Init.Parity = UART_PARITY_NONE;
    UART3_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART3_Handler.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&UART3_Handler);

    USART3_DMA_Init();
    USART3_TxDMA_Init();    // 初始化 DMA 发送
}

void USART3_DMA_Init(void)
{
    // 使能DMA1时钟，STM32F4中USART3_RX通道为DMA1 Stream1 Channel4
    __HAL_RCC_DMA1_CLK_ENABLE();

    USART3_RxDMA_Handler.Instance = DMA1_Stream1;
    USART3_RxDMA_Handler.Init.Channel = DMA_CHANNEL_4;
    USART3_RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;
    USART3_RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    USART3_RxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    USART3_RxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    USART3_RxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    USART3_RxDMA_Handler.Init.Mode = DMA_CIRCULAR;
    USART3_RxDMA_Handler.Init.Priority = DMA_PRIORITY_MEDIUM;
    USART3_RxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&USART3_RxDMA_Handler);

    __HAL_LINKDMA(&UART3_Handler, hdmarx, USART3_RxDMA_Handler);

    // 启动DMA接收
    if (HAL_UART_Receive_DMA(&UART3_Handler, USART3_Rx_DMA_Buffer, USART3_DMA_BUFFER_SIZE) != HAL_OK) {
        printf("USART3 DMA Receive Error!\r\n");
    }

    // 使能DMA全传输中断和半传输中断（用于及时处理数据）
    __HAL_DMA_ENABLE_IT(&USART3_RxDMA_Handler, DMA_IT_TC | DMA_IT_HT);
}

void usart3_send_char(u8 c)
{
    HAL_UART_Transmit(&UART3_Handler, &c, 1, 100);
}

void usart3_send_string(u8 *str)
{
    while (*str) usart3_send_char(*str++);
}

void usart3_niming_report(u8 fun, u8* data, u8 len)
{
    u8 send_buf[32];
    u8 i;
    if (len > 28) return;
    send_buf[len + 3] = 0;
    send_buf[0] = 0x88;
    send_buf[1] = fun;
    send_buf[2] = len;
    for (i = 0; i < len; i++) send_buf[3 + i] = data[i];
    for (i = 0; i < len + 3; i++) send_buf[len + 3] += send_buf[i];
    for (i = 0; i < len + 4; i++) usart3_send_char(send_buf[i]);
}

void mpu6050_send_data_bt(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
    u8 tbuf[12];
    tbuf[0] = (aacx >> 8) & 0xFF;
    tbuf[1] = aacx & 0xFF;
    tbuf[2] = (aacy >> 8) & 0xFF;
    tbuf[3] = aacy & 0xFF;
    tbuf[4] = (aacz >> 8) & 0xFF;
    tbuf[5] = aacz & 0xFF;
    tbuf[6] = (gyrox >> 8) & 0xFF;
    tbuf[7] = gyrox & 0xFF;
    tbuf[8] = (gyroy >> 8) & 0xFF;
    tbuf[9] = gyroy & 0xFF;
    tbuf[10] = (gyroz >> 8) & 0xFF;
    tbuf[11] = gyroz & 0xFF;
    usart3_niming_report(0xA1, tbuf, 12);
}

void usart3_send_imu_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
    u8 tbuf[28];
    u8 i;
    for (i = 0; i < 28; i++) tbuf[i] = 0;
    tbuf[0] = (aacx >> 8) & 0xFF;
    tbuf[1] = aacx & 0xFF;
    tbuf[2] = (aacy >> 8) & 0xFF;
    tbuf[3] = aacy & 0xFF;
    tbuf[4] = (aacz >> 8) & 0xFF;
    tbuf[5] = aacz & 0xFF;
    tbuf[6] = (gyrox >> 8) & 0xFF;
    tbuf[7] = gyrox & 0xFF;
    tbuf[8] = (gyroy >> 8) & 0xFF;
    tbuf[9] = gyroy & 0xFF;
    tbuf[10] = (gyroz >> 8) & 0xFF;
    tbuf[11] = gyroz & 0xFF;
    tbuf[18] = (roll >> 8) & 0xFF;
    tbuf[19] = roll & 0xFF;
    tbuf[20] = (pitch >> 8) & 0xFF;
    tbuf[21] = pitch & 0xFF;
    tbuf[22] = (yaw >> 8) & 0xFF;
    tbuf[23] = yaw & 0xFF;
    usart3_niming_report(0xAF, tbuf, 28);
}

// 处理接收到的数据（在循环中调用）
void USART3_Process_Received_Data(void)
{
    // 读取当前 DMA 已接收字节数（通过尾指针位置）
    uint16_t ndtr = __HAL_DMA_GET_COUNTER(&USART3_RxDMA_Handler);
    uint16_t current_pos = USART3_DMA_BUFFER_SIZE - ndtr;
    static uint16_t last_pos = 0;

    if (current_pos != last_pos) {
        // 处理 last_pos 到 current_pos 之间的数据
        if (current_pos > last_pos) {
            for (uint16_t i = last_pos; i < current_pos; i++) {
                u8 data = USART3_Rx_DMA_Buffer[i];
                usart3_send_char(data);
                printf("%c", data);
            }
        } else {  // 环形覆盖
            for (uint16_t i = last_pos; i < USART3_DMA_BUFFER_SIZE; i++) {
                u8 data = USART3_Rx_DMA_Buffer[i];
                usart3_send_char(data);
                printf("%c", data);
            }
            for (uint16_t i = 0; i < current_pos; i++) {
                u8 data = USART3_Rx_DMA_Buffer[i];
                usart3_send_char(data);
                printf("%c", data);
            }
        }
        last_pos = current_pos;
    }
}

//==================== 串口6 ====================

void USART6_Init(u32 bound)
{
    UART6_Handler.Instance = USART6;
    UART6_Handler.Init.BaudRate = bound;
    UART6_Handler.Init.WordLength = UART_WORDLENGTH_8B;
    UART6_Handler.Init.StopBits = UART_STOPBITS_1;
    UART6_Handler.Init.Parity = UART_PARITY_NONE;
    UART6_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART6_Handler.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&UART6_Handler);

    USART6_DMA_Init();
}

void USART6_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    USART6_RxDMA_Handler.Instance = DMA2_Stream1;
    USART6_RxDMA_Handler.Init.Channel = DMA_CHANNEL_5;
    USART6_RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;
    USART6_RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    USART6_RxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    USART6_RxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    USART6_RxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    USART6_RxDMA_Handler.Init.Mode = DMA_CIRCULAR;
    USART6_RxDMA_Handler.Init.Priority = DMA_PRIORITY_HIGH;
    USART6_RxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&USART6_RxDMA_Handler);

    __HAL_LINKDMA(&UART6_Handler, hdmarx, USART6_RxDMA_Handler);

    USART6_DMA_Receive();
}

void USART6_DMA_Receive(void)
{
    USART6_DMA_Ready = 0;
    if (HAL_UART_Receive_DMA(&UART6_Handler, USART6_Rx_DMA_Buffer, USART6_DMA_BUFFER_SIZE) != HAL_OK) {
        printf("USART6 DMA Receive Error!\r\n");
    }
}

void usart6_send_char(u8 c)
{
    while (__HAL_UART_GET_FLAG(&UART6_Handler, UART_FLAG_TC) != SET);
    HAL_UART_Transmit(&UART6_Handler, &c, 1, 1000);
}

void usart6_send_string(u8 *str)
{
    while (*str) usart6_send_char(*str++);
}

void USART6_Process_IBUS_Data(void)
{
    if (USART6_DMA_Ready) {
        USART6_DMA_Ready = 0;
        IBUS_Process_Data(USART6_Rx_DMA_Buffer);
    }
}

//==================== HAL MSP 回调 ====================

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_EnableIRQ(USART1_IRQn);
        HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);
    }
    else if (huart->Instance == USART3) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // DMA接收中断
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 2);
        // DMA发送中断
        HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
        HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 2);
        // 串口接收中断
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        HAL_NVIC_SetPriority(USART3_IRQn, 2, 2);
    }
    else if (huart->Instance == USART6) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_USART6_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        HAL_NVIC_EnableIRQ(USART6_IRQn);
        HAL_NVIC_SetPriority(USART6_IRQn, 2, 2);
        HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
        HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 1);
    }
}

//==================== DMA中断回调 ====================

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // 串口1原有处理...
        if ((USART1_RX_STA & 0x8000) == 0) {
            if (USART1_RX_STA & 0x4000) {
                if (aRxBuffer[0] != 0x0a) USART1_RX_STA = 0;
                else USART1_RX_STA |= 0x8000;
            } else {
                if (aRxBuffer[0] == 0x0d) USART1_RX_STA |= 0x4000;
                else {
                    USART1_RX_BUF[USART1_RX_STA & 0x3FFF] = aRxBuffer[0];
                    USART1_RX_STA++;
                    if (USART1_RX_STA > (USART1_REC_LEN - 1)) USART1_RX_STA = 0;
                }
            }
        }
        HAL_UART_Receive_IT(&UART1_Handler, aRxBuffer, RXBUFFERSIZE);
    }
    else if (huart->Instance == USART3) {
        // DMA接收完成或半传输时置标志，具体在DMA中断中处理细节
        // 此处仅置标志，实际可由DMA的TC/HT中断处理
        USART3_DMA_Ready = 1;
    }
    else if (huart->Instance == USART6) {
        USART6_DMA_Ready = 1;
    }
}

// 为了及时处理半传输和全传输写在DMA中断回调（可选）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3) {
        USART3_DMA_Ready = 1;
    }
    else if (huart->Instance == USART6) {
        USART6_DMA_Ready = 1;
    }
}

// 中断服务函数

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UART1_Handler);
    uint32_t timeout = 0;
    while (HAL_UART_GetState(&UART1_Handler) != HAL_UART_STATE_READY) {
        if (++timeout > HAL_MAX_DELAY) break;
    }
    timeout = 0;
    while (HAL_UART_Receive_IT(&UART1_Handler, aRxBuffer, RXBUFFERSIZE) != HAL_OK) {
        if (++timeout > HAL_MAX_DELAY) break;
    }
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UART3_Handler);
}

void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UART6_Handler);
}

void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&USART3_RxDMA_Handler);
}

void DMA2_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&USART6_RxDMA_Handler);
}
