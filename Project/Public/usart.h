#ifndef _usart_H
#define _usart_H

#include "system.h"
#include "stdio.h"
#include "ibus.h"

// ************** 宏定义 **************
#define USART1_REC_LEN              200
#define USART3_REC_LEN              32      // 串口3接收缓冲区大小（DMA用）
#define USART3_DMA_BUFFER_SIZE      64      // DMA接收缓冲区大小（需覆盖2帧数据）
#define USART6_REC_LEN              32
#define USART6_DMA_BUFFER_SIZE      32
#define RXBUFFERSIZE                1

// ************** 串口1 **************
extern u8  USART1_RX_BUF[USART1_REC_LEN];
extern u16 USART1_RX_STA;
extern UART_HandleTypeDef UART1_Handler;

void USART1_Init(u32 bound);
void usart1_send_char(u8 c);
void usart1_niming_report(u8 fun, u8* data, u8 len);
void mpu6050_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz);
void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw);

// ************** 串口3（DMA）**************
extern UART_HandleTypeDef UART3_Handler;
extern DMA_HandleTypeDef USART3_RxDMA_Handler;
extern u8 USART3_Rx_DMA_Buffer[USART3_DMA_BUFFER_SIZE];
extern volatile u8 USART3_DMA_Ready;   // DMA接收完成标志，由中断全程置位

void USART3_Init(u32 bound);
void USART3_DMA_Init(void);
void usart3_send_char(u8 c);
void usart3_send_string(u8 *str);
void usart3_niming_report(u8 fun, u8* data, u8 len);
void mpu6050_send_data_bt(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz);
void usart3_send_imu_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw);
void USART3_Process_Received_Data(void);  // 处理接收到的数据

// 串口3 DMA发送
extern DMA_HandleTypeDef USART3_TxDMA_Handler;
void USART3_TxDMA_Init(void);
void usart3_send_dma(uint8_t *data, uint16_t len);

// ************** 串口6 **************
extern u8  USART6_RX_BUF[USART6_REC_LEN];
extern u16 USART6_RX_STA;
extern UART_HandleTypeDef UART6_Handler;
extern DMA_HandleTypeDef USART6_RxDMA_Handler;
extern u8 aRxBuffer[RXBUFFERSIZE];
extern u8 aRxBuffer6[RXBUFFERSIZE];
extern u8 USART6_Rx_DMA_Buffer[USART6_DMA_BUFFER_SIZE];
extern volatile u8 USART6_DMA_Ready;

void USART6_Init(u32 bound);
void USART6_DMA_Init(void);
void usart6_send_char(u8 c);
void usart6_send_string(u8 *str);
void USART6_Receive_Callback(u8 data);
void USART6_DMA_Receive(void);
void USART6_Process_IBUS_Data(void);

#endif
