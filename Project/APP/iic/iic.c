#include "iic.h"
#include "SysTick.h"

/*******************************************************************************
* 函 数 名         : IIC_Init
* 函数功能         : IIC初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    IIC_SCL_PORT_RCC;
    IIC_SDA_PORT_RCC;

    GPIO_InitStructure.Pin = IIC_SCL_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
    GPIO_InitStructure.Pull = GPIO_PULLUP;          // 上拉
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;     // 高速
    HAL_GPIO_Init(IIC_SCL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = IIC_SDA_PIN;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);

    IIC_SDA = 1;
    IIC_SCL = 1;
}

/*******************************************************************************
* 函 数 名         : SDA_OUT
* 函数功能         : SDA输出模式
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
    GPIO_InitStructure.Pull = GPIO_PULLUP;          // 上拉
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;     // 高速
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);
}

/*******************************************************************************
* 函 数 名         : SDA_IN
* 函数功能         : SDA输入模式
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;          // 上拉
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;     // 高速
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);
}

/*******************************************************************************
* 函 数 名         : IIC_Start
* 函数功能         : 发送IIC起始信号
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Start(void)
{
    SDA_OUT();     // sda线输出
    IIC_SDA = 1;
    IIC_SCL = 1;
    delay_us(2);
    IIC_SDA = 0;   // START: when CLK is high, DATA change from high to low
    delay_us(3);
    IIC_SCL = 0;   // 钳住I2C总线，准备发送或接收
}

/*******************************************************************************
* 函 数 名         : IIC_Stop
* 函数功能         : 发送IIC停止信号
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Stop(void)
{
    SDA_OUT();     // sda线输出
    IIC_SCL = 0;
    IIC_SDA = 0;   // STOP: when CLK is high DATA change from low to high
    IIC_SCL = 1;
    delay_us(2);
    IIC_SDA = 1;   // 发送I2C总线结束信号
    delay_us(3);
}

/*******************************************************************************
* 函 数 名         : IIC_Wait_Ack
* 函数功能         : 等待应答信号到来
* 输    入         : 无
* 输    出         : 1，接收应答失败
                     0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
    u8 tempTime = 0;

    SDA_IN();      // SDA设置为输入
    IIC_SDA = 1;
    delay_us(1);
    IIC_SCL = 1;
    delay_us(1);

    while (READ_SDA) {
        tempTime++;
        if (tempTime > 250) {
            IIC_Stop();
            return 1;
        }
    }

    IIC_SCL = 0;   // 时钟输出0
    return 0;
}

/*******************************************************************************
* 函 数 名         : IIC_Ack
* 函数功能         : 发送ACK应答
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(5);
    IIC_SCL = 0;
}

/*******************************************************************************
* 函 数 名         : IIC_NAck
* 函数功能         : 发送NACK非应答
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(5);
    IIC_SCL = 0;
}

/*******************************************************************************
* 函 数 名         : IIC_Send_Byte
* 函数功能         : IIC发送一个字节
* 输    入         : txd：发送的一个字节
* 输    出         : 无
*******************************************************************************/
void IIC_Send_Byte(u8 txd)
{
    u8 t;

    SDA_OUT();
    IIC_SCL = 0;   // 拉低时钟开始数据传输

    for (t = 0; t < 8; t++) {
        if ((txd & 0x80) > 0)   // 0x80  1000 0000
            IIC_SDA = 1;
        else
            IIC_SDA = 0;

        txd <<= 1;
        delay_us(1);   // 对TEA5767这三个延时都是必须的
        IIC_SCL = 1;
        delay_us(1);
        IIC_SCL = 0;
        delay_us(1);
    }
}

/*******************************************************************************
* 函 数 名         : IIC_Read_Byte
* 函数功能         : IIC读一个字节
* 输    入         : ack=1时发送ACK，ack=0发送nACK
* 输    出         : 应答或非应答
*******************************************************************************/
u8 IIC_Read_Byte(u8 ack)
{
    u8 i, receive = 0;

    SDA_IN();      // SDA设置为输入

    for (i = 0; i < 8; i++) {
        IIC_SCL = 0;
        delay_us(1);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA) receive++;
        delay_us(1);
    }

    if (!ack)
        IIC_NAck(); // 发送nACK
    else
        IIC_Ack();  // 发送ACK

    return receive;
}
