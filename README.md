# STM32F407 Quadcopter Flight Controller

基于 STM32F407 的四轴飞行器飞控系统，采用串级 PID 控制，支持 MPU6050 DMP 姿态解算、IBUS 遥控器协议、蓝牙无线调参。

> 大三实验室开放项目 | 自动化专业 | 独立开发（本项目使用江协科技提供的DMP库源码解算姿态角）

## 硬件平台

| 组件 | 型号 |
|------|------|
| 主控 | STM32F407ZGT6（Cortex-M4, 168MHz） |
| IMU | MPU6050（6轴陀螺仪+加速度计，内置DMP） |
| 遥控器 | FlySky FS-i6（IBUS协议） |
| 接收机 | FS-iA6B |
| 蓝牙模块 | HC-05（串口透传，用于调参和数据监视） |
| 电调 | BLHeli_S 30A（PWM 500-2000us） |
| 机架 | X型 250mm 轴距 |

> 硬件使用 STM32F407ZGT6 开发板 + 杜邦线连接各外设模块。

## 功能特性

- **姿态解算**：MPU6050 内置 DMP（Digital Motion Processor）输出欧拉角，500Hz 更新率
- **串级 PID 控制**：
  - 外环（角度环）：100Hz，P 控制，将角度误差转换为角速度目标
  - 内环（角速度环）：200Hz，PID 控制（D on measurement），将角速度误差转换为电机输出
- **多级低通滤波**：
  - MPU6050 内部 DLPF：42Hz（寄存器配置）
  - 陀螺仪二阶 Butterworth 低通滤波：35Hz 截止频率（`biquad_filter.c`）
  - 内环 D 项一阶低通滤波：截止约 20Hz@100Hz（PID 内部）
- **Slew Rate 斜坡限制**：限制角速度目标的变化率，避免阶跃冲击
- **故障保护**：角度/角速度超限自动停转（failsafe）
- **IBUS 遥控器协议**：支持 FlySky FS-i6 14通道数据解析
- **蓝牙实时数据**：通过串口 DMA 发送调试数据（12通道 CSV 格式），可用 SerialPlot 实时查看波形
- **NRF24L01 无线通信**：驱动代码已完成，仅需初始化即可使用（早期用于自制遥控器通信，因抗干扰问题已替换为富斯 IBUS 遥控系统）

## 控制架构

```
遥控器(IBUS) ──→ 角度外环PID(100Hz) ──→ Slew Rate限幅 ──→ 角速度内环PID(200Hz) ──→ 电机混控 ──→ PWM输出
                      ↑                                        ↑
                 DMP姿态角                          Butterworth低通(35Hz) + D项低通(20Hz)
                      ↑                                        ↑
                 MPU6050(DMP) ────────────────── MPU6050(原始陀螺仪数据)
                                                      ↑
                                                内部DLPF(42Hz)
```

### PID 参数（当前版本）

| 回路 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|------|----|----|----|---------|---------|
| Pitch 角度外环 | 2.0 | 0 | 0 | 100 | 80 |
| Pitch 角速度内环 | 0.4 | 0.08 | 0.003 | 100 | 160 |
| Roll 角度外环 | 2.0 | 0 | 0 | 100 | 80 |
| Roll 角速度内环 | 0.4 | 0.08 | 0.003 | 100 | 100 |
| Yaw 角度外环 | 2.0 | 0 | 0 | 100 | 30 |
| Yaw 角速度内环 | 0.8 | 0.15 | 0 | 100 | 100 |

## 目录结构

```
├── User/               # 用户代码（主程序入口）
│   ├── main.c          # 主循环、初始化、中断回调、校准
│   ├── main.h          # 全局宏定义、外环/内环频率配置
│   └── stm32f4xx_hal_msp.c  # HAL 外设 MSP 初始化
├── APP/                # 应用层模块
│   ├── control/        # 飞控核心（串级PID + 混控 + 故障保护）
│   ├── pid/            # PID 控制器（标准PID + D-on-measurement）
│   ├── mpu6050/        # MPU6050 驱动（I2C读写 + DMP接口 + 低通滤波）
│   │   ├── biquad_filter.c/h  # 二阶 Butterworth 低通滤波器
│   │   └── eMPL/       # InvenSense 官方 DMP 驱动库（江协科技提供）
│   ├── motor/          # 电机PWM输出（TIM5 四通道）
│   ├── ibus/           # FlySky IBUS 协议解析
│   ├── nrf24l01/       # NRF24L01 2.4G无线模块驱动（已完成，待集成）
│   ├── pwm/            # PWM初始化
│   ├── iic/            # 软件I2C驱动
│   ├── led/            # LED指示
│   ├── key/            # 按键输入
│   └── tim/            # 定时器初始化
├── Public/             # 公共驱动
│   ├── usart.c/h       # 串口驱动（USART1/3/6 + DMA）
│   ├── system.c/h      # 系统时钟配置
│   └── SysTick.c/h     # 系统滴答定时器
├── Libraries/          # STM32 标准外设库
│   ├── CMSIS/          # Cortex-M4 CMSIS
│   └── STM32F4xx_HAL_Driver/  # HAL 库
├── 姿态参数/           # PID调参时的姿态数据（CSV）
└── 新姿态参数/         # 优化代码后的新PID参数的调试数据（CSV）
```

## 编译和烧录

### 环境要求

- **IDE**: Keil MDK-ARM V5
- **编译器**: Arm Compiler 5 或 6
- **调试器**: ST-Link V2
- **STM32 CubeMX**: 用于 HAL 库配置（可选）

### 编译步骤

1. 用 Keil MDK 打开工程文件（`Template.uvprojx`）
2. 确认芯片型号为 STM32F407ZGTx
3. 点击 Build（F7）编译
4. 点击 Download（F8）烧录

### 串口连接

| 串口 | 引脚 | 用途 |
|------|------|------|
| USART1 | PA9(TX), PA10(RX) | printf 调试输出 |
| USART3 | PB10(TX), PB11(RX) | 蓝牙/SerialPlot 数据发送 |
| USART6 | PC6(TX), PC7(RX) | IBUS 遥控器接收 |

## 调试数据格式

USART3 以 115200 波特率输出 CSV 格式的调试数据帧（20ms/帧），可用 [SerialPlot](https://hackaday.io/project/5334-serialplot) 软件实时查看波形。

### 数据帧格式（12通道）

```
target_pitch, current_pitch, pitch_angle_error, pitch_rate_target_raw,
pitch_rate_target_out, current_pitch_rate, pitch_rate_p_out,
pitch_rate_output, motor_pitch_diff, throttle, d_out, i_out
```

| 通道 | 变量 | 说明 | 单位 |
|------|------|------|------|
| 1 | target_pitch | 遥控器目标角度 | deg |
| 2 | current_pitch | 当前实际角度 | deg |
| 3 | pitch_angle_error | 外环角度误差 | deg |
| 4 | pitch_rate_target_raw | 外环原始角速度输出 | deg/s |
| 5 | pitch_rate_target_out | 斜坡限幅后角速度目标 | deg/s |
| 6 | current_pitch_rate | 当前角速度（滤波后） | deg/s |
| 7 | pitch_rate_p_out | 内环P项输出 | - |
| 8 | pitch_rate_output | 内环总输出 | - |
| 9 | motor_pitch_diff | Pitch 差动值 | PWM |
| 10 | throttle | 油门 | % |
| 11 | d_out | 内环D项输出 | - |
| 12 | i_out | 内环I项输出 | - |

### 查看方法

1. 打开 SerialPlot
2. 选择对应的串口号，波特率 115200
3. 导入本项目 `姿态参数/` 目录下的 CSV 文件作为参考
4. 设置通道数为12，分隔符为逗号

## PID 调参记录

`姿态参数/` 和 `新姿态参数/` 目录下保存了完整的调参过程数据。文件命名规则：

```
[模式] [参数值].csv
```

例如：`外环 Kp=2.0,内环 Kp=0.4,Kd=0.003,Ki=0.08.csv`

这些数据记录了从最初 Kp=0.5 开始逐步调参到最终参数的全过程，可配合 SerialPlot 可视化分析。

## 已知问题和待改进

- [ ] 目前仅完成地面测试（姿态跟随、电机响应），尚未实际飞行
- [ ] 飞控上电后需要保持静止等待陀螺仪校准完成（约5秒）
- [ ] DMP 姿态角在剧烈机动时可能存在漂移，考虑引入互补滤波/扩展卡尔曼滤波
- [ ] I2C 通信缺少超时重试机制
- [ ] 仅实现了 Acro（Rate）模式和 Angle 模式，缺少定高、GPS 等高级功能

## 开发日志

- 2025年7月：搭建硬件平台，完成 MPU6050 驱动和 DMP 初始化
- 2025年8月：实现 IBUS 遥控器协议解析，PWM 电机驱动
- 2025年8-12月：串级 PID 控制器开发与调参，增加低通滤波和保护逻辑
- 2026年3-6月：蓝牙串口调试功能，大量调参测试（见姿态参数数据）
- 2026年6月：代码整理，准备上传 GitHub

## 许可证

MIT License — 详见 [LICENSE](LICENSE) 文件。

---

**Author**: 杨佳鑫
**Contact**: 2677177898@qq.com
**University**: 西北民族大学 自动化专业 2023级
