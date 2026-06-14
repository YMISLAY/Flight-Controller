#include "system.h"

// 时钟系统配置函数
// Fvco = Fs * (plln / pllm);
// SYSCLK = Fvco / pllp = Fs * (plln / (pllm * pllp));
// Fusb = Fvco / pllq = Fs * (plln / (pllm * pllq));
// Fvco: VCO频率
// SYSCLK: 系统时钟频率
// Fusb: USB, SDIO, RNG等的时钟频率
// Fs: PLL输入时钟频率, 可以是HSI, HSE等.
// plln: 主PLL倍频系数(PLL倍频), 取值范围: 64~432.
// pllm: 主PLL输入分频PLL分频系数(PLL之前的分频), 取值范围: 2~63.
// pllp: 系统时钟的主PLL分频系数(PLL之后的分频), 取值范围: 2,4,6,8.(只有这4个值!)
// pllq: USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频), 取值范围: 2~15.
// 外部晶振为8M的时候, 推荐值: plln=336, pllm=8, pllp=2, pllq=7.
// 得到: Fvco = 8 * (336 / 8) = 336Mhz
//       SYSCLK = 336 / 2 = 168Mhz
//       Fusb = 336 / 7 = 48Mhz
// 返回值: 0, 成功; 1, 失败

void SystemClock_Init(u32 pllm, u32 plln, u32 pllp, u32 pllq)
{
    HAL_StatusTypeDef ret = HAL_OK;
    RCC_OscInitTypeDef RCC_OscInitStructure;
    RCC_ClkInitTypeDef RCC_ClkInitStructure;

    __HAL_RCC_PWR_CLK_ENABLE(); // 使能PWR时钟

    // 下面这个设置用来设置调压器输出电压级别，以便在器件未运行到最大频率
    // 时使能与频率匹配的性能与功耗平衡。
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); // 设置调压器输出电压级别1

    RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSE;    // 时钟源为HSE
    RCC_OscInitStructure.HSEState = RCC_HSE_ON;                      // 开HSE
    RCC_OscInitStructure.PLL.PLLState = RCC_PLL_ON;                  // 开PLL
    RCC_OscInitStructure.PLL.PLLSource = RCC_PLLSOURCE_HSE;          // PLL时钟源选择HSE
    RCC_OscInitStructure.PLL.PLLM = pllm; // 主PLL输入分频PLL分频系数(PLL之前的分频), 取值范围: 2~63.
    RCC_OscInitStructure.PLL.PLLN = plln; // 主PLL倍频系数(PLL倍频), 取值范围: 64~432.
    RCC_OscInitStructure.PLL.PLLP = pllp; // 系统时钟的主PLL分频系数(PLL之后的分频), 取值范围: 2,4,6,8.(只有这4个值!)
    RCC_OscInitStructure.PLL.PLLQ = pllq; // USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频), 取值范围: 2~15.
    ret = HAL_RCC_OscConfig(&RCC_OscInitStructure); // 初始化

    if (ret != HAL_OK) while (1);

    // 选中PLL作为系统时钟源并且配置HCLK, PCLK1和PCLK2
    RCC_ClkInitStructure.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStructure.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;   // 设置系统时钟时钟源为PLL
    RCC_ClkInitStructure.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB分频系数为1
    RCC_ClkInitStructure.APB1CLKDivider = RCC_HCLK_DIV4;          // APB1分频系数为4
    RCC_ClkInitStructure.APB2CLKDivider = RCC_HCLK_DIV2;          // APB2分频系数为2
    ret = HAL_RCC_ClockConfig(&RCC_ClkInitStructure, FLASH_LATENCY_5); // 同时设置FLASH延时周期为5WS，也就是6个CPU周期。

    if (ret != HAL_OK) while (1);

    // STM32F405x/407x/415x/417x Z版本的器件才支持预取指
    if (HAL_GetREVID() == 0x1001) {
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();  // 使能flash预取指
    }
}
