#include "Delay.h"
#include "stm32f1xx_hal.h"

/* 延时初始化: HAL_Init 已配好 SysTick 1ms 时基, 无需额外操作 */
void Delay_Init(void)
{
}

/* 微秒延时: 基于 DWT 周期计数器 (72MHz), 首次调用时使能 */
void Delay_us(uint32_t nus)
{
    static uint8_t dwt_inited = 0;
    volatile uint32_t start, now;

    /* 首次调用使能 DWT 周期计数器 */
    if (dwt_inited == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        DWT->CYCCNT = 0;
        dwt_inited = 1;
    }

    start = DWT->CYCCNT;
    while (1)
    {
        now = DWT->CYCCNT;
        if ((now - start) >= (uint32_t)(nus * 72))  /* 72MHz → 1us = 72 周期 */
            break;
    }
}

void Delay_ms(uint32_t nms)
{
    HAL_Delay(nms);
}

void Delay_s(uint32_t ns)
{
    HAL_Delay(ns * 1000);
}
