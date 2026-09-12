#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

void Delay_Init(void);              /* 初始化 (HAL 已配好时基) */
void Delay_us(uint32_t nus);        /* 微秒延时 (DWT 周期计数器) */
void Delay_ms(uint32_t nms);        /* 毫秒延时 */
void Delay_s(uint32_t ns);          /* 秒延时 */

#endif
