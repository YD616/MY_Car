#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include "stm32f1xx_hal_gpio.h"

/* GPIO 外部中断回调（覆盖 HAL 库 __weak 弱函数，实现在 Interrupt.c） */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif
