#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>
#include "main.h"
#include "Delay.h"
#include "stm32f1xx_hal.h"
extern uint8_t Button_Num;     /* 定义在 Button.c */
uint8_t Button_Get(void);
#endif /* BUTTON_H */
