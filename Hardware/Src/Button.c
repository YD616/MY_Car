#include "Button.h"
#include "stm32f1xx_hal.h"
#include "Delay.h"         /* Delay_ms */

/* 按键状态: 每按一次翻转 (0/1 交替) */
uint8_t Button_Num = 0;

/* 读取按键 (PA5, 低电平按下): 阻塞式, 含 10ms 消抖 + 等松手, 松手后翻转状态 */
uint8_t Button_Get(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
        Delay_ms(10);                     /* 消抖 */
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET)
        {
            Delay_ms(10);                 /* 等松手 */
        }
        Button_Num = !Button_Num;
    }
    return Button_Num;
}
