/* 外部中断回调: PA12 下降沿触发, 中断内完成姿态解算
 * 注意: 中断内禁止 printf / HAL_Delay / OLED 等耗时操作 */
#include "main.h"
#include "Interrupt.h"
#include "app_sensor.h"

/* GPIO 外部中断回调 (覆盖 HAL 库 __weak 弱函数) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_12)
    {
        Sensor_Angle_Update();   /* 采集 + 卡尔曼融合 + 磁力计航向 */
    }
}
