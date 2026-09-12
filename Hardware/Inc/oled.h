/* SSD1306 OLED 128x64 驱动 (4线软件SPI)
 * 引脚映射: RST=PB3, DC=PA15, SCLK=PB5, SDIN=PB4 */
#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"

/* ---- 引脚定义 ---- */
#define OLED_RST_PORT    GPIOB
#define OLED_RST_PIN     GPIO_PIN_3    /* RST   - PB3 */

#define OLED_DC_PORT     GPIOA
#define OLED_DC_PIN      GPIO_PIN_15   /* DC    - PA15 */

#define OLED_SCLK_PORT   GPIOB
#define OLED_SCLK_PIN    GPIO_PIN_5    /* SCLK  - PB5 */

#define OLED_SDIN_PORT   GPIOB
#define OLED_SDIN_PIN    GPIO_PIN_4    /* SDIN  - PB4 */

/* ---- 命令 / 数据标志 ---- */
#define OLED_CMD  0   /* 写命令 */
#define OLED_DATA 1   /* 写数据 */

/* ---- 字体尺寸 (正点原子风格兼容宏) ---- */
#define OLED_6X8   6   /* 6x8 字体 (1206) */
#define OLED_8X16  16  /* 8x16 字体 (1608) */
#define OLED_Update()  OLED_Refresh_Gram()  /* 兼容 OLED_Update() 调用 */

/* ---- 函数声明 ---- */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *p, uint8_t size);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t mode);

void OLED_ClearArea(uint8_t x1, uint8_t y1, uint8_t width, uint8_t height);   /* 清矩形区域 */

void OLED_ShowFloatNum(uint8_t x, uint8_t y, double num, uint8_t intlen, uint8_t declen, uint8_t size);

/* 显示带符号浮点数: 正数显'+', 负数显'-', 按固定总宽右对齐;
 * 总宽 = 符号1位 + intlen位整数 + 小数点 + declen位小数 (空位补空格) */
void OLED_ShowSignedFloat(uint8_t x, uint8_t y, float val,
                          uint8_t intlen, uint8_t declen, uint8_t size);

#endif /* __OLED_H__ */
