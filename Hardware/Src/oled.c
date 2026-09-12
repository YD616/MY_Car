/* SSD1306 OLED 128x64 驱动 (4线软件SPI)
 * 引脚: SCLK=PB5, SDIN=PB4, DC=PA15, RST=PB3 (已在 gpio.c 初始化为推挽输出) */
#include "oled.h"
#include "oledfont.h"
#include <stdio.h>
#include <string.h>

/* ---- 显存 ---- */
static uint8_t OLED_GRAM[128][8];

/* ---- 底层引脚控制(内联) ---- */
static inline void OLED_RST_Clr(void) { HAL_GPIO_WritePin(OLED_RST_PORT,  OLED_RST_PIN,  GPIO_PIN_RESET); }
static inline void OLED_RST_Set(void) { HAL_GPIO_WritePin(OLED_RST_PORT,  OLED_RST_PIN,  GPIO_PIN_SET);   }
static inline void OLED_DC_Clr(void)  { HAL_GPIO_WritePin(OLED_DC_PORT,   OLED_DC_PIN,   GPIO_PIN_RESET); }
static inline void OLED_DC_Set(void)  { HAL_GPIO_WritePin(OLED_DC_PORT,   OLED_DC_PIN,   GPIO_PIN_SET);   }
static inline void OLED_SCLK_Clr(void){ HAL_GPIO_WritePin(OLED_SCLK_PORT, OLED_SCLK_PIN, GPIO_PIN_RESET); }
static inline void OLED_SCLK_Set(void){ HAL_GPIO_WritePin(OLED_SCLK_PORT, OLED_SCLK_PIN, GPIO_PIN_SET);   }
static inline void OLED_SDIN_Clr(void){ HAL_GPIO_WritePin(OLED_SDIN_PORT, OLED_SDIN_PIN, GPIO_PIN_RESET); }
static inline void OLED_SDIN_Set(void){ HAL_GPIO_WritePin(OLED_SDIN_PORT, OLED_SDIN_PIN, GPIO_PIN_SET);   }

/* ---- 基本读写 ---- */
/* 向 OLED 写一个字节 (模拟SPI时序): cmd 0=命令, 1=数据 */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    uint8_t i;

    if (cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();

    for (i = 0; i < 8; i++) {
        OLED_SCLK_Clr();
        if (dat & 0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        OLED_SCLK_Set();
        dat <<= 1;
    }

    OLED_DC_Set();   /* 恢复为数据模式 */
}

/* ---- 显示控制 ---- */
void OLED_Display_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);  /* SET DCDC命令 */
    OLED_WR_Byte(0x14, OLED_CMD);  /* DCDC ON */
    OLED_WR_Byte(0xAF, OLED_CMD);  /* DISPLAY ON */
}

void OLED_Display_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);  /* SET DCDC命令 */
    OLED_WR_Byte(0x10, OLED_CMD);  /* DCDC OFF */
    OLED_WR_Byte(0xAE, OLED_CMD);  /* DISPLAY OFF */
}

/* 清屏 (填充全黑) */
void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
        for (n = 0; n < 128; n++)
            OLED_GRAM[n][i] = 0x00;
    OLED_Refresh_Gram();
}

/* 将显存刷新到 OLED (整屏刷新) */
void OLED_Refresh_Gram(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        OLED_WR_Byte(0xB0 + i, OLED_CMD);   /* 设置页地址 (0~7) */
        OLED_WR_Byte(0x00, OLED_CMD);        /* 设置显示位置—列低地址 */
        OLED_WR_Byte(0x10, OLED_CMD);        /* 设置显示位置—列高地址 */
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
    }
}

/* ---- 绘图 ---- */
/* 画点 (x 0~127, y 0~63): t 1=点亮, 0=熄灭 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t pos, bx, temp = 0;

    if (x > 127 || y > 63) return;   /* 超出范围 */

    pos = 7 - y / 8;
    bx  = y % 8;
    temp = 1 << (7 - bx);

    if (t)
        OLED_GRAM[x][pos] |= temp;
    else
        OLED_GRAM[x][pos] &= ~temp;
}

/* ---- 字符/字符串/数字显示 ---- */
/* 显示一个 ASCII 字符: size 12 或 16, mode 0=反白 1=正常 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
    uint8_t temp, t, t1;
    uint8_t y0 = y;

    chr = chr - ' ';  /* 得到偏移后的值 */

    for (t = 0; t < size; t++) {
        if (size == 12)
            temp = oled_asc2_1206[chr][t];  /* 1206字体 */
        else
            temp = oled_asc2_1608[chr][t];  /* 1608字体 */

        for (t1 = 0; t1 < 8; t1++) {
            if (temp & 0x80)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp <<= 1;
            y++;
            if ((y - y0) == size) {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/* 求 m 的 n 次方 (内部辅助函数) */
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/* 显示数字: 高位补空格, 超出部分正常显示 */
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++) {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size, 1);
                continue;
            } else {
                enshow = 1;
            }
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 1);
    }
}

/* 显示字符串: 自动换行(超出x→下一行, 超出y→清屏), size 取 OLED_6X8 / OLED_8X16 */
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *p, uint8_t size)
{
#define MAX_CHAR_POSX  122
#define MAX_CHAR_POSY  58

    /* 字体映射: 6x8 → 1206字体(12), 8x16 → 1608字体(16) */
    uint8_t font = (size == OLED_6X8) ? 12 : 16;
    uint8_t step = (size == OLED_6X8) ? 6 : 8;

    while (*p != '\0') {
        if (x > MAX_CHAR_POSX) { x = 0; y += font; }
        if (y > MAX_CHAR_POSY) { y = x = 0; OLED_Clear(); }
        OLED_ShowChar(x, y, *p, font, 1);
        x += step;
        p++;
    }
}

/* 显示汉字 (16x16 点阵): num 为字库序号, 仅改显存需 OLED_Update() 上屏
 * 字库移植自 WHEELTEC B570 平衡小车 oledfont.h */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t mode)
{
    uint8_t t, t1;
    uint8_t temp;

    for (t = 0; t < 16; t++) {                    /* 16行 */
        temp = oled_chinese_1616[num][t * 2];     /* 左半部分 (高8位) */
        for (t1 = 0; t1 < 8; t1++) {
            if (temp & 0x80)
                OLED_DrawPoint(x + t1, y + t, mode);
            else
                OLED_DrawPoint(x + t1, y + t, !mode);
            temp <<= 1;
        }
        temp = oled_chinese_1616[num][t * 2 + 1]; /* 右半部分 (低8位) */
        for (t1 = 0; t1 < 8; t1++) {
            if (temp & 0x80)
                OLED_DrawPoint(x + 8 + t1, y + t, mode);
            else
                OLED_DrawPoint(x + 8 + t1, y + t, !mode);
            temp <<= 1;
        }
    }
}

/* 清空指定矩形区域 (左上角 x1,y1 + 宽高), 仅改显存需 OLED_Update() 上屏 */
void OLED_ClearArea(uint8_t x1, uint8_t y1, uint8_t width, uint8_t height)
{
    uint8_t x, y;
    for (y = y1; y < y1 + height && y <= 63; y++)
        for (x = x1; x < x1 + width && x <= 127; x++)
            OLED_DrawPoint(x, y, 0);
}

/* 显示浮点数: 用整数拆分, 不依赖 sprintf 的 %f 浮点格式化
 * (Keil ARMCC 部分库配置下 %f 小数位会被截断/丢失);
 * 正数不显 '+', 小数位固定按 declen 显示 */
void OLED_ShowFloatNum(uint8_t x, uint8_t y, double num, uint8_t intlen, uint8_t declen, uint8_t size)
{
    uint8_t i;
    uint8_t font = (size == OLED_6X8) ? 12 : 16;
    uint8_t step = (size == OLED_6X8) ? 6 : 8;
    long    ipart, dpart, mult, tmp;
    uint8_t ilen, dlen;

    /* 负数处理 */
    if (num < 0.0) {
        OLED_ShowChar(x, y, '-', font, 1);
        x += step;
        num = -num;
    }

    /* 拆整数部分 */
    ipart = (long)num;

    /* 拆小数部分 (四舍五入到 declen 位) */
    mult = 1;
    for (i = 0; i < declen; i++) mult *= 10;
    dpart = (long)((num - (double)ipart) * (double)mult + 0.5);
    if (dpart >= mult) {                /* 小数进位到整数 (如 3.999 → 4.00) */
        dpart -= mult;
        ipart++;
    }

    /* 整数位数 */
    tmp = ipart; ilen = 1;
    while (tmp >= 10) { tmp /= 10; ilen++; }

    /* 整数部分: 左补空格到 intlen 宽, 逐位显示 (纯整数运算, 不依赖 printf 库) */
    for (i = intlen; i > ilen; i--) {
        OLED_ShowChar(x, y, ' ', font, 1);
        x += step;
    }
    tmp = ipart;
    for (i = ilen; i > 0; i--) {
        OLED_ShowChar(x, y, (uint8_t)('0' + (tmp / oled_pow(10, i - 1)) % 10), font, 1);
        x += step;
    }

    /* 小数点 */
    OLED_ShowChar(x, y, '.', font, 1);
    x += step;

    /* 小数位数 */
    tmp = dpart; dlen = 1;
    while (tmp >= 10) { tmp /= 10; dlen++; }

    /* 小数部分: 高位补 '0' 到 declen 位, 逐位显示 */
    for (i = declen; i > dlen; i--) {
        OLED_ShowChar(x, y, '0', font, 1);
        x += step;
    }
    tmp = dpart;
    for (i = dlen; i > 0; i--) {
        OLED_ShowChar(x, y, (uint8_t)('0' + (tmp / oled_pow(10, i - 1)) % 10), font, 1);
        x += step;
    }
}

/* 显示带符号浮点数: 正数显'+', 负数显'-', 按固定总宽右对齐, 内部用 snprintf("%+*.*f")
 * 与 OLED_ShowFloatNum 的区别: 本函数强制带符号并按固定宽度对齐, 但依赖 C 库浮点 printf */
void OLED_ShowSignedFloat(uint8_t x, uint8_t y, float val,
                          uint8_t intlen, uint8_t declen, uint8_t size)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%+*.*f", (int)intlen + declen + 2, (int)declen, (double)val);
    OLED_ShowString(x, y, (const uint8_t *)buf, size);
}

/* ---- 初始化 ---- */
/* 初始化 OLED: 执行 SSD1306 寄存器配置序列 (引脚已由 MX_GPIO_Init 配好) */
void OLED_Init(void)
{
    /* 硬件复位 */
    OLED_RST_Clr();
    HAL_Delay(100);
    OLED_RST_Set();

    /* SSD1306 初始化命令序列 */
    OLED_WR_Byte(0xAE, OLED_CMD); /* 关闭显示 */

    OLED_WR_Byte(0xD5, OLED_CMD); /* 设置时钟分频因子/振荡器频率 */
    OLED_WR_Byte(80, OLED_CMD);   /* [3:0]分频因子; [7:4]振荡器频率 */

    OLED_WR_Byte(0xA8, OLED_CMD); /* 设置驱动路数 */
    OLED_WR_Byte(0x3F, OLED_CMD); /* 默认0x3F (1/64) */

    OLED_WR_Byte(0xD3, OLED_CMD); /* 设置显示偏移 */
    OLED_WR_Byte(0x00, OLED_CMD); /* 默认0 */

    OLED_WR_Byte(0x40, OLED_CMD); /* 设置显示起始行 [5:0]行号 */

    OLED_WR_Byte(0x8D, OLED_CMD); /* 电荷泵设置 */
    OLED_WR_Byte(0x14, OLED_CMD); /* bit2: 开启/关闭 */

    OLED_WR_Byte(0x20, OLED_CMD); /* 设置内存地址模式 */
    OLED_WR_Byte(0x02, OLED_CMD); /* [1:0]: 00=列地址; 01=行地址; 10=页地址 */

    OLED_WR_Byte(0xA1, OLED_CMD); /* 段重定义设置: bit0 — 0:0→0; 1:0→127 */
    OLED_WR_Byte(0xC0, OLED_CMD); /* COM扫描方向: bit3 — 0:普通; 1:重定义 COM[N-1]→COM0 */

    OLED_WR_Byte(0xDA, OLED_CMD); /* 设置COM硬件引脚配置 */
    OLED_WR_Byte(0x12, OLED_CMD); /* [5:4]配置 */

    OLED_WR_Byte(0x81, OLED_CMD); /* 对比度设置 */
    OLED_WR_Byte(0xEF, OLED_CMD); /* 1~255, 默认0x7F (越大越亮) */

    OLED_WR_Byte(0xD9, OLED_CMD); /* 设置预充电周期 */
    OLED_WR_Byte(0xF1, OLED_CMD); /* [3:0]PHASE 1; [7:4]PHASE 2 */

    OLED_WR_Byte(0xDB, OLED_CMD); /* 设置VCOMH电压倍率 */
    OLED_WR_Byte(0x30, OLED_CMD); /* [6:4] 000=0.65*Vcc; 001=0.77*Vcc; 011=0.83*Vcc */

    OLED_WR_Byte(0xA4, OLED_CMD); /* 全局显示开启: bit0 — 1:开启; 0:关闭 (白屏/黑屏) */
    OLED_WR_Byte(0xA6, OLED_CMD); /* 设置显示方式: bit0 — 1:反相显示; 0:正常显示 */

    OLED_WR_Byte(0xAF, OLED_CMD); /* 开启显示 */

    OLED_Clear();
}
