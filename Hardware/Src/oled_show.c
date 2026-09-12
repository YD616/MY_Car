/* OLED 显示应用层 (移植自 WHEELTEC B570 平衡小车 MiniBalance\show\show.c)
 * 移植要点:
 *   - 底层驱动不用 WHEELTEC 标准库版, 直接调用本项目 Hardware\oled.c (HAL + 软件SPI)
 *   - 原 oled_show() 依赖的 Mode/Voltage/Motor_Left 等变量本项目没有, 已替换为:
 *       姿态角 -> app_sensor 的 pitch/roll/yaw;  轮速 -> Encoder_GetSpeedL/R (cm/s)
 *   - main 中暂不调用, 需要时直接调用本文件接口
 * size 约定: 统一使用 OLED_6X8 / OLED_8X16 */
#include "oled_show.h"
#include "oled.h"          /* OLED 底层驱动 */
#include "sys.h"           /* u8 等类型别名 */
#include "app_config.h"    /* USE_MAG 等宏 */
#include "app_sensor.h"    /* pitch / roll / yaw / mag_ok */
#include "encode.h"        /* Encoder_GetSpeedL / Encoder_GetSpeedR */

/* ---- 内部辅助函数 ---- */
/* 显示带符号浮点数 (保留 1 位小数, 四舍五入); int_len 为整数位数(不含符号) */
static void oled_show_signed_1dec(uint8_t x, uint8_t y, float v, uint8_t int_len, uint8_t size)
{
    uint8_t font = (size == OLED_6X8) ? 12 : 16;
    uint8_t step = font / 2;

    int     sign  = (v < 0) ? -1 : 1;
    float   a     = v * (float)sign;                      /* 取绝对值 */
    int32_t whole = (int32_t)a;                           /* 整数部分 */
    int32_t dec   = (int32_t)((a - (float)whole) * 10.0f + 0.5f); /* 十分位, 四舍五入 */

    uint8_t xpos = x;

    /* 进位处理: 如 9.96 -> 10.0 */
    if (dec >= 10) {
        dec = 0;
        whole++;
    }

    /* 符号 */
    OLED_ShowChar(xpos, y, (sign < 0) ? '-' : '+', font, 1);
    xpos += step;

    /* 整数部分 */
    OLED_ShowNumber(xpos, y, (uint32_t)whole, int_len, font);
    xpos += step * int_len;

    /* 小数点 + 十分位 */
    OLED_ShowChar(xpos, y, '.', font, 1);
    xpos += step;
    OLED_ShowNumber(xpos, y, (uint32_t)dec, 1, font);
}

/* ---- 对外显示接口 ---- */

/* 开机欢迎画面 (移植自 WHEELTEC oled_show_once()) */
void OLED_Show_Welcome(void)
{
    OLED_Clear();
    OLED_ShowString(0,  0, "WHEELTEC OLED", OLED_8X16);
    OLED_ShowString(0, 20, "Port to MY Car", OLED_8X16);
    OLED_ShowString(0, 40, "Driver: SSD1306", OLED_8X16);
    OLED_Update();
}

/* 姿态角显示 (Pitch / Roll / Yaw, 16字体) */
void OLED_Show_Attitude(void)
{
    OLED_Clear();

    OLED_ShowString(0,  0, "P:", OLED_8X16);
    oled_show_signed_1dec(16,  0, pitch, 3, OLED_8X16);

    OLED_ShowString(0, 16, "R:", OLED_8X16);
    oled_show_signed_1dec(16, 16, roll,  3, OLED_8X16);

    OLED_ShowString(0, 32, "Y:", OLED_8X16);
    oled_show_signed_1dec(16, 32, yaw,   3, OLED_8X16);

#if USE_MAG
    OLED_ShowString(0, 48, "M:", OLED_8X16);
    if (mag_ok) OLED_ShowString(16, 48, "OK", OLED_8X16);
    else        OLED_ShowString(16, 48, "--", OLED_8X16);
#endif

    OLED_Update();
}

/* 遥测画面 (12字体: 姿态角 + 左右轮速) */
void OLED_Show_Telemetry(void)
{
    OLED_Clear();

    /* 第 0 行: 标题 */
    OLED_ShowString(0, 0, "MY Car", OLED_6X8);

    /* 第 1 行: Pitch / Roll */
    OLED_ShowString(0, 10, "P", OLED_6X8);
    oled_show_signed_1dec(6, 10, pitch, 3, OLED_6X8);
    OLED_ShowString(42, 10, "R", OLED_6X8);
    oled_show_signed_1dec(48, 10, roll, 3, OLED_6X8);

    /* 第 2 行: Yaw / 磁力计状态 */
    OLED_ShowString(0, 20, "Y", OLED_6X8);
    oled_show_signed_1dec(6, 20, yaw, 3, OLED_6X8);
#if USE_MAG
    OLED_ShowString(48, 20, "M", OLED_6X8);
    if (mag_ok) OLED_ShowString(54, 20, "OK", OLED_6X8);
    else        OLED_ShowString(54, 20, "--", OLED_6X8);
#endif

    /* 第 3 行: 左右轮速 (cm/s) */
    OLED_ShowString(0, 30, "L", OLED_6X8);
    oled_show_signed_1dec(6, 30, Encoder_GetSpeedL(), 3, OLED_6X8);
    OLED_ShowString(48, 30, "R", OLED_6X8);
    oled_show_signed_1dec(54, 30, Encoder_GetSpeedR(), 3, OLED_6X8);
    OLED_ShowString(96, 30, "cm/s", OLED_6X8);

    /* 第 4 行: 状态提示 */
    OLED_ShowString(0, 50, "OLED Ready", OLED_6X8);

    OLED_Update();
}
