/* 编码器测速模块 (B570 霍尔编码减速电机): 电机参数集中在此, 速度接口统一返回 cm/s */
#ifndef __ENCODE_H__
#define __ENCODE_H__

#include "main.h"
#include "tim.h"

/* ---- 编码器硬件参数 ---- */
/* B570 霍尔编码减速电机: 线数 11 (单相每转), 4 倍频 (TI12 两相上下沿均计数), 减速比 1:30
 * 轮子每转计数 = 11 x 4 x 30 = 1320 */
#define ENCODER_LINES       11                 /* 编码器每转脉冲数(单相) */
#define ENCODER_MULTIPLIER  4                  /* STM32 编码器接口 4 倍频 */
#define GEAR_RATIO          30                 /* 减速比 1:30 */
#define ENCODER_RESOLUTION  (ENCODER_LINES * ENCODER_MULTIPLIER * GEAR_RATIO)   /* =1320 轮子每转计数 */

/* ---- 车轮参数 ---- */
#define WHEEL_DIAMETER_CM   6.5f               /* 轮子直径 (cm) */
#define WHEEL_PERIMETER_CM  (3.14159265358979f * WHEEL_DIAMETER_CM)     /* 约 20.42 cm */

/* 速度校准系数(可选微调): 实测偏大就调小, 偏小就调大, 默认 1.0 */
#define ENCODER_CALIBRATION 1.0f

/* 测速方向翻转: 编码器 A/B 相接反导致速度符号反时置 1 (0=不翻转) */
#define ENCODER_DIR_FLIP    1

/* 左右编码器映射: 具体"轮 -> 定时器"集中在 encode.c 的 ENC_TIM_L / ENC_TIM_R 两行
 * 注意: 此处原注释称"实机 TIM4 接左轮、TIM3 接右轮", 与 encode.c 当前的
 *       L->TIM3 / R->TIM4 定义相反, 调整前需先确认实机接线 */
#define SPEED_SAMPLE_MS     10                 /* 默认采样周期(ms), 实际测速用真实时间戳 */

/* ---- 数据接口 ---- */
void  Encoder_Init(void);        /* 启动 TIM3/TIM4 编码器接口 (main 中调用一次) */
float Encoder_GetSpeedL(void);   /* 左轮速度 (cm/s) */
float Encoder_GetSpeedR(void);   /* 右轮速度 (cm/s) */

#endif /* __ENCODE_H__ */
