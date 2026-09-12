/* 电机驱动: PWM 由 TIM1 产生(10kHz), 方向由 GPIO 控制 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "tim.h"
#include "pid.h"
#include "encode.h"

/* ---- 硬件引脚定义 ---- */
/* 已按实机接线对调: 'L' 对应物理左轮(PA8/CH1), 保证 pid_speed_L/VL 语义一致 */
/* 电机L (左) */
#define MOTOR_L_PWM_CH      TIM_CHANNEL_1   /* PA8  - TIM1_CH1 */
#define MOTOR_L_IN1_PORT    GPIOB
#define MOTOR_L_IN1_PIN     GPIO_PIN_14
#define MOTOR_L_IN2_PORT    GPIOB
#define MOTOR_L_IN2_PIN     GPIO_PIN_15

/* 电机R (右) */
#define MOTOR_R_PWM_CH      TIM_CHANNEL_4   /* PA11 - TIM1_CH4 */
#define MOTOR_R_IN1_PORT    GPIOB
#define MOTOR_R_IN1_PIN     GPIO_PIN_13
#define MOTOR_R_IN2_PORT    GPIOB
#define MOTOR_R_IN2_PIN     GPIO_PIN_12

/*电机方向*/
#define DIR_FWD  0   //forward 前进
#define DIR_BWD  1   //backward 后退
#define DIR_STOP 2   //stop 刹车制动
/* PWM 最大占空比值 (TIM1 ARR = 7199) */
#define PWM_MAX             7200

/* ---- 电机控制模式 ---- */
typedef enum {
    MOTOR_MODE_STOP  = 0,   /* 停止 */
    MOTOR_MODE_OPEN  = 1,   /* 开环控制 (直接PWM) */
    MOTOR_MODE_SPEED = 2,   /* 速度闭环 */
    MOTOR_MODE_POS   = 3    /* 位置闭环 */
} Motor_Mode;

/* ---- 函数声明 ---- */
void Motor_Direction(char position, int8_t direction);   /* 'L'/'R' + DIR_FWD/DIR_BWD/其他=刹车 */
void Motor_Init(void);                                    /* 启动 TIM1 PWM */
void Motor_PWM(char position, int16_t speed);             /* 占空比 0~PWM_MAX */

#endif /* __MOTOR_H__ */
