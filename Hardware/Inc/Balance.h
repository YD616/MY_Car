/* 直立平衡控制器: 角度PD直立环
 * 格式与 PID.h 对齐(结构体 + Init/Compute/SetTunings), 由 main 每5ms调用一次
 *   1. Balance_Init(&balance, Kp, Ki, Kd, OUT_MAX);   // 上电1次
 *   2. Balance_SetSetpoint(&balance, pitch_upright);  // 设定直立目标角
 *   3. pwm = Balance_Compute(&balance, pitch);        // 每5ms 1次
 *   4. Balance_SetTunings(&balance, kp, ki, kd);      // 蓝牙在线调参 */
#ifndef __BALANCE_H__
#define __BALANCE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---- 直立环默认参数 ----
 * 可被蓝牙 KPA/KDA 在线调整 (每次上电按宏重新初始化, 调好后请固化到宏)
 * 目标角来源: app_sensor 校准记录的 pitch_upright (开机姿态的绝对俯仰)
 * 方向约定: 前倾(pitch > setpoint) → Compute 返回正 → main 映射 DIR_FWD;
 *           若实机相反, 翻转 app_config.h PITCH_FLIP */
#define BALANCE_KP         30.0f    /* 直立环 Kp: 倾斜1°对应的输出增量 */
#define BALANCE_KI         0.0f     /* 直立环 Ki: 纯PD置0, 留作抗静偏备用 */
#define BALANCE_KD         0.8f     /* 直立环 Kd: 角速度阻尼 */
#define BALANCE_OUT_MAX    7000.0f  /* 直立环输出限幅(±) */

/* ---- 控制器结构体 (与 PID_Controller 对齐) ---- */
typedef struct {
    float Kp;               /* 比例系数 */
    float Ki;               /* 积分系数 */
    float Kd;               /* 微分系数 */

    float setpoint;         /* 直立目标角(°): 校准记录的 pitch_upright */
    float input;            /* 当前输入角度 pitch(°) */
    float output;           /* 控制器输出 (带符号PWM, 前倾→正) */

    float error;            /* 直立误差: pitch - setpoint */
    float last_input;       /* 上次输入角度, 用于求角速度 */
    float integral;         /* 积分累积 (Ki一般置0) */

    float output_limit;     /* 输出限幅 (绝对值) */
    float integral_limit;   /* 积分限幅 (绝对值) */
    uint32_t last_tick;     /* 上次调用时间戳, 用于真实dt微分 */
} Balance_Controller;

/* ---- 全局实例 ---- */
extern Balance_Controller balance;   /* 直立环唯一实例 (main 每5ms调 Compute) */

/* ---- 函数声明 ---- */
void Balance_Init(Balance_Controller *bal, float Kp, float Ki, float Kd, float output_limit);   /* 初始化 */

/* 直立环单次计算: 输入当前绝对俯仰角(°), 返回带符号PWM (前倾→正)
 * 与通用 PID 库的区别: 微分项对"输入角速度"微分(带真实dt), 是标准角度PD,
 * 避免直立环正反馈; 需固定周期调用 */
float Balance_Compute(Balance_Controller *bal, float input);

void Balance_SetSetpoint(Balance_Controller *bal, float sp);                      /* 设直立目标角 */
void Balance_SetTunings(Balance_Controller *bal, float Kp, float Ki, float Kd);   /* 在线改参 */
void Balance_Reset(Balance_Controller *bal);                                      /* 清历史状态 */

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_H__ */
