/* 直立平衡控制器: 角度PD直立环
 * 格式与 PID.c 对齐(结构体 + Init/Compute/SetTunings), 算法为直立环专用:
 *   - 角度模型: pitch 是"跟随重力的绝对角"(平放≈0, 竖直≈±90), 直立目标角不是固定 0,
 *     而是校准时记录的 pitch_upright (由 Balance_SetSetpoint 设置)
 *   - 微分项用"角速度"(对输入求差/真实dt), 而非通用库的误差差分:
 *     通用 PID 的微分直接套到直立环会产生正反馈, 故此处独立实现
 *   - 方向: 前倾(input > setpoint) → 输出为正 → main 映射 DIR_FWD (朝倾倒方向追重心) */
#include "Balance.h"
#include <string.h>

/* ---- 全局直立环实例 ---- */
Balance_Controller balance;   /* 直立环唯一实例 */

/* ---- 直立环初始化 ---- */
/* 写入 Kp/Ki/Kd 与输出限幅, 状态清零; 每次上电按宏调用一次 */
void Balance_Init(Balance_Controller *bal, float Kp, float Ki, float Kd, float output_limit)
{
    memset(bal, 0, sizeof(Balance_Controller));
    bal->Kp = Kp;
    bal->Ki = Ki;
    bal->Kd = Kd;
    bal->output_limit   = output_limit;
    bal->integral_limit = output_limit * 0.5f;   /* 积分限幅默认输出的50% */
}

/* ---- 直立环计算 (每个控制周期调用一次) ----
 * 用真实dt(HAL_GetTick 实测, 带上下限保护), 调用周期允许一定抖动;
 * 需先 Balance_Init, 且目标角需 Balance_SetSetpoint 设定 */
float Balance_Compute(Balance_Controller *bal, float input)
{
    float dt   = 0.005f;            /* 兜底: 主循环约5ms */
    float err, omega, out;
    uint32_t now = HAL_GetTick();

    bal->input = input;

    /* 实测调用周期: ms→s (微分/积分用真实dt) */
    if (bal->last_tick != 0)
        dt = (float)(now - bal->last_tick) / 1000.0f;
    bal->last_tick = now;
    if (dt < 0.001f) dt = 0.001f;   /* 保护: 最小1ms */
    if (dt > 0.05f)  dt = 0.05f;    /* 保护: 最大50ms */

    err = input - bal->setpoint;    /* 直立误差: 前倾为正 */
    bal->error = err;

    omega = (input - bal->last_input) / dt;   /* 角速度: 前倾为正 */
    bal->last_input = input;

    /* 积分项 (带限幅抗饱和, Ki 默认0不用) */
    bal->integral += err * dt;
    if (bal->integral >  bal->integral_limit) bal->integral =  bal->integral_limit;
    if (bal->integral < -bal->integral_limit) bal->integral = -bal->integral_limit;

    /* 合成输出: P回正 + I抗静偏 + D角速度阻尼 */
    out  = bal->Kp * err;
    out += bal->Ki * bal->integral;
    out += bal->Kd * omega;

    if (out >  bal->output_limit) out =  bal->output_limit;
    if (out < -bal->output_limit) out = -bal->output_limit;

    bal->output = out;
    return out;                     /* 正=前倾追重心(DIR_FWD) */
}

/* ---- 辅助函数 ---- */
void Balance_SetSetpoint(Balance_Controller *bal, float sp)   /* 设直立目标角(校准记录值) */
{
    bal->setpoint = sp;
}

void Balance_SetTunings(Balance_Controller *bal, float Kp, float Ki, float Kd)   /* 在线改参, 不重置历史 */
{
    bal->Kp = Kp;
    bal->Ki = Ki;
    bal->Kd = Kd;
}

void Balance_Reset(Balance_Controller *bal)   /* 清历史状态(积分/上次输入/时间戳), 保留参数 */
{
    bal->error      = 0.0f;
    bal->input      = 0.0f;
    bal->integral   = 0.0f;
    bal->last_input = 0.0f;
    bal->output     = 0.0f;
    bal->last_tick  = 0;
}
