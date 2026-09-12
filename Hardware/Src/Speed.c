/* 速度闭环控制 (直立内环 + 速度外环), main 每 5ms 调用 Speed_Tick() 一次
 * 调用链: 编码器测速 → 直立环 Balance(角度PD) → 目标速度(基础0+倾角操纵)
 *         → 速度环 PID(左右对称) → 方向/限幅 → 电机
 * 符号约定(实机若反请翻 app_config.h PITCH_FLIP 或 encode.h ENCODER_DIR_FLIP):
 *   前倾(pitch>pitch_upright) → Balance 输出正 → DIR_FWD(轮向前); 前进时编码器为正 */
#include "Speed.h"
#include "Balance.h"
#include "Motor.h"
#include "encode.h"
#include "pid.h"
#include "app_sensor.h"
#include <math.h>

/* app_sensor.h 导出的姿态量: pitch(前倾为正) / pitch_upright(直立参考角) */

/* 复用在 PID.c 已定义/声明的两个速度环实例 (A=左轮, B=右轮; 接反则互换) */
extern PID_Controller pid_speed_A;
extern PID_Controller pid_speed_B;

static float   s_vL = 0.0f;       /* 最近一版实测左轮速度缓存 (cm/s) */
static float   s_vR = 0.0f;       /* 最近一版实测右轮速度缓存 (cm/s) */
static float   s_target = 0.0f;   /* 当前目标速度 */
static uint8_t s_enabled = 0;     /* 控制使能标志: 0=停车 */

/* ---- 内部: 单轮带符号 PWM 输出 ----
 * pwm>0 → DIR_FWD 前进; pwm<0 → DIR_BWD 后退; 死区内 → 锁轮制动 */
static void Motor_Drive(char wheel, float pwm)
{
    int16_t mag;

    if (pwm > -PWM_DEADZONE && pwm < PWM_DEADZONE) {   /* 死区: 锁轮 */
        Motor_Direction(wheel, DIR_STOP);
        Motor_PWM(wheel, 0);
        return;
    }

    if (pwm >= 0.0f) {
        Motor_Direction(wheel, DIR_FWD);
        mag = (pwm > PWM_MAX) ? PWM_MAX : (int16_t)pwm;
    } else {
        Motor_Direction(wheel, DIR_BWD);
        pwm = -pwm;
        mag = (pwm > PWM_MAX) ? PWM_MAX : (int16_t)pwm;
    }
    Motor_PWM(wheel, mag);
}

/* ---- 初始化 ---- */
void Speed_Init(void)
{
    /* 左右轮各一个速度PID, 参数一致保证左右对称 */
    PID_Init(&pid_speed_A, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_OUT_MAX);
    PID_Init(&pid_speed_B, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_OUT_MAX);
    PID_SetSetpoint(&pid_speed_A, SPEED_BASE_CM);
    PID_SetSetpoint(&pid_speed_B, SPEED_BASE_CM);

    s_vL = s_vR = s_target = 0.0f;
    s_enabled = 0;
}

/* ---- 使能 / 急停 ---- */
void Speed_Enable  (void)
{
    PID_Reset(&pid_speed_A);   /* 清积分等历史, 防启动瞬间突跳 */
    PID_Reset(&pid_speed_B);
    s_enabled = 1;
}

void Speed_Disable(void)
{
    s_enabled = 0;
    Motor_Direction('L', DIR_STOP);   /* 锁轮制动 */
    Motor_Direction('R', DIR_STOP);
    Motor_PWM('L', 0);
    Motor_PWM('R', 0);
}

/* ---- 5ms 控制主拍 ---- */
void Speed_Tick(void)
{
    float vL, vR, lean, bal, uL, uR, v_ref, pwmL, pwmR;

    if (!s_enabled)   /* 未使能: 维持停车, 不采样编码器 */
        return;

    /* 失稳保护: 倾角超限立即锁轮停车 */
    lean = pitch - pitch_upright;
    if (fabsf(lean) > SAFE_TILT_DEG) {
        Speed_Disable();
        return;
    }

    /* ① 编码器实测轮速 (本拍只采一次, OLED/上报用缓存) */
    vL = Encoder_GetSpeedL();
    vR = Encoder_GetSpeedR();
    s_vL = vL;
    s_vR = vR;

    /* ② 内环: 直立角度PD — 倾角大小/方向 → 基础驱动力 */
    bal = Balance_Control();

    /* ③ 外环目标速度: 基础0 + 倾角操纵量 (前倾→正向前, 后仰→负向后) */
    v_ref = SPEED_BASE_CM + SPEED_LEAN_K * lean;
    s_target = v_ref;
    PID_SetSetpoint(&pid_speed_A, v_ref);
    PID_SetSetpoint(&pid_speed_B, v_ref);

    /* ④ 速度环: 左右对称 PID 维持目标速度 (假定固定 5ms 调用周期) */
    uL = PID_Compute(&pid_speed_A, vL);
    uR = PID_Compute(&pid_speed_B, vR);

    /* ⑤ 合成并输出: 两环同号相加 (正=前倾追重心=前进) */
    pwmL = bal + uL;
    pwmR = bal + uR;
    Motor_Drive('L', pwmL);
    Motor_Drive('R', pwmR);
}

/* ---- 状态读取 (供 OLED / 蓝牙上报) ---- */
float Speed_GetSpeedL(void) { return s_vL; }
float Speed_GetSpeedR(void) { return s_vR; }
float Speed_GetTarget(void) { return s_target; }

/* ---- 蓝牙在线调参: 右轮速度环 ----
 * 直接改 pid_speed_B 系数(结构体字段公开), 不清积分/误差历史, 参数即时生效 */
void Speed_SetRightKp(float kp) { pid_speed_B.Kp = kp; }
void Speed_SetRightKi(float ki) { pid_speed_B.Ki = ki; }
void Speed_SetRightKd(float kd) { pid_speed_B.Kd = kd; }

/* 读取当前参数: 返回 pid_speed_B 当前生效值 (蓝牙调参后立即反映) */
float Speed_GetRightKp(void) { return pid_speed_B.Kp; }
float Speed_GetRightKi(void) { return pid_speed_B.Ki; }
float Speed_GetRightKd(void) { return pid_speed_B.Kd; }
