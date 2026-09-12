/* 速度闭环控制模块 (直立内环 + 速度外环)
 *   内环: Balance_Control() 角度PD, 保证不倒
 *   外环: 左右轮速度PID, 维持目标速度(基础0 + 倾角操纵量)
 *   每轮驱动 = 直立环输出 + 速度环输出, 左右对称 */
#ifndef __SPEED_H__
#define __SPEED_H__

#include "main.h"

/* ---- 速度控制参数 (需实机整定) ---- */
#define SPEED_BASE_CM     0.0f    /* 基础目标速度: 直立静止为0 (cm/s) */
#define SPEED_LEAN_K     10.0f    /* 倾角操纵增益: 前倾1°→目标速度约+10cm/s (整定) */
#define SPEED_KP          4.0f    /* 速度环比例 (整定起点) */
#define SPEED_KI          0.02f   /* 速度环积分: 抑制静差/抗平移漂移 */
#define SPEED_KD          0.0f    /* 速度环一般不加D (编码器测速有量化噪声) */
#define SPEED_OUT_MAX   2000.0f   /* 速度环输出限幅(PWM档, 小于直立环限幅7000) */
#define SAFE_TILT_DEG    30.0f    /* 倾角超过此值判失稳 → 立即锁轮停车 */
#define PWM_DEADZONE      80      /* 合成输出死区: |输出|<此值锁轮 */

/* ---- 函数声明 ---- */
void  Speed_Init(void);           /* 上电初始化: 速度PID参数复位 + 状态清零 */
void  Speed_Enable(void);         /* 按键松开后使能速度闭环 (清PID历史防突跳) */
void  Speed_Disable(void);        /* 急停/失稳: 全部锁轮 */
void  Speed_Tick(void);           /* 控制主拍: main 每5ms调用一次(唯一电机驱动入口) */

float Speed_GetSpeedL(void);      /* 最近一版左轮实测速度(cm/s), 供OLED/上报复用 */
float Speed_GetSpeedR(void);      /* 最近一版右轮实测速度(cm/s) */
float Speed_GetTarget(void);      /* 当前目标速度(cm/s), 调试用 */

/* ---- 蓝牙在线调参: 右轮速度环 pid_speed_B ----
 * KPA/KIA/KDA 数据包(KPA:xx## 格式) → 只改右轮速度环对应单参数, 不清 PID 历史,
 * 供实机整定右轮速度环等效的 SPEED_KP/SPEED_KI/SPEED_KD 值 */
void Speed_SetRightKp(float kp);  /* 调右轮速度环 Kp */
void Speed_SetRightKi(float ki);  /* 调右轮速度环 Ki */
void Speed_SetRightKd(float kd);  /* 调右轮速度环 Kd */

/* 读取当前右轮速度环 PID 参数 (OLED 底部 KP/KI/KD 三参数行回显用) */
float Speed_GetRightKp(void);
float Speed_GetRightKi(void);
float Speed_GetRightKd(void);

#endif /* __SPEED_H__ */
