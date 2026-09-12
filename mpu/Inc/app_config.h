#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/*==================== 卡尔曼滤波参数 ====================*/
// 过程噪声（角度/角速度）与测量噪声，值越大越信任该来源
#define Q_ANGLE   0.001f    // 过程噪声（角度）
#define Q_GYRO    0.003f    // 过程噪声（角速度）
#define R_ANGLE   2.0f      // 测量噪声：电机振动会使加速度计解算角度带噪，
                            // 增大此值 → 更信任陀螺仪积分，抑制振动注入角度

/*==================== 显示相关 ====================*/
#define DEADBAND  0.05f     // OLED 刷新死区（度），变化小于该值不刷新

/*==================== 数学常量 ====================*/
#define DEG2RAD  0.01745329252f   // 度 → 弧度
#define RAD2DEG  57.29577951f     // 弧度 → 度

/*==================== 传感器量程 ====================*/
#define ACCEL_LSB_PER_G   16384.0f  // 加速度 ±2g 量程 (LSB/g)
#define GYRO_LSB_PER_DPS  16.4f     // 陀螺仪 ±2000dps 量程 (LSB/(°/s))

/*==================== 角度方向 ====================*/
// 传感器安装方向不同时翻转符号：1 或 -1
#define PITCH_FLIP  1.0f
#define YAW_FLIP    1.0f

/*==================== 磁力计 ====================*/
#define USE_MAG     1       // 1：启用磁力计航向融合；0：仅陀螺仪积分
#define MAG_WEIGHT  0.03f   // 磁力计航向融合权重（典型值0.03，推荐0.05）
                            // 过大(如0.5)会让磁力计单帧噪声大幅注入Yaw，电机干扰下表现为漂移

/*==================== DWT 周期计数器（dt 测量） ====================*/
#define DEMCR           (*(volatile unsigned long *)0xE000EDFC)  // Debug Exception and Monitor Control Register
#define TRCENA_BIT      (1UL << 24)                              // 使能 DWT/ITM 追踪
#define DWT_CYCCNT      (*(volatile unsigned long *)0xE0001004)  // 周期计数器
#define DWT_CR          (*(volatile unsigned long *)0xE0001000)  // DWT 控制寄存器
#define CYCCNTENA_BIT   (1UL << 0)                               // 使能周期计数器

#endif
