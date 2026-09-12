#ifndef __APP_SENSOR_H
#define __APP_SENSOR_H

#include "sys.h"

// 将角度归一化到 [0, 360) 范围
float Angle_Normalize360(float angle);

// 将角度归一化到 [-180, 180) 范围（显示更直观）
float Angle_Normalize180(float angle);

// 死区判断 + 角度归一化：需要刷新返回1
u8 Display_NeedUpdate(float pitch, float roll, float yaw);

// 获取归一化后的显示角度（0~360）
float Display_GetP(void);
float Display_GetR(void);
float Display_GetY(void);

// 三个姿态角（由 main 中三个角度获取函数更新）
extern float pitch, roll, yaw;
// 直立参考绝对俯仰角（Sensor_Calibrate 记录，供直立环作目标角）
extern float pitch_upright;
// 加速度计解算角度（度）
extern float accel_pitch, accel_roll;
// 陀螺仪角速度（度/秒，已去除上电零偏）
extern float gx_f, gy_f, gz_f;
// 磁力计航向（度），-999 表示无有效参考
extern float yaw_mag;
// 磁力计数据有效性（供显示 M:OK/M:--）
extern u8 mag_ok;

// 上电校准：采集陀螺仪静止零偏；Pitch/Roll 初始值=当前重力绝对姿态，
// 直接跟随重力方向（不再以上电零位为 0）
void Sensor_Calibrate(void);

// 数据采集与预处理：实测dt + 读取加速度计/陀螺仪 + 单位换算 + 计算加速度计角度
// 供 main 中 Pitch/Roll 两个角度获取函数使用
void Sensor_Update(void);

// 磁力计航向计算与有效性判定
// 注意：须在 main 中 Pitch/Roll 角度获取函数调用之后再调用（倾斜补偿需要最新 pitch/roll）
void Sensor_UpdateMag(void);

// 角度整体更新：采集 + 卡尔曼融合(Pitch/Roll) + 磁力计航向(Yaw)
// 由 PA12 外部中断(HAL_GPIO_EXTI_Callback)触发调用，替代 main 循环中的轮询采样
void Sensor_Angle_Update(void);

#endif
