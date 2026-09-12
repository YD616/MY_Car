/* OLED 显示应用层 (移植自 WHEELTEC B570 平衡小车 show.c)
 * 仅保留"显示应用层", 底层驱动复用本项目 Hardware\oled.c;
 * main 中暂不调用, 后续需要时直接调用下述接口即可 */
#ifndef __OLED_SHOW_H__
#define __OLED_SHOW_H__

/* ---- 显示接口 (各接口内部已含 OLED_Update() 上屏) ---- */
void OLED_Show_Welcome(void);     /* 开机欢迎画面 (品牌 + 说明) */
void OLED_Show_Attitude(void);    /* 姿态角显示: pitch/roll/yaw (app_sensor) */
void OLED_Show_Telemetry(void);   /* 遥测画面: 姿态角 + 左右轮速 (Encoder_GetSpeedL/R, cm/s) */

#endif /* __OLED_SHOW_H__ */
