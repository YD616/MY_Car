/* 串口通信 USART3 (蓝牙, 9600bps), printf 重定向到 USART3
 *   USART_SendBit / USART_SendString   发送
 *   USART_GetBit / USART_GetString     接收 (非阻塞)
 *   KPA_Get / KIA_Get / KDA_Get / LV_Get  单值数据包解析
 * 接收协议: 包结束符支持 "##" 或 "\r\n" */
#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdint.h>

/* ---- 蓝牙接收缓冲区 ---- */
#define BT_RX_BUF_SIZE    128   /* 蓝牙接收缓冲区大小 */
#define BT_CMD_MAX_LEN     32   /* 单条命令最大长度 */

/* 最近一包原始内容 (诊断用: 可显示到 OLED / 调试器查看, 确认数据是否到达) */
extern char Serial_RxDebug[BT_CMD_MAX_LEN + 1];

/* ---- 接收诊断计数器 (临时调试用) ----
 * Serial_RxByteCnt: 每进一次接收中断 +1 (含被丢弃的字节)
 * Serial_RxPktCnt : 每识别出一个完整包 (帧尾 "##" 或 "\r\n") +1
 * 显示到 OLED 可定位"收不到"的环节:
 *   RX 不涨        -> 一个字节都没进来: 模块 TX 未接 PB11 / 没配对 / App 未发出
 *   RX 涨,PK 不涨  -> 字节到了但帧尾没识别: App 发的不是 ## 也不是 \r\n
 *   RX 涨,PK 也涨  -> 接收正常, 问题在解析/取值侧 */
extern volatile uint32_t Serial_RxByteCnt;
extern volatile uint32_t Serial_RxPktCnt;

/* ---- 函数声明 ---- */
void Serial_Init(void);                                /* 使能中断 + 启动中断接收 + 重定向 printf */

void USART_SendBit(char c);                            /* 发送一个字符 */
void USART_SendString(const char *str);                /* 发送字符串 (不自动加换行) */

uint8_t  USART_GetBit(char *c);                        /* 接收一个字符 (非阻塞) */
uint16_t USART_GetString(char *buf, uint16_t maxlen);  /* 接收一个完整数据包, 返回长度, 无数据返回 0 */

uint8_t KPA_Get(float *value);                         /* "KPA:xx##" -> Kp */
uint8_t KIA_Get(float *value);                         /* "KIA:xx##" -> Ki */
uint8_t KDA_Get(float *value);                         /* "KDA:xx##" -> Kd */
uint8_t LV_Get (float *value);                         /* "LV:xx.xx##" -> 左轮目标速度 (cm/s) */

/* printf 重定向原型 (GCC / Keil 两种写法) */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#endif /* __SERIAL_H__ */
