/* 串口通信 USART3 (蓝牙, 9600bps), printf 调试输出同走 USART3
 * 收发接口:
 *   USART_SendBit(c)             发送一个字符
 *   USART_SendString(str)        发送字符串
 *   USART_GetBit(&c)             接收一个字符 (非阻塞)
 *   USART_GetString(buf,len)     接收一个完整数据包 (非阻塞)
 *   KPA_Get/KIA_Get/KDA_Get(&v)  接收 "KPA:xx##" / "KIA:xx##" / "KDA:xx##"
 *   SV_Get(&v)                   接收数据包 "SV:xx.xx##"
 * 接收协议: 包结束符支持 "##" 或 "\r\n", 中断按包缓存, 调用时取最近一包
 * 例: "KPA:1.21##" -> KPA_Get 返回 1, *v = 1.21
 * 多格式分发: 各 Get 可被主循环依次轮询, 内部带"未匹配回滚缓存", 不会互相吃掉对方的包 */
#include "serial.h"
#include <string.h>
#include <stdio.h>

/* ---- 内部状态 ---- */
static uint8_t           bt_rx_buf[BT_RX_BUF_SIZE];  /* USART3 接收缓冲区 */
static volatile uint16_t bt_rx_len = 0;              /* 已接收字节数 */
static volatile uint8_t  bt_rx_ready = 0;            /* 收到完整一行的标志 */
static uint8_t           bt_rx_byte = 0;             /* 中断单字节中转(固定地址, 防与主循环 len 竞争) */

/* 主循环中依次轮询的单值解析函数总数 (KPA/KIA/KDA/SV):
 * 数据包被全部函数尝试过仍不匹配, 才判定为未知包丢弃.
 * !!! 注意: 以后新增/删除解析函数时必须同步改此值, 否则排最后的函数会收不到包 !!! */
#define SERIAL_SINGLE_GET_COUNT   4

static char    bt_pending_line[BT_CMD_MAX_LEN + 1];  /* 未匹配数据包回滚缓存 */
static uint8_t bt_pending_valid = 0;                 /* 回滚缓存有效标志 */
static uint8_t bt_pending_try  = 0;                  /* 当前包已被尝试过的解析函数数(≥SERIAL_SINGLE_GET_COUNT 视为未知包丢弃) */

/* 最近一包原始内容 (诊断用: 可在 OLED/调试器查看, 确认数据是否到达及格式) */
char Serial_RxDebug[BT_CMD_MAX_LEN + 1];

/* ---- printf 重定向到 USART3 ---- */
#ifdef __GNUC__
int __io_putchar(int ch)
#else
/* 禁用半主机模式: Keil 标准库 printf 首次调用会走 _sys_open (BKPT), 脱离调试器会死机.
 * 重定向到 no-semihosting 后 printf -> fputc -> USART3 */
#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;

int fputc(int ch, FILE *f)
#endif
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

#ifndef __GNUC__
/* no-semihosting 库要求的替换函数 */
void _sys_exit(int x) { x = x; }
void _ttywrch(int ch) { ch = ch; }
int  ferror(FILE *f)  { (void)f; return 0; }
#endif

/* ---- 初始化 ---- */
void Serial_Init(void)
{
    memset(bt_rx_buf, 0, sizeof(bt_rx_buf));
    memset(Serial_RxDebug, 0, sizeof(Serial_RxDebug));
    bt_rx_len = 0;
    bt_rx_ready = 0;
    bt_pending_valid = 0;
    bt_rx_byte = 0;

    /* 使能 USART3 中断 (NVIC 层), 否则 HAL_UART_Receive_IT 收不到数据 */
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* 启动单字节中断接收: 固定收进 bt_rx_byte 再入缓存, 避免接收地址依赖
     * bt_rx_len 与主循环清零竞争导致丢字节 */
    HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
}

/* ---- USART3 发送函数 ---- */
/* 发送一个字符 (不自动加换行) */
void USART_SendBit(char c)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&c, 1, HAL_MAX_DELAY);
}

/* 发送字符串 (不自动加换行; 需要换行时请自行追加 "\r\n") */
void USART_SendString(const char *str)
{
    if (str == NULL) return;
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/* ---- USART3 接收函数 ---- */
/* 取最近一行 (static 内部实现): 返回行长度(不含结尾 \r\n), 无数据返回 0 */
static uint16_t serial_rx_line(char *buf, uint16_t maxlen)
{
    uint16_t len;

    if (buf == NULL || maxlen == 0) return 0;
    if (!bt_rx_ready) return 0;
    bt_rx_ready = 0;

    /* 临界区: 拷贝期间屏蔽中断, 防止新数据覆盖缓冲 */
    __disable_irq();
    len = (bt_rx_len < maxlen - 1) ? bt_rx_len : (maxlen - 1);
    memcpy(buf, bt_rx_buf, len);
    memcpy(Serial_RxDebug, bt_rx_buf, len);   /* 同步到诊断缓存, 供 OLED/调试查看 */
    Serial_RxDebug[len] = '\0';
    memset(bt_rx_buf, 0, BT_RX_BUF_SIZE);
    bt_rx_len = 0;
    __enable_irq();
    buf[len] = '\0';
    return len;
}

/* ---- 单值数据包解析 (KPA/KIA/KDA/SV) ---- */
/* 取一行: 优先返回"未匹配回滚缓存", 其次取新数据包 (static 内部函数) */
static uint16_t serial_rx_line_keep(char *buf, uint16_t maxlen)
{
    uint16_t len;

    if (buf == NULL || maxlen == 0) return 0;

    /* 缓存中的包已被全部解析函数依次尝试过仍未匹配: 判定为未知格式直接丢弃,
     * 让后续新包立即被处理 (防止垃圾包在各 Get 间无限回滚占住缓存) */
    if (bt_pending_valid && bt_pending_try >= SERIAL_SINGLE_GET_COUNT) {
        bt_pending_valid = 0;
        bt_pending_try  = 0;
    }

    if (bt_pending_valid) {
        bt_pending_try++;                     /* 缓存中的包: 已被多一个函数尝试 */
        len = (uint16_t)strlen(bt_pending_line);
        if (len > maxlen - 1) len = maxlen - 1;
        memcpy(buf, bt_pending_line, len);
        buf[len] = '\0';
        bt_pending_valid = 0;
        return len;
    }

    bt_pending_try = 1;                       /* 新包: 首次尝试 */
    return serial_rx_line(buf, maxlen);
}

/* 将一行保存到回滚缓存, 供下一个解析函数尝试匹配 (static 内部函数) */
static void serial_rx_line_save(const char *line)
{
    if (line == NULL) return;
    strncpy(bt_pending_line, line, BT_CMD_MAX_LEN);
    bt_pending_line[BT_CMD_MAX_LEN] = '\0';
    bt_pending_valid = 1;              /* 尝试计数 bt_pending_try 保持不变 */
}

/* 通用单值解析: 严格匹配 "PREFIX:value" 格式 (static 内部函数)
 * 返回 1=匹配并输出; 0=无数据或格式不匹配 (此时数据包回滚缓存, 不丢失) */
static uint8_t serial_get_single(const char *prefix, float *value)
{
    char  line[BT_CMD_MAX_LEN + 1];
    char *p;
    long  ipart = 0, dpart = 0, mult = 1;
    int   sign = 1;

    if (value == NULL || prefix == NULL) return 0;

    if (serial_rx_line_keep(line, sizeof(line)) == 0) return 0;

    p = line;
    if (*p == '#') p++;               /* 可选 '#' 前缀, 兼容 "#KPA:xx" 旧格式 */
    if (strncmp(p, prefix, strlen(prefix)) != 0) { serial_rx_line_save(line); return 0; }
    p += strlen(prefix);
    if (*p++ != ':') { serial_rx_line_save(line); return 0; }

    /* 符号 */
    if (*p == '-') { sign = -1; p++; }
    else if (*p == '+') p++;

    /* 整数部分: 逐位累加 (纯整数运算, 不依赖 sscanf 的 %f) */
    if (*p < '0' || *p > '9') { serial_rx_line_save(line); return 0; }
    while (*p >= '0' && *p <= '9') {
        ipart = ipart * 10 + (*p - '0');
        p++;
    }

    /* 小数部分: 逐位累加, 记录位数 */
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            dpart = dpart * 10 + (*p - '0');
            mult *= 10;
            p++;
        }
    }

    /* 行尾容忍: 空格/回车/换行/残留 '#' (双保险) */
    while (*p == ' ' || *p == '\r' || *p == '\n' || *p == '#') p++;
    if (*p != '\0') { serial_rx_line_save(line); return 0; }

    /* 组合浮点值: 整数 + 小数/10^n (double 运算保证精度, 转 float 存储) */
    *value = (float)((double)ipart + (double)dpart / (double)mult);
    if (sign < 0) *value = -*value;
    return 1;
}

/* 接收一个字符 (非阻塞): 从最近缓存的一行中逐字节取出, 无数据返回 0 */
uint8_t USART_GetBit(char *c)
{
    static uint16_t pos = 0;
    static char     line[BT_CMD_MAX_LEN + 1];
    static uint8_t  have_line = 0;

    if (c == NULL) return 0;

    /* 当前行已取完, 尝试取下一行 */
    if (!have_line) {
        if (serial_rx_line(line, sizeof(line)) == 0) return 0;
        pos = 0;
        have_line = 1;
    }

    if (pos < (uint16_t)strlen(line)) {
        *c = line[pos++];
        return 1;
    }
    have_line = 0;               /* 一行取完 */
    return 0;
}

/* 接收一个完整数据包 (非阻塞): 返回长度(不含结尾 ##), 无数据返回 0 */
uint16_t USART_GetString(char *buf, uint16_t maxlen)
{
    /* 与 KPA_Get/KIA_Get/KDA_Get 共享"回滚缓存", 保证 #PIDA 整包在三个
     * 单值函数依次轮询不匹配后, 能在这里被取到 (否则 PIDA 包会卡死缓存) */
    return serial_rx_line_keep(buf, maxlen);
}

/* ---- 单个 PID 参数 / 目标速度接收 ---- */
uint8_t KPA_Get(float *value) { return serial_get_single("KPA", value); }   /* "KPA:xx##" */
uint8_t KIA_Get(float *value) { return serial_get_single("KIA", value); }   /* "KIA:xx##" */
uint8_t KDA_Get(float *value) { return serial_get_single("KDA", value); }   /* "KDA:xx##" */
uint8_t SV_Get (float *value) { return serial_get_single("SV",  value); }   /* "SV:xx.xx##" */

/* ---- USART3 接收中断回调 ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uint8_t c = bt_rx_byte;   /* 从固定地址读取刚收到的字节, 不依赖 bt_rx_len */

        /* 若上一包尚未被主循环取走, 丢弃新字节 (防止覆盖有效数据) */
        if (bt_rx_ready) {
            HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
            return;
        }

        /* 帧尾判定:
         * 1) 连续两个 '#' 表示一包结束: 第二个 '#' 不入缓存, 只需剥掉已存入的前一个 '#',
         *    保证 "KPA:3.50##" 存成 "KPA:3.50"; 这里只能剥一个, 多剥会吃掉末位数字
         * 2) '\r' 或 '\n' 均表示行结束, 兼容 \r\n / \n / \r 三种帧尾
         *    (末尾是 '\r\n' 则剥掉 '\r') */
        if (c == '#' && bt_rx_len > 0 && bt_rx_buf[bt_rx_len - 1] == '#') {
            bt_rx_len--;                    /* 剥掉已存入的前一个 '#' */
            bt_rx_ready = 1;                /* 数据包接收完成 */
        } else if (c == '\n' || c == '\r') {
            if (c == '\n' && bt_rx_len > 0 && bt_rx_buf[bt_rx_len - 1] == '\r') {
                bt_rx_len--;                /* 去掉包尾的 '\r' */
            }
            bt_rx_ready = 1;                /* 数据包接收完成 */
        } else if (bt_rx_len < BT_RX_BUF_SIZE - 1) {
            bt_rx_buf[bt_rx_len++] = c;     /* 存入接收缓冲区 */
        } else {
            bt_rx_len = 0;                  /* 溢出复位 */
        }
        /* 继续接收下一个字节 (固定地址, 与主循环取包清零互不干扰) */
        HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
    }
}
