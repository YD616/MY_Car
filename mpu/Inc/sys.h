#ifndef __SYS_H
#define __SYS_H
#include "stm32f1xx_hal.h"

// 类型别名（兼容标准库风格的旧代码）
typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint8_t   uchar;
typedef uint16_t  ushort;
typedef uint32_t  ulong;

// 位带操作宏（mpuiic.h IO口操控需要）
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr)        *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

// IO口地址映射
#define GPIOA_ODR_Addr  (GPIOA_BASE + 12)
#define GPIOB_ODR_Addr  (GPIOB_BASE + 12)
#define GPIOC_ODR_Addr  (GPIOC_BASE + 12)
#define GPIOD_ODR_Addr  (GPIOD_BASE + 12)
#define GPIOA_IDR_Addr  (GPIOA_BASE + 8)
#define GPIOB_IDR_Addr  (GPIOB_BASE + 8)
#define GPIOC_IDR_Addr  (GPIOC_BASE + 8)
#define GPIOD_IDR_Addr  (GPIOD_BASE + 8)

// IO口位带操作（mpuiic.h SDA/SCL引脚控制需要）
#define PAout(n)  BIT_ADDR(GPIOA_ODR_Addr, n)
#define PAin(n)   BIT_ADDR(GPIOA_IDR_Addr, n)
#define PBout(n)  BIT_ADDR(GPIOB_ODR_Addr, n)
#define PBin(n)   BIT_ADDR(GPIOB_IDR_Addr, n)
#define PCout(n)  BIT_ADDR(GPIOC_ODR_Addr, n)
#define PCin(n)   BIT_ADDR(GPIOC_IDR_Addr, n)
#define PDout(n)  BIT_ADDR(GPIOD_ODR_Addr, n)
#define PDin(n)   BIT_ADDR(GPIOD_IDR_Addr, n)

// 系统函数
void WFI_SET(void);
void INTX_DISABLE(void);
void INTX_ENABLE(void);
void MSR_MSP(u32 addr);

#endif
