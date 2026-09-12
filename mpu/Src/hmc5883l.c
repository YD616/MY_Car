#include "hmc5883l.h"
#include "mpu6050.h"
#include "mpuiic.h"

/**
 * HMC5883L 磁力计驱动
 * 使用与MPU6050共同的I2C总线 (PB8=SCL, PB9=SDA)
 * HMC5883L I2C地址: 0x1E
 */

/**************************实现函数********************************************
*函 数 名：HMC5883L_Write_Reg
*功    能：写HMC5883L寄存器
*******************************************************************************/
static void HMC5883L_Write_Reg(u8 reg, u8 data)
{
    u8 buf = data;
    MPU_Write_Len(HMC5883L_ADDR, reg, 1, &buf);
}

/**************************实现函数********************************************
*函 数 名：HMC5883L_Read_Buf
*功    能：读取HMC5883L多个寄存器
*******************************************************************************/
static u8 HMC5883L_Read_Buf(u8 reg, u8 *buf, u8 len)
{
    return MPU_Read_Len(HMC5883L_ADDR, reg, len, buf);
}

/**************************实现函数********************************************
*函 数 名：HMC5883L_Init
*功    能：初始化HMC5883L
*返 回 值：0-成功
*说    明：采样平均8次, 75Hz输出率, ±1.9Ga量程, 连续转换模式
*******************************************************************************/
u8 HMC5883L_Init(void)
{
    /* Config Reg A: 8次采样平均 + 75Hz输出率 + 正常测量模式 */
    HMC5883L_Write_Reg(HMC5883L_CONFIG_A,
        MA_SAMPLE_AVG_8 | DO_RATE_75HZ | MA_MEASURE_NORMAL);

    /* Config Reg B: ±1.9Ga 量程 */
    HMC5883L_Write_Reg(HMC5883L_CONFIG_B, GAIN_1_9GA);

    /* Mode: 连续转换 */
    HMC5883L_Write_Reg(HMC5883L_MODE_REG, MODE_CONTINUOUS);

    return 0;
}

/**************************实现函数********************************************
*函 数 名：HMC5883L_Read
*功    能：读取三轴磁力计数据
*参    数：mx, my, mz: 三轴磁场原始值（有符号16位）
*返 回 值：0-成功
*******************************************************************************/
u8 HMC5883L_Read(short *mx, short *my, short *mz)
{
    u8 buf[6];
    u8 res;

    /* 连续读两次，丢弃第一次：
     * 主循环约200Hz读取，而HMC5883L数据75Hz更新(≈13.3ms)，跨更新边界
     * 读取会读到新旧混合的撕裂数据（mag_norm突变，表现为 M:OK/M:--
     * 频繁跳变）。第一次读取用于丢弃可能撕裂的帧，第二次数据才一致。 */
    res = HMC5883L_Read_Buf(HMC5883L_DATA_X_H, buf, 6);
    if (res) return 1;
    res = HMC5883L_Read_Buf(HMC5883L_DATA_X_H, buf, 6);
    if (res) return 1;

    /* 从数据寄存器开始连续读6字节: X_H, X_L, Z_H, Z_L, Y_H, Y_L */
    *mx = ((short)buf[0] << 8) | buf[1];
    *mz = ((short)buf[2] << 8) | buf[3];
    *my = ((short)buf[4] << 8) | buf[5];

    return 0;
}

/**************************实现函数********************************************
*函 数 名：HMC5883L_Read_ID
*功    能：读取HMC5883L设备ID寄存器
*返 回 值：设备ID (应为 'H', '4', '3')
*******************************************************************************/
u8 HMC5883L_Read_ID(void)
{
    u8 id[3];
    HMC5883L_Read_Buf(HMC5883L_ID_A, id, 3);
    /* 返回ID_A, 正常应为 'H'=0x48 */
    return id[0];
}
