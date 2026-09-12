#include "mpu6050.h"
#include "mpuiic.h"
#include "Delay.h"
#include <math.h>
#include <stdio.h>

/**
 * MPU6050 卡尔曼滤波姿态解算
 * 使用卡尔曼滤波器融合加速度计和陀螺仪数据
 */

// 卡尔曼滤波参数
#define CALIBRATION_SAMPLES  500
#define Q_ANGLE   0.002f   // 过程噪声（角度）
#define Q_GYRO    0.002f   // 过程噪声（角速度）
#define R_ANGLE   0.05f    // 测量噪声

// 角度输出
float Pitch = 0, Roll = 0, Yaw = 0;

// 陀螺仪零偏
static float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

// 卡尔曼滤波结构体
typedef struct {
    float Q_angle;    // 过程噪声协方差（角度）
    float Q_gyro;     // 过程噪声协方差（角速度）
    float R_angle;    // 测量噪声协方差
    float x_angle;    // 角度估计
    float x_bias;     // 零偏估计
    float P[2][2];    // 协方差矩阵
} Kalman_t;

// 三轴卡尔曼滤波器实例
static Kalman_t kalman_pitch = {Q_ANGLE, Q_GYRO, R_ANGLE, 0, 0, {{0, 0}, {0, 0}}};
static Kalman_t kalman_roll  = {Q_ANGLE, Q_GYRO, R_ANGLE, 0, 0, {{0, 0}, {0, 0}}};
static Kalman_t kalman_yaw   = {Q_ANGLE, Q_GYRO, R_ANGLE, 0, 0, {{0, 0}, {0, 0}}};

// 上次时间戳
static uint32_t last_time = 0;

/**************************内部函数********************************************
*函 数 名：Kalman_Init
*功    能：初始化卡尔曼滤波器
*******************************************************************************/
static void Kalman_Init(Kalman_t *kf)
{
    kf->Q_angle = Q_ANGLE;
    kf->Q_gyro = Q_GYRO;
    kf->R_angle = R_ANGLE;
    kf->x_angle = 0;
    kf->x_bias = 0;
    kf->P[0][0] = 0;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 0;
}

/**************************内部函数********************************************
*函 数 名：Kalman_Update
*功    能：卡尔曼滤波更新
*参    数：kf-滤波器指针, newAngle-加速度计角度, newRate-陀螺仪角速度, dt-时间间隔
*返 回 值：滤波后的角度
*******************************************************************************/
static float Kalman_Update(Kalman_t *kf, float newAngle, float newRate, float dt)
{
    float dt2 = dt * dt;

    // 预测步骤
    kf->x_angle += dt * (newRate - kf->x_bias);
    kf->P[0][0] += dt2 * kf->P[1][1] - dt * kf->P[0][1] - dt * kf->P[1][0] + dt2 * kf->Q_gyro;
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_gyro * dt;

    // 更新步骤
    float S = kf->P[0][0] + kf->R_angle;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    float y = newAngle - kf->x_angle;
    kf->x_angle += K0 * y;
    kf->x_bias += K1 * y;

    kf->P[0][0] -= K0 * kf->P[0][0];
    kf->P[0][1] -= K0 * kf->P[0][1];
    kf->P[1][0] -= K1 * kf->P[0][0];
    kf->P[1][1] -= K1 * kf->P[0][1];

    return kf->x_angle;
}

/**************************实现函数********************************************
*函 数 名：MPU_Init
*功    能：初始化MPU6050
*返 回 值：0-成功，其他-失败
*******************************************************************************/
u8 MPU_Init(void)
{
    u8 res;

    // 初始化I2C（MPU6050 INT 已接 PA12，EXTI 中断在 gpio.c 中配置，
    // 此处不再占用 PA15 —— PA15 是 OLED 的 DC 引脚，由 main.c 配置为输出）
    MPU_IIC_Init();

    // 复位MPU6050
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80);
    Delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);

    // 配置参数
    MPU_Set_Gyro_Fsr(3);      // ±2000dps
    MPU_Set_Accel_Fsr(0);     // ±2g
    MPU_Set_Rate(50);          // 50Hz
    MPU_Set_LPF(10);           // DLPF带宽降到10Hz：滤除电机振动高频分量
                               //（MPU_Set_Rate内部默认25Hz带宽，此处覆盖为10Hz）
    MPU_Write_Byte(MPU_INT_EN_REG, 0x01);   // 使能DATA_RDY数据就绪中断：PA12(EXTI下降沿)触发角度更新
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0x00);  // I2C_MST_EN=0：关闭I2C主机模式，MPU6050不占用软件I2C总线
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0x00);
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0x80);  // INT_PIN_CFG: bit7 INT_LEVEL=1：INT低有效
                                               // (平时高电平, 数据就绪时拉低产生下降沿, 匹配PA12 IT_FALLING)
                                               // bit1 I2C_BYPASS_EN=0：关闭I2C旁路

    // 检测设备ID
    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res == MPU_ADDR) {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x01); // PLL X轴
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0x00);
        MPU_Set_Rate(50);
        MPU_Set_LPF(10);   // DLPF 10Hz（此处最终生效，覆盖前面的25Hz设置）
    } else {
        return 1;
    }

    // 初始化卡尔曼滤波器
    Kalman_Init(&kalman_pitch);
    Kalman_Init(&kalman_roll);
    Kalman_Init(&kalman_yaw);

    return 0;
}

/**************************实现函数********************************************
*函 数 名：MPU6050_Calibrate
*功    能：陀螺仪零偏校准（设备需保持静止）
*******************************************************************************/
void MPU6050_Calibrate(void)
{
    short gx, gy, gz, ax, ay, az;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    short i;

    for (i = 0; i < CALIBRATION_SAMPLES; i++) {
        MPU_Get_Gyroscope(&gx, &gy, &gz);
        MPU_Get_Accelerometer(&ax, &ay, &az);
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        ax_sum += ax; ay_sum += ay; az_sum += az;
        Delay_ms(2);
    }

    gyro_x_offset = (float)gx_sum / CALIBRATION_SAMPLES;
    gyro_y_offset = (float)gy_sum / CALIBRATION_SAMPLES;
    gyro_z_offset = (float)gz_sum / CALIBRATION_SAMPLES;
}

/**************************实现函数********************************************
*函 数 名：MPU6050_Angle_Calc
*功    能：卡尔曼滤波计算姿态角
*参    数：pitch_out, roll_out, yaw_out: 输出角度指针
*说    明：需要在定时器中断中以固定周期调用（如20ms）
*******************************************************************************/
void MPU6050_Angle_Calc(float *pitch_out, float *roll_out, float *yaw_out)
{
    short ax, ay, az;
    short gx, gy, gz;
    float ax_f, ay_f, az_f;
    float gx_f, gy_f, gz_f;
    float accel_pitch, accel_roll;
    float dt;
    uint32_t now;

    // 读取原始数据
    MPU_Get_Gyroscope(&gx, &gy, &gz);
    MPU_Get_Accelerometer(&ax, &ay, &az);

    // 转换为物理单位
    // 加速度: ±2g -> 16384 LSB/g
    ax_f = (float)ax / 16384.0f;
    ay_f = (float)ay / 16384.0f;
    az_f = (float)az / 16384.0f;

    // 陀螺仪: ±2000dps -> 16.4 LSB/(°/s)，减去零偏
    gx_f = ((float)gx - gyro_x_offset) / 16.4f;
    gy_f = ((float)gy - gyro_y_offset) / 16.4f;
    gz_f = ((float)gz - gyro_z_offset) / 16.4f;

    // 加速度计计算角度（静态时准确）
    accel_roll = atan2f(ay_f, az_f) * 57.29578f;
    accel_pitch = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * 57.29578f;

    // 计算时间间隔
    now = SysTick->VAL;
    if (last_time == 0) last_time = now;
    dt = (float)(last_time - now) / 72000000.0f; // 72MHz
    if (dt <= 0 || dt > 0.5f) dt = 0.02f;        // 默认20ms
    last_time = now;

    // 卡尔曼滤波融合
    Roll = Kalman_Update(&kalman_roll, accel_roll, gx_f, dt);
    Pitch = Kalman_Update(&kalman_pitch, accel_pitch, gy_f, dt);

    // Yaw角：只能靠陀螺仪积分（无磁力计）
    Yaw = Kalman_Update(&kalman_yaw, Yaw, gz_f, dt);

    // 限制Yaw在±180度
    if (Yaw > 180.0f) Yaw -= 360.0f;
    if (Yaw < -180.0f) Yaw += 360.0f;

    // 输出结果
    *pitch_out = Pitch;
    *roll_out = Roll;
    *yaw_out = Yaw;
}

/**************************实现函数********************************************
*函 数 名：MPU_Set_Gyro_Fsr
*功    能：设置陀螺仪量程
*******************************************************************************/
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);
}

/**************************实现函数********************************************
*函 数 名：MPU_Set_Accel_Fsr
*功    能：设置加速度计量程
*******************************************************************************/
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);
}

/**************************实现函数********************************************
*函 数 名：MPU_Set_LPF
*功    能：设置数字低通滤波器
*******************************************************************************/
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data = 0;
    if (lpf >= 188) data = 1;
    else if (lpf >= 98)  data = 2;
    else if (lpf >= 42)  data = 3;
    else if (lpf >= 20)  data = 4;
    else if (lpf >= 10)  data = 5;
    else                 data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data);
}

/**************************实现函数********************************************
*函 数 名：MPU_Set_Rate
*功    能：设置采样率
*******************************************************************************/
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;
    if (rate > 1000) rate = 1000;
    if (rate < 4) rate = 4;
    data = 1000 / rate - 1;
    MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);
    return MPU_Set_LPF(rate / 2);
}

/**************************实现函数********************************************
*函 数 名：MPU_Get_Temperature
*功    能：读取温度
*******************************************************************************/
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((u16)buf[0] << 8) | buf[1];
    temp = 36.53f + ((float)raw) / 340.0f;
    return temp * 100;
}

/**************************实现函数********************************************
*函 数 名：MPU_Get_Gyroscope
*功    能：读取陀螺仪原始数据
*******************************************************************************/
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0) {
        *gx = ((u16)buf[0] << 8) | buf[1];
        *gy = ((u16)buf[2] << 8) | buf[3];
        *gz = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
}

/**************************实现函数********************************************
*函 数 名：MPU_Get_Accelerometer
*功    能：读取加速度计原始数据
*******************************************************************************/
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0) {
        *ax = ((u16)buf[0] << 8) | buf[1];
        *ay = ((u16)buf[2] << 8) | buf[3];
        *az = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
}

/**************************I2C底层函数****************************************/
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0);
    if (MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    for (i = 0; i < len; i++) {
        MPU_IIC_Send_Byte(buf[i]);
        if (MPU_IIC_Wait_Ack()) {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_Stop();
    return 0;
}

u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0);
    if (MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 1);
    MPU_IIC_Wait_Ack();
    while (len) {
        if (len == 1) *buf = MPU_IIC_Read_Byte(0);
        else         *buf = MPU_IIC_Read_Byte(1);
        len--;
        buf++;
    }
    MPU_IIC_Stop();
    return 0;
}

u8 MPU_Write_Byte(u8 reg, u8 data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0);
    if (MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(data);
    if (MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}

u8 MPU_Read_Byte(u8 reg)
{
    u8 res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 1);
    MPU_IIC_Wait_Ack();
    res = MPU_IIC_Read_Byte(0);
    MPU_IIC_Stop();
    return res;
}
