/**
 * 卡尔曼滤波算法完整实现
 * 提取自平衡小车 MiniBalanceV5.0 源码
 *
 * 数学原理：
 * 卡尔曼滤波是一种递推估计算法，用于从含有噪声的测量中
 * 估计动态系统的状态。它是最优估计（最小均方误差）。
 *
 * 系统模型：
 * - 状态方程：x(k) = A * x(k-1) + B * u(k) + w(k)
 * - 观测方程：z(k) = H * x(k) + v(k)
 *
 * 其中：
 * - x(k)：状态向量 [角度, 零偏]
 * - A：状态转移矩阵
 * - H：观测矩阵
 * - w(k)：过程噪声，协方差为Q
 * - v(k)：测量噪声，协方差为R
 *
 * 对于MPU6050姿态解算：
 * - 状态：角度和陀螺仪零偏
 * - 观测：加速度计计算的角度
 * - 输入：陀螺仪角速度
 */

#include "kalman_filter.h"
#include "app_config.h"
#include <math.h>
#include <stddef.h>

/*======================== 全局滤波器实例 ========================*/
// 全局实例（便捷使用）
static KalmanFilter_t global_kf_pitch;
static KalmanFilter_t global_kf_roll;
static KalmanFilter_t global_kf_yaw;

/*======================== 核心算法实现 ========================*/

/**
 * @brief  初始化卡尔曼滤波器
 * @param  kf: 滤波器结构体指针
 * @retval 无
 *
 * 初始化策略：
 * - 状态变量清零（角度和零偏的初始估计为0）
 * - 误差协方差矩阵设为单位矩阵（表示初始不确定性较大）
 * - 加载默认噪声参数
 */
void Kalman_Filter_Init(KalmanFilter_t *kf)
{
    if (kf == NULL) return;

    // 设置默认参数
    kf->Q_angle = KALMAN_DEFAULT_Q_ANGLE;
    kf->Q_gyro = KALMAN_DEFAULT_Q_GYRO;
    kf->R_angle = KALMAN_DEFAULT_R_ANGLE;
    kf->dt = KALMAN_DEFAULT_DT;

    // 初始化状态
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->rate = 0.0f;

    // 初始化误差协方差矩阵
    // 初始时，角度和零偏的不确定性都设为1
    kf->P[0][0] = 1.0f;  // 角度估计误差方差
    kf->P[0][1] = 0.0f;  // 角度和零偏的协方差
    kf->P[1][0] = 0.0f;  // 零偏和角度的协方差
    kf->P[1][1] = 1.0f;  // 零偏估计误差方差
}

/**
 * @brief  卡尔曼滤波更新（核心算法）
 * @param  kf: 滤波器结构体指针
 * @param  accel_angle: 加速度计测量角度（度）
 * @param  gyro_rate: 陀螺仪角速度（度/秒）
 * @retval 滤波后的角度（度）
 *
 * 算法步骤详解（结合代码注释）：
 *
 * 步骤1：预测（Predict）
 * - 使用陀螺仪数据预测角度变化
 * - 考虑零偏补偿
 * - 更新误差协方差矩阵
 *
 * 步骤2：更新（Update）
 * - 计算测量残差（innovation）
 * - 计算卡尔曼增益
 * - 使用测量值修正预测
 * - 更新误差协方差矩阵
 *
 * 数学公式：
 * 预测：
 * angle = angle + (gyro - bias) * dt
 * P = A * P * A^T + Q
 *
 * 更新：
 * K = P * H^T * (H * P * H^T + R)^-1
 * angle = angle + K * (z - H * angle)
 * P = (I - K * H) * P
 */
float Kalman_Filter_Update(KalmanFilter_t *kf,
                            float accel_angle,
                            float gyro_rate)
{
    if (kf == NULL) return 0.0f;

    float dt = kf->dt;
    float dt2 = dt * dt;

    //==================== 步骤1：预测 ====================//

    // 角度预测（使用陀螺仪数据）
    // angle = angle + (gyro - bias) * dt
    // - gyro：陀螺仪测量的角速度
    // - bias：估计的陀螺仪零偏
    kf->angle += dt * (gyro_rate - kf->bias);

    // 误差协方差矩阵预测
    // P = A * P * A^T + Q
    // 简化形式（针对我们的状态空间模型）：
    float Pdot[4];
    Pdot[0] = kf->Q_angle - kf->P[0][1] - kf->P[1][0];
    Pdot[1] = -kf->P[1][1];
    Pdot[2] = -kf->P[1][1];
    Pdot[3] = kf->Q_gyro;

    kf->P[0][0] += Pdot[0] * dt;
    kf->P[0][1] += Pdot[1] * dt;
    kf->P[1][0] += Pdot[2] * dt;
    kf->P[1][1] += Pdot[3] * dt;

    //==================== 步骤2：更新 ====================//

    // 计算测量残差（innovation）
    // y = z - H * x_predict
    // 其中z是加速度计测量的角度，H=[1, 0]
    float angle_err = accel_angle - kf->angle;

    // 计算卡尔曼增益
    // K = P * H^T * (H * P * H^T + R)^-1
    float C_0 = 1.0f;  // H矩阵的第一个元素
    float PCt_0 = C_0 * kf->P[0][0];
    float PCt_1 = C_0 * kf->P[1][0];

    // 创新协方差：S = H * P * H^T + R
    float S = kf->R_angle + C_0 * PCt_0;

    // 卡尔曼增益：K = P * H^T * S^-1
    float K_0 = PCt_0 / S;  // 角度的卡尔曼增益
    float K_1 = PCt_1 / S;  // 零偏的卡尔曼增益

    // 更新状态估计
    // x = x + K * y
    kf->angle += K_0 * angle_err;  // 更新角度估计
    kf->bias += K_1 * angle_err;   // 更新零偏估计

    // 计算去偏后的角速度
    kf->rate = gyro_rate - kf->bias;

    // 更新误差协方差矩阵
    // P = (I - K * H) * P
    float t_0 = PCt_0;
    float t_1 = C_0 * kf->P[0][1];

    kf->P[0][0] -= K_0 * t_0;
    kf->P[0][1] -= K_0 * t_1;
    kf->P[1][0] -= K_1 * t_0;
    kf->P[1][1] -= K_1 * t_1;

    return kf->angle;
}

/**
 * @brief  获取当前角度估计
 * @param  kf: 滤波器结构体指针
 * @retval 角度值（度）
 */
float Kalman_Filter_GetAngle(KalmanFilter_t *kf)
{
    if (kf == NULL) return 0.0f;
    return kf->angle;
}

/**
 * @brief  获取当前角速度估计（已去除零偏）
 * @param  kf: 滤波器结构体指针
 * @retval 角速度值（度/秒）
 *
 * 说明：这个值是陀螺仪原始数据减去估计的零偏
 * 比原始陀螺仪数据更准确
 */
float Kalman_Filter_GetRate(KalmanFilter_t *kf)
{
    if (kf == NULL) return 0.0f;
    return kf->rate;
}

/**
 * @brief  获取当前陀螺仪零偏估计
 * @param  kf: 滤波器结构体指针
 * @retval 零偏值（度/秒）
 *
 * 说明：
 * - 零偏是陀螺仪在静止时的输出
 * - 会随时间和温度变化
 * - 卡尔曼滤波器可以实时估计和补偿零偏
 */
float Kalman_Filter_GetBias(KalmanFilter_t *kf)
{
    if (kf == NULL) return 0.0f;
    return kf->bias;
}

/**
 * @brief  设置卡尔曼滤波参数
 * @param  kf: 滤波器结构体指针
 * @param  Q_angle: 过程噪声（角度）
 * @param  Q_gyro: 过程噪声（角速度）
 * @param  R_angle: 测量噪声
 * @retval 无
 *
 * 详细调参指南：
 *
 * 【Q_angle（角度过程噪声）】
 * 物理含义：陀螺仪测量的角度变化有多不可靠
 *
 * - 值小（如0.0001）：
 *   - 更信任陀螺仪
 *   - 输出平滑，噪声小
 *   - 但响应慢，可能丢失快速运动
 *   - 适合：慢速运动、高精度应用
 *
 * - 值大（如0.01）：
 *   - 更信任加速度计
 *   - 响应快，能跟踪快速运动
 *   - 但噪声大，可能不平滑
 *   - 适合：快速运动、动态响应要求高
 *
 * 【Q_gyro（角速度过程噪声）】
 * 物理含义：陀螺仪零偏变化有多快
 *
 * - 值小：零偏估计稳定，但收敛慢
 * - 值大：零偏估计快，但可能不稳定
 *
 * 【R_angle（测量噪声）】
 * 物理含义：加速度计测量有多不可靠
 *
 * - 值小（如0.1）：
 *   - 更信任加速度计
 *   - 响应快
 *   - 但对振动敏感
 *
 * - 值大（如1.0）：
 *   - 不信任加速度计
 *   - 更平滑
 *   - 但响应慢
 */
void Kalman_Filter_SetParams(KalmanFilter_t *kf,
                              float Q_angle,
                              float Q_gyro,
                              float R_angle)
{
    if (kf == NULL) return;

    kf->Q_angle = Q_angle;
    kf->Q_gyro = Q_gyro;
    kf->R_angle = R_angle;
}

/**
 * @brief  设置采样周期
 * @param  kf: 滤波器结构体指针
 * @param  dt: 采样周期（秒）
 * @retval 无
 *
 * 重要：dt必须与实际采样率精确匹配！
 * - 如果dt偏小：滤波器过度信任陀螺仪，可能发散
 * - 如果dt偏大：滤波器过度信任加速度计，噪声大
 *
 * 常见采样率对应的dt：
 * - 200Hz -> dt=0.005s
 * - 100Hz -> dt=0.01s
 * - 50Hz  -> dt=0.02s
 */
void Kalman_Filter_SetDt(KalmanFilter_t *kf, float dt)
{
    if (kf == NULL) return;
    if (dt <= 0.0f) return;
    kf->dt = dt;
}

/**
 * @brief  重置滤波器状态
 * @param  kf: 滤波器结构体指针
 * @retval 无
 *
 * 使用场景：
 * 1. 系统重启
 * 2. 传感器重新校准后
 * 3. 检测到滤波器发散
 * 4. 切换工作模式
 */
void Kalman_Filter_Reset(KalmanFilter_t *kf)
{
    if (kf == NULL) return;

    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->rate = 0.0f;

    // 重置协方差矩阵
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
}

/**
 * @brief  获取当前参数
 * @param  kf: 滤波器结构体指针
 * @param  Q_angle: 过程噪声（角度）输出
 * @param  Q_gyro: 过程噪声（角速度）输出
 * @param  R_angle: 测量噪声输出
 * @retval 无
 */
void Kalman_Filter_GetParams(KalmanFilter_t *kf,
                              float *Q_angle,
                              float *Q_gyro,
                              float *R_angle)
{
    if (kf == NULL) return;

    if (Q_angle != NULL) *Q_angle = kf->Q_angle;
    if (Q_gyro != NULL) *Q_gyro = kf->Q_gyro;
    if (R_angle != NULL) *R_angle = kf->R_angle;
}

/*======================== 便捷函数实现 ========================*/

/**
 * @brief  初始化全局卡尔曼滤波器
 * @retval 无
 *
 * 初始化三个轴的滤波器：
 * - global_kf_pitch：俯仰角
 * - global_kf_roll：横滚角
 * - global_kf_yaw：偏航角
 *
 * 使用示例：
 * // 初始化一次
 * Kalman_Filter_GlobalInit();
 *
 * // 主循环中使用
 * while(1) {
 *     // 计算加速度计角度
 *     float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;
 *     float accel_roll = atan2f(ay, az) * 57.29578f;
 *
 *     // 获取陀螺仪角速度
 *     float gyro_pitch = gy / 16.4f;
 *     float gyro_roll = gx / 16.4f;
 *     float gyro_yaw = gz / 16.4f;
 *
 *     // 更新滤波器
 *     float pitch = Kalman_Filter_GlobalUpdate(accel_pitch, -gyro_pitch);
 *     float roll = Kalman_Filter_GlobalUpdateRoll(accel_roll, gyro_roll);
 *     // Yaw没有加速度计参考，传入0
 *     float yaw = Kalman_Filter_GlobalUpdateYaw(0.0f, gyro_yaw);
 * }
 */
void Kalman_Filter_GlobalInit(void)
{
    // 启用DWT周期计数器（72MHz），用于实测每循环耗时并动态设置卡尔曼dt
    DEMCR |= TRCENA_BIT;
    DWT_CYCCNT = 0;
    DWT_CR |= CYCCNTENA_BIT;

    Kalman_Filter_Init(&global_kf_pitch);
    Kalman_Filter_Init(&global_kf_roll);
    Kalman_Filter_Init(&global_kf_yaw);
}

/**
 * @brief  重置全局滤波器状态（角度/零偏/协方差归零，参数与dt保持不变）
 * @retval 无
 *
 * 使用场景：
 * - 传感器重新校零后调用，清除校准前中断融合的历史状态
 * - 不重置 DWT 周期计数器，不影响 Sensor_Update 的 dt 实测
 */
void Kalman_Filter_GlobalReset(void)
{
    Kalman_Filter_Reset(&global_kf_pitch);
    Kalman_Filter_Reset(&global_kf_roll);
    Kalman_Filter_Reset(&global_kf_yaw);
}

/**
 * @brief  更新全局Pitch滤波器
 * @param  accel_angle: 加速度计Pitch角度（度）
 * @param  gyro_rate: Y轴陀螺仪角速度（度/秒）
 * @retval Pitch角估计值（度）
 */
float Kalman_Filter_GlobalUpdate(float accel_angle, float gyro_rate)
{
    return Kalman_Filter_Update(&global_kf_pitch, accel_angle, gyro_rate);
}

/**
 * @brief  更新全局Roll滤波器
 * @param  accel_angle: 加速度计Roll角度（度）
 * @param  gyro_rate: X轴陀螺仪角速度（度/秒）
 * @retval Roll角估计值（度）
 */
float Kalman_Filter_GlobalUpdateRoll(float accel_angle, float gyro_rate)
{
    return Kalman_Filter_Update(&global_kf_roll, accel_angle, gyro_rate);
}

/**
 * @brief  更新全局Yaw滤波器
 * @param  accel_angle: 加速度计或磁力计Yaw角度（度），无磁力计时传0
 * @param  gyro_rate: Z轴陀螺仪角速度（度/秒）
 * @retval Yaw角估计值（度）
 *
 * 注意：如果没有磁力计，accel_angle应传入0
 * 此时仅靠陀螺仪积分，会缓慢漂移
 */
float Kalman_Filter_GlobalUpdateYaw(float accel_angle, float gyro_rate)
{
    return Kalman_Filter_Update(&global_kf_yaw, accel_angle, gyro_rate);
}

/**
 * @brief  使用磁力计更新全局Yaw滤波器（互补滤波方式修正）
 * @param  mag_yaw: 磁力计计算的Yaw角度（度），传入-999表示无有效参考
 * @param  gyro_rate: Z轴陀螺仪角速度（度/秒）
 * @param  weight: 磁力计信任权重 (0.0~0.1)，典型值0.03
 * @retval Yaw角估计值（度）
 *
 * 算法说明：
 * - 使用一阶互补滤波: yaw = yaw + gyro*dt + weight*(mag_yaw - yaw)
 * - 等同于卡尔曼滤波中测量为mag_yaw的情况，但更轻量
 * - weight=0时退化为纯陀螺仪积分
 */
float Kalman_Filter_GlobalUpdateYawMag(float mag_yaw, float gyro_rate, float weight)
{
    /* 直接使用 global_kf_yaw.angle 作为状态：
     * 原先的 static 局部变量 yaw 无法被 Kalman_Filter_GlobalReset() 重置，
     * 导致校准完成后 Y 角仍从校准前的旧值继续，初始显示非 0。 */
    float yaw = global_kf_yaw.angle;

    /* 陀螺仪积分 */
    yaw += gyro_rate * global_kf_yaw.dt;

    /* 磁力计修正（一阶互补滤波） */
    if (mag_yaw > -998.0f && weight > 0.0f)
    {
        /* 处理角度跳变（±180°） */
        float diff = mag_yaw - yaw;
        if (diff > 180.0f)  diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;

        yaw += weight * diff;
    }

    /* 限制在±180° */
    if (yaw > 180.0f)  yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;

    global_kf_yaw.angle = yaw;
    return yaw;
}

/**
 * @brief  获取全局Pitch估计
 * @retval Pitch值（度）
 */
float Kalman_Filter_GlobalGetAngle(void)
{
    return Kalman_Filter_GetAngle(&global_kf_pitch);
}

/**
 * @brief  获取全局Roll估计
 * @retval Roll值（度）
 */
float Kalman_Filter_GlobalGetRoll(void)
{
    return Kalman_Filter_GetAngle(&global_kf_roll);
}

/**
 * @brief  获取全局Yaw估计
 * @retval Yaw值（度）
 */
float Kalman_Filter_GlobalGetYaw(void)
{
    return Kalman_Filter_GetAngle(&global_kf_yaw);
}

/**
 * @brief  获取全局滤波器的角速度
 * @retval Pitch角速度（度/秒）
 */
float Kalman_Filter_GlobalGetRate(void)
{
    return Kalman_Filter_GetRate(&global_kf_pitch);
}

/**
 * @brief  获取全局滤波器的零偏
 * @retval Pitch零偏（度/秒）
 */
float Kalman_Filter_GlobalGetBias(void)
{
    return Kalman_Filter_GetBias(&global_kf_pitch);
}

/**
 * @brief  设置全局滤波器的参数
 * @param  Q_angle: 过程噪声（角度）
 * @param  Q_gyro: 过程噪声（角速度）
 * @param  R_angle: 测量噪声
 * @retval 无
 *
 * 注意：此函数会同时设置三个轴的参数
 * 如需分别设置，请直接操作对应的滤波器结构体
 */
void Kalman_Filter_GlobalSetParams(float Q_angle, float Q_gyro, float R_angle)
{
    Kalman_Filter_SetParams(&global_kf_pitch, Q_angle, Q_gyro, R_angle);
    Kalman_Filter_SetParams(&global_kf_roll, Q_angle, Q_gyro, R_angle);
    Kalman_Filter_SetParams(&global_kf_yaw, Q_angle, Q_gyro, R_angle);
}

/**
 * @brief  设置全局采样周期（三轴同步）
 * @param  dt: 采样周期（秒）
 * @retval 无
 */
void Kalman_Filter_GlobalSetDt(float dt)
{
    Kalman_Filter_SetDt(&global_kf_pitch, dt);
    Kalman_Filter_SetDt(&global_kf_roll, dt);
    Kalman_Filter_SetDt(&global_kf_yaw, dt);
}

//------------------End of File----------------------------
