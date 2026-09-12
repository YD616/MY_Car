/**
 * 卡尔曼滤波算法完整实现
 * 提取自平衡小车 MiniBalanceV5.0 源码
 *
 * 算法特点：
 * - 最优递归估计器
 * - 自适应融合加速度计和陀螺仪数据
 * - 自动估计和补偿陀螺仪零偏
 * - 对噪声有良好的鲁棒性
 *
 * 卡尔曼滤波原理：
 * 1. 预测（Predict）：基于上一时刻状态和系统模型预测当前状态
 * 2. 更新（Update）：使用当前时刻的测量值修正预测
 *
 * 优势：
 * - 理论上是最优估计（最小均方误差）
 * - 可以处理过程噪声和测量噪声
 * - 能够估计隐含状态（如陀螺仪零偏）
 */

#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*======================== 参数配置 ========================*/
// 默认参数（可调整）
#define KALMAN_DEFAULT_Q_ANGLE  0.001f   // 过程噪声协方差（角度）
#define KALMAN_DEFAULT_Q_GYRO   0.003f   // 过程噪声协方差（角速度）
#define KALMAN_DEFAULT_R_ANGLE  0.5f     // 测量噪声协方差
#define KALMAN_DEFAULT_DT       0.005f   // 采样周期（秒），5ms=200Hz

/*======================== 数据结构 ========================*/
/**
 * 卡尔曼滤波器结构体
 * 包含滤波器的所有状态和参数
 */
typedef struct {
    // 可调参数
    float Q_angle;      // 过程噪声协方差（角度）
    float Q_gyro;       // 过程噪声协方差（角速度）
    float R_angle;      // 测量噪声协方差

    // 状态变量
    float angle;        // 角度估计值（度）
    float bias;         // 陀螺仪零偏估计值（度/秒）
    float rate;         // 角速度估计值（去偏后）（度/秒）

    // 误差协方差矩阵（2x2）
    // P[0][0] = 角度估计误差方差
    // P[0][1] = P[1][0] = 角度和零偏的协方差
    // P[1][1] = 零偏估计误差方差
    float P[2][2];

    // 时间参数
    float dt;           // 采样周期（秒）
} KalmanFilter_t;

/*======================== 函数接口 ========================*/

/**
 * @brief  初始化卡尔曼滤波器
 * @param  kf: 滤波器结构体指针
 * @retval 无
 *
 * 初始化内容：
 * 1. 设置默认参数（Q_angle=0.001, Q_gyro=0.003, R_angle=0.5）
 * 2. 清零状态变量（angle=0, bias=0）
 * 3. 初始化协方差矩阵为单位矩阵
 *
 * 使用示例：
 * KalmanFilter_t kf;
 * Kalman_Filter_Init(&kf);
 */
void Kalman_Filter_Init(KalmanFilter_t *kf);

/**
 * @brief  卡尔曼滤波更新（核心算法）
 * @param  kf: 滤波器结构体指针
 * @param  accel_angle: 加速度计测量角度（度）
 * @param  gyro_rate: 陀螺仪角速度（度/秒）
 * @retval 滤波后的角度（度）
 *
 * 算法步骤详解：
 *
 * 【预测步骤】
 * 1. 角度预测：angle = angle + (gyro - bias) * dt
 *    - 使用陀螺仪数据预测角度变化
 *    - 减去估计的零偏
 *
 * 2. 协方差预测：P = A * P * A^T + Q
 *    - 更新误差协方差矩阵
 *    - 加入过程噪声Q
 *
 * 【更新步骤】
 * 3. 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^-1
 *    - 决定如何分配预测和测量的权重
 *    - 如果测量噪声R大，则更信任预测
 *    - 如果预测误差P大，则更信任测量
 *
 * 4. 更新状态估计：angle = angle + K * (measurement - predicted)
 *    - 使用测量残差修正预测
 *
 * 5. 更新协方差：P = (I - K * H) * P
 *    - 反映估计精度的提高
 *
 * 使用示例：
 * KalmanFilter_t kf;
 * Kalman_Filter_Init(&kf);
 *
 * while(1) {
 *     float accel_angle = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;
 *     float gyro_rate = gy / 16.4f;
 *     float angle = Kalman_Filter_Update(&kf, accel_angle, -gyro_rate);
 * }
 */
float Kalman_Filter_Update(KalmanFilter_t *kf,
                            float accel_angle,
                            float gyro_rate);

/**
 * @brief  获取当前角度估计
 * @param  kf: 滤波器结构体指针
 * @retval 角度值（度）
 */
float Kalman_Filter_GetAngle(KalmanFilter_t *kf);

/**
 * @brief  获取当前角速度估计（已去除零偏）
 * @param  kf: 滤波器结构体指针
 * @retval 角速度值（度/秒）
 */
float Kalman_Filter_GetRate(KalmanFilter_t *kf);

/**
 * @brief  获取当前陀螺仪零偏估计
 * @param  kf: 滤波器结构体指针
 * @retval 零偏值（度/秒）
 */
float Kalman_Filter_GetBias(KalmanFilter_t *kf);

/**
 * @brief  设置卡尔曼滤波参数
 * @param  kf: 滤波器结构体指针
 * @param  Q_angle: 过程噪声（角度）
 * @param  Q_gyro: 过程噪声（角速度）
 * @param  R_angle: 测量噪声
 * @retval 无
 *
 * 调参指南：
 *
 * 【Q_angle（角度过程噪声）】
 * - 增大：更信任加速度计，响应快但噪声大
 * - 减小：更信任陀螺仪，平滑但响应慢
 * - 推荐范围：0.0001 - 0.01
 *
 * 【Q_gyro（角速度过程噪声）】
 * - 影响零偏估计的收敛速度
 * - 增大：零偏估计更快，但可能不稳定
 * - 减小：零偏估计更稳定，但收敛慢
 * - 推荐范围：0.001 - 0.01
 *
 * 【R_angle（测量噪声）】
 * - 增大：更不信任测量，结果更平滑
 * - 减小：更信任测量，响应快但噪声大
 * - 推荐范围：0.1 - 1.0
 *
 * 典型应用场景：
 * - 快速运动：减小R_angle，增大Q_angle
 * - 慢速平滑：增大R_angle，减小Q_angle
 * - 噪声环境：增大R_angle
 */
void Kalman_Filter_SetParams(KalmanFilter_t *kf,
                              float Q_angle,
                              float Q_gyro,
                              float R_angle);

/**
 * @brief  设置采样周期
 * @param  kf: 滤波器结构体指针
 * @param  dt: 采样周期（秒）
 * @retval 无
 *
 * 注意：dt必须与实际采样率匹配，否则滤波效果会变差
 */
void Kalman_Filter_SetDt(KalmanFilter_t *kf, float dt);

/**
 * @brief  重置滤波器状态
 * @param  kf: 滤波器结构体指针
 * @retval 无
 *
 * 使用场景：
 * - 系统重启
 * - 传感器重新校准
 * - 检测到异常数据时
 */
void Kalman_Filter_Reset(KalmanFilter_t *kf);

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
                              float *R_angle);

/*======================== 便捷函数 ========================*/
// 以下函数提供全局滤波器实例，简化使用

/**
 * @brief  初始化全局卡尔曼滤波器
 * @retval 无
 *
 * 使用示例：
 * Kalman_Filter_GlobalInit();
 * while(1) {
 *     float angle = Kalman_Filter_GlobalUpdate(accel_angle, gyro_rate);
 * }
 */
void Kalman_Filter_GlobalInit(void);

/**
 * @brief  重置全局滤波器状态（角度/零偏/协方差归零，参数与dt保持不变）
 * @retval 无
 *
 * 使用场景：
 * - 传感器重新校零后调用，清除校准前中断融合的历史状态
 * - 不重置 DWT 周期计数器，不影响 Sensor_Update 的 dt 实测
 */
void Kalman_Filter_GlobalReset(void);

/**
 * @brief  更新全局滤波器
 * @param  accel_angle: 加速度计角度（度）
 * @param  gyro_rate: 陀螺仪角速度（度/秒）
 * @retval 滤波后的角度（度）
 */
float Kalman_Filter_GlobalUpdate(float accel_angle, float gyro_rate);

/**
 * @brief  更新全局Roll滤波器
 * @param  accel_angle: 加速度计Roll角度（度）
 * @param  gyro_rate: X轴陀螺仪角速度（度/秒）
 * @retval Roll角估计值（度）
 */
float Kalman_Filter_GlobalUpdateRoll(float accel_angle, float gyro_rate);

/**
 * @brief  更新全局Yaw滤波器
 * @param  accel_angle: 加速度计或磁力计Yaw角度（度），无磁力计时传0
 * @param  gyro_rate: Z轴陀螺仪角速度（度/秒）
 * @retval Yaw角估计值（度）
 *
 * 注意：如果没有磁力计，accel_angle应传入0
 * 此时仅靠陀螺仪积分，会缓慢漂移
 */
float Kalman_Filter_GlobalUpdateYaw(float accel_angle, float gyro_rate);

/**
 * @brief  使用外部参考角度更新全局Yaw滤波器（磁力计）
 * @param  mag_yaw: 磁力计计算的Yaw角度（度），传入-999表示无有效参考
 * @param  gyro_rate: Z轴陀螺仪角速度（度/秒）
 * @param  weight: 磁力计信任权重 (0.0~1.0)，典型值0.02~0.1
 * @retval Yaw角估计值（度）
 *
 * 说明：
 * - 当mag_yaw=-999时退化为纯陀螺仪积分
 * - weight控制磁力计修正力度：越大修正越快但噪声大
 * - 推荐weight=0.05，即每次修正5%的误差
 */
float Kalman_Filter_GlobalUpdateYawMag(float mag_yaw, float gyro_rate, float weight);

/**
 * @brief  获取全局Pitch估计
 * @retval Pitch值（度）
 */
float Kalman_Filter_GlobalGetAngle(void);

/**
 * @brief  获取全局Roll估计
 * @retval Roll值（度）
 */
float Kalman_Filter_GlobalGetRoll(void);

/**
 * @brief  获取全局Yaw估计
 * @retval Yaw值（度）
 */
float Kalman_Filter_GlobalGetYaw(void);

/**
 * @brief  获取全局滤波器的角速度
 * @retval Pitch角速度（度/秒）
 */
float Kalman_Filter_GlobalGetRate(void);

/**
 * @brief  获取全局滤波器的零偏
 * @retval Pitch零偏（度/秒）
 */
float Kalman_Filter_GlobalGetBias(void);

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
void Kalman_Filter_GlobalSetParams(float Q_angle, float Q_gyro, float R_angle);

/**
 * @brief  设置全局采样周期
 * @param  dt: 采样周期（秒）
 * @retval 无
 *
 * 重要：dt必须与实际循环周期精确匹配！
 * 若dt偏小（实际循环比dt慢），陀螺仪积分角度会被低估（增益<1）
 * 建议用定时器/DWT实测每循环耗时后调用本函数
 */
void Kalman_Filter_GlobalSetDt(float dt);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_FILTER_H */

//------------------End of File----------------------------
