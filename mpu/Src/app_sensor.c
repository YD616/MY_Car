#include "app_sensor.h"
#include "app_config.h"
#include "sys.h"
#include "mpu6050.h"
#include "kalman_filter.h"
#include "hmc5883l.h"
#include "Delay.h"
#include <math.h>

// 将角度归一化到 [0, 360) 范围
// 用 fmodf 实现：对超大角度/负角度健壮，且不存在 while 循环的极端情况
float Angle_Normalize360(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

// 将角度归一化到 [-180, 180) 范围
float Angle_Normalize180(float angle)
{
    angle = Angle_Normalize360(angle);
    if (angle >= 180.0f) angle -= 360.0f;
    return angle;
}

// 两角之间的最小夹角差，返回 [-180, 180)
// 对 0°↔360° 与 ±180° 边界均不误判（死区判断的核心）
static float Angle_Diff(float a, float b)
{
    float d = a - b;
    if (d > 180.0f)  d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// 显示死区判断与角度归一化
static float disp_p = 0, disp_r = 0, disp_y = 0;   // 归一化后的显示角度
static float last_p = 0, last_r = 0, last_y = 0;   // 用于死区判断的连续角度
static u8 first_frame = 1;                         // 首帧强制刷新

// 滤波状态（文件级：Sensor_Calibrate 校准完成后整体重置，
// 初始状态 = 当前重力绝对姿态，Pitch/Roll 跟随重力方向而非 0）
static float lp_pitch = 0, lp_roll = 0;            // 加速度计角度一阶低通
static float yaw_mag_lp = 0.0f;                    // 磁力计航向一阶低通
static u8    yaw_mag_lp_init = 0;                  // 磁力计低通首次初始化标志

// 死区判断 + 角度归一化：需要刷新返回1
u8 Display_NeedUpdate(float pitch, float roll, float yaw)
{
    // 用最小夹角差判断死区：Yaw 被限制在 ±180°，跨边界时
    // (如 179.9°→-179.9° 实际只动 0.2°) 旧算法按 359.8° 会误刷新
    float dp = fabsf(Angle_Diff(pitch, last_p));
    float dr = fabsf(Angle_Diff(roll,  last_r));
    float dy = fabsf(Angle_Diff(yaw,   last_y));

    if (first_frame || dp >= DEADBAND || dr >= DEADBAND || dy >= DEADBAND)
    {
        first_frame = 0;
        last_p = pitch;
        last_r = roll;
        last_y = yaw;

        // 归一化到 ±180° 显示：比 0~360° 更直观（-8.7 而非 351.3）
        disp_p = Angle_Normalize180(pitch);
        disp_r = Angle_Normalize180(roll);
        disp_y = Angle_Normalize180(yaw);
        return 1;
    }
    return 0;
}

float Display_GetP(void) { return disp_p; }
float Display_GetR(void) { return disp_r; }
float Display_GetY(void) { return disp_y; }

// 三个姿态角（main 中三个角度获取函数的输出）
float pitch = 0, roll = 0, yaw = 0;
// 直立参考绝对俯仰角（°）：Sensor_Calibrate 校准时把车当前放置姿态当作
// "直立平衡目标"，这里存其输出坐标系下的绝对 pitch（含 PITCH_FLIP），
// 供直立环做目标角；显示/遥测仍用绝对 pitch 本身，不受此量影响。
float pitch_upright = 0;
// 加速度计解算角度（度）
float accel_pitch, accel_roll;
// 陀螺仪角速度（度/秒，已去除上电零偏）
float gx_f, gy_f, gz_f;
// 磁力计航向（度），-999 表示无有效参考
float yaw_mag = -999.0f;
// 磁力计数据有效性（供显示 M:OK/M:--）
u8 mag_ok = 0;

// 上电航向参考角（Sensor_Calibrate 计算）：Yaw 以校准时磁力计朝向为 0；
// Pitch/Roll 为绝对角、直接跟随重力方向，不需要"上电零位"
static float init_yaw = 0;
// 陀螺仪零偏（LSB，Sensor_Calibrate 计算）
static float bias_gx = 0, bias_gy = 0, bias_gz = 0;

// 磁力计连续无效计数（迟滞防抖，避免 M:OK/M:-- 频繁跳变）
static u8 mag_ready = 0;
static u8 mag_fail_cnt = 0;

// 重力向量 → 绝对俯仰角（度）。物理模型：姿态角与重力方向保持一致，
// 不再依赖"上电零位"。
// 输入：重力在三轴的分量（单位 g 或任意同尺度 LSB 均可）
// 处理流程：
//   1) 归一化：g = g / |g|，得到单位重力向量 (gx, gy, gz)，
//      消除各轴量程差异 / 增益误差 / 模长漂移带来的数值误差；
//   2) 由单位重力分量用 atan2 推导欧拉角：
//       pitch = atan2(-gx, sqrt(gy²+gz²)) * 180/π
// 约定：模块平放(z 轴竖直向上)时 gx=0、gy=0、gz=1 → pitch=0；
//       绕 Y 轴倾斜时 gx 增大 → pitch 实时指向重力方向。
static float GravityToPitchAbs(float ax, float ay, float az)
{
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-6f) return 0.0f;          // 零向量保护，避免除零/数值误差
    ax /= n; ay /= n; az /= n;           // 归一化 → ax²+ay²+az²=1

    // 公式：pitch = atan2(-gx, sqrt(gy²+gz²))
    return atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
}

// 重力向量 → 绝对翻滚角（度）。约定：绕 X 轴倾斜时 ay/az 变化，
// roll = atan2(gy, gz) * 180/π，方向与重力实时一致。
static float GravityToRollAbs(float ax, float ay, float az)
{
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-6f) return 0.0f;
    ax /= n; ay /= n; az /= n;           // 归一化重力向量，消除数值误差

    // 公式：roll = atan2(gy, gz)
    return atan2f(ay, az) * RAD2DEG;
}

// 上电校准：采集陀螺仪静止零偏（Yaw漂移的根源，必须校准），
// 并把 Pitch/Roll 的初始值设为"当前重力绝对姿态"（不再强制为 0）
void Sensor_Calibrate(void)
{
    short a0x, a0y, a0z, g0x, g0y, g0z, m0x, m0y, m0z;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float mx_sum = 0, my_sum = 0;
    float gyro_move;
    int i, j;

    // 预热：让MPU6050上电零偏稳定（期间勿触碰模块）
    Delay_ms(500);

    // 循环校准，直到检测到模块静止为止（最多 10 轮，防止模块持续晃动导致卡死）
    for (i = 0; i < 10; i++)
    {
        ax_sum = 0; ay_sum = 0; az_sum = 0;
        gx_sum = 0; gy_sum = 0; gz_sum = 0;
        mx_sum = 0; my_sum = 0;
        gyro_move = 0.0f;

        for (j = 0; j < 300; j++)
        {
            MPU_Get_Accelerometer(&a0x, &a0y, &a0z);
            MPU_Get_Gyroscope(&g0x, &g0y, &g0z);
            HMC5883L_Read(&m0x, &m0y, &m0z);
            ax_sum += a0x; ay_sum += a0y; az_sum += a0z;
            gx_sum += g0x; gy_sum += g0y; gz_sum += g0z;
            mx_sum += m0x; my_sum += m0y;
            gyro_move += (float)((g0x < 0 ? -g0x : g0x) +
                                 (g0y < 0 ? -g0y : g0y) +
                                 (g0z < 0 ? -g0z : g0z));
            Delay_ms(2);
        }

        // 静止时三轴角速度绝对值之和应很小（<100 LSB ≈ 2°/s）
        if (gyro_move / 300.0f < 100.0f)
            break;
    }

    /* Pitch/Roll 初始值 = 当前重力绝对姿态（校准均值，LSB 同尺度，
     * 函数内部会归一化）。Yaw 以校准时磁力计朝向为 0（保留 init_yaw）。 */
    float abs_pitch = GravityToPitchAbs(ax_sum / 300.0f, ay_sum / 300.0f, az_sum / 300.0f);
    float abs_roll  = GravityToRollAbs(ax_sum / 300.0f, ay_sum / 300.0f, az_sum / 300.0f);
    init_yaw        = atan2f(my_sum, mx_sum) * RAD2DEG;

    // 陀螺仪零偏（静止时角速度应≈0，实测平均值即零偏，单位LSB）
    bias_gx = gx_sum / 300.0f;
    bias_gy = gy_sum / 300.0f;
    bias_gz = gz_sum / 300.0f;

    /* ========== 校准完成：重置滤波状态到"当前重力姿态" ==========
     * 原因：PA12 中断在 Sensor_Calibrate() 之前就已触发 Sensor_Angle_Update()，
     * 期间卡尔曼/低通已用校准前的旧数据融合。校准完成后需整体重置，
     * 并以刚解算出的绝对姿态(abs_pitch/abs_roll)作为初始状态重新起步：
     * Pitch/Roll 直接跟随重力方向，初始值即放置姿态（不再强制为 0）。
     * Yaw 无重力参考，仍以校准时磁力计朝向为 0。 */
    Kalman_Filter_GlobalReset();                 /* angle=0, bias=0, P=I (参数/dt保留) */
    Kalman_Filter_GlobalSetParams(Q_ANGLE, Q_GYRO, R_ANGLE);   /* 保险：恢复参数 */
    accel_pitch = abs_pitch; accel_roll = abs_roll;   /* accel 解算角初始=当前重力姿态 */
    lp_pitch = abs_pitch; lp_roll = abs_roll;         /* 低通初始=当前重力姿态 */
    /* 卡尔曼内部从 0 起步，P=I 使首帧大增益，立即向绝对姿态收敛 */
    pitch = Kalman_Filter_GlobalUpdate(abs_pitch * PITCH_FLIP, 0.0f);
    roll  = Kalman_Filter_GlobalUpdateRoll(abs_roll, 0.0f);
    yaw = 0;                                     /* Yaw 以校准时磁力计朝向为 0 */
    yaw_mag_lp = 0.0f; yaw_mag_lp_init = 0;      /* 磁力计低通复位（下次直接取当前值） */
    first_frame = 1;                             /* 首帧强制刷新显示当前姿态 */
    last_p = last_r = last_y = 0;
}

// 数据采集与预处理：实测dt + 读取加速度计/陀螺仪 + 单位换算 + 计算加速度计角度
// 供 main 中 Pitch/Roll 两个角度获取函数使用
void Sensor_Update(void)
{
    static short ax, ay, az;
    static short gx, gy, gz;
    static uint32_t last_tick = 0;   // 首次调用时记录基准
    static u16 still_cnt = 0;        // 静止连续计数（40次=200ms）
    static u16 bias_cnt = 0;         // 零偏统计计数（200次=1s）
    static float bias_sum = 0;       // 静止期 gz 原始 LSB 累计
    float ax_f, ay_f, az_f;

    // 关键：用DWT实测每循环实际耗时并设置dt。
    if (last_tick == 0) last_tick = DWT_CYCCNT;
    uint32_t now = DWT_CYCCNT;
    float dt = (float)(now - last_tick) / 72000000.0f;  // 72MHz主频
    last_tick = now;
    if (dt < 0.001f) dt = 0.001f;  // 下限保护（极少数情况）
    if (dt > 0.1f)   dt = 0.1f;    // 上限保护（防止卡顿导致积分突变）
    Kalman_Filter_GlobalSetDt(dt);

    // 读取原始数据
    MPU_Get_Accelerometer(&ax, &ay, &az);
    MPU_Get_Gyroscope(&gx, &gy, &gz);

    // 转换为物理单位
    // 加速度: ±2g -> ACCEL_LSB_PER_G LSB/g
    ax_f = (float)ax / ACCEL_LSB_PER_G;
    ay_f = (float)ay / ACCEL_LSB_PER_G;
    az_f = (float)az / ACCEL_LSB_PER_G;

    // 陀螺仪: ±2000dps -> GYRO_LSB_PER_DPS LSB/(°/s)，减去上电校准的零偏
    gx_f = ((float)gx - bias_gx) / GYRO_LSB_PER_DPS;
    gy_f = ((float)gy - bias_gy) / GYRO_LSB_PER_DPS;
    gz_f = ((float)gz - bias_gz) / GYRO_LSB_PER_DPS;

    /* ========== 静止检测 + Z轴零偏在线修正（抑制 Yaw 漂移） ========== */
    /* 原理：模块静止时真实角速度为 0，gz 实测值即当前零偏残留。
     * 静止时动态更新 bias_gz，消除温漂/残留零偏导致的 Yaw 匀速漂移。
     * 磁力计失效(纯积分)时此修正直接决定 Yaw 漂移速度。 */
    if (fabsf(gx_f) < 1.0f && fabsf(gy_f) < 1.0f && fabsf(gz_f) < 0.8f)
    {
        if (still_cnt < 40) still_cnt++;   /* 连续200ms静止才进入统计 */
        if (still_cnt >= 40)
        {
            bias_sum += (float)gz;         /* 累计原始 LSB */
            if (++bias_cnt >= 200)         /* 统计1秒(200×5ms) */
            {
                float new_bias = bias_sum / 200.0f;
                /* 与当前零偏差 > 30 LSB(≈2°/s) 视为异常，不采纳 */
                if (fabsf(new_bias - bias_gz) < 30.0f)
                    bias_gz = bias_gz * 0.7f + new_bias * 0.3f;  /* 平滑更新防突变 */
                bias_cnt = 0;
                bias_sum = 0.0f;
            }
        }
    }
    else
    {
        still_cnt = 0;   /* 模块在动，重新计时 */
        bias_cnt = 0;
        bias_sum = 0.0f;
    }

    // 计算加速度计绝对角度：归一化重力向量后用 atan2 解算（见
    // GravityToPitchAbs / GravityToRollAbs）。角度=重力方向的绝对欧拉角，
    // 逐帧跟随重力方向变化，不再减上电零位、不再把初始姿态强制为 0。
    // 一阶低通滤波：电机机械振动会使加速度计解算角度高频抖动，
    // 低通后卡尔曼融合更干净（截止约5Hz，alpha=0.35 可现场微调）
    {
        float new_pitch = GravityToPitchAbs(ax_f, ay_f, az_f);
        float new_roll  = GravityToRollAbs(ax_f, ay_f, az_f);
        lp_pitch += 0.35f * (new_pitch - lp_pitch);
        lp_roll  += 0.35f * (new_roll  - lp_roll);
        accel_pitch = lp_pitch;
        accel_roll  = lp_roll;
    }
}

// 磁力计航向计算与有效性判定
// 注意：须在 main 中 Pitch/Roll 角度获取函数调用之后再调用（倾斜补偿需要最新 pitch/roll）
void Sensor_UpdateMag(void)
{
    static short mx, my, mz;

#if USE_MAG
    // 磁场总强度应在地磁范围（0.25~0.65G，±1.9Ga量程820 LSB/G → 200~550 LSB）
    // 上限放宽到2000，避免环境磁场稍强时误判；读数接近0说明传感器未工作
    if (HMC5883L_Read(&mx, &my, &mz) == 0)  // I2C读取成功才参与判定
    {
        float mag_norm = sqrtf((float)mx * mx + (float)my * my + (float)mz * mz);
        if (mag_norm > 50.0f && mag_norm < 2000.0f)
        {
            mag_fail_cnt = 0;
            // 倾斜补偿：把磁力计三轴投影到水平面，模块倾斜时航向也准确
            float cp = cosf(pitch * DEG2RAD);
            float sp = sinf(pitch * DEG2RAD);
            float cr = cosf(roll * DEG2RAD);
            float sr = sinf(roll * DEG2RAD);
            float Xh = (float)mx * cp + (float)my * sp * sr + (float)mz * sp * cr;
            float Yh = (float)my * cr - (float)mz * sr;

            // 计算磁力计方位角（减去上电零位参考，保证初始为 0）
            yaw_mag = atan2f(Yh, Xh) * RAD2DEG - init_yaw;
            // 方向翻转（HMC5883L 安装方向不同时调整）
            yaw_mag *= YAW_FLIP;
            // 补偿磁偏角（根据所在地区调整，中国约-5°~+5°）
            yaw_mag += 0.0f;
            if (yaw_mag > 180.0f)  yaw_mag -= 360.0f;
            if (yaw_mag < -180.0f) yaw_mag += 360.0f;
            // 低通滤波：电机电磁干扰会造成磁力计读数尖峰（角度差方式，正确处理±180°跳变）
            {
                float diff;
                if (!yaw_mag_lp_init) { yaw_mag_lp = yaw_mag; yaw_mag_lp_init = 1; }
                diff = yaw_mag - yaw_mag_lp;
                if (diff > 180.0f)  diff -= 360.0f;
                if (diff < -180.0f) diff += 360.0f;
                yaw_mag_lp += 0.3f * diff;   /* 平滑系数0.3，可现场微调 */
                if (yaw_mag_lp > 180.0f)  yaw_mag_lp -= 360.0f;
                if (yaw_mag_lp < -180.0f) yaw_mag_lp += 360.0f;
                yaw_mag = yaw_mag_lp;
            }
            mag_ready = 0;
            mag_ok = 1;
        }
        else
        {
            mag_fail_cnt++;
            if (mag_fail_cnt >= 10)  // 连续10次(≈50ms)无效才判定失效，防临界抖动
            {
                mag_ok = 0;
                mag_ready++;
                if (mag_ready > 50) yaw_mag = -999.0f;  // 磁力计读数异常，无效
            }
        }
    }
    else
    {
        // I2C读取失败：保留上一次有效数据与状态，仅累计失败计数
        mag_fail_cnt++;
        if (mag_fail_cnt >= 10)
        {
            mag_ok = 0;
            mag_ready++;
            if (mag_ready > 50) yaw_mag = -999.0f;
        }
    }
#else
    yaw_mag = -999.0f;
    mag_ok = 0;
#endif
}

// 角度整体更新：由 PA12 外部中断回调触发调用
// 内部顺序不可颠倒：Sensor_Update(采集) → Pitch/Roll融合 →
// Sensor_UpdateMag(磁力计, 倾斜补偿依赖最新Pitch/Roll) → Yaw融合
void Sensor_Angle_Update(void)
{
    Sensor_Update();

    /* 三个角度获取函数（Pitch/Roll/Yaw） */
    pitch = Kalman_Filter_GlobalUpdate(accel_pitch * PITCH_FLIP, -gy_f * PITCH_FLIP);
    roll  = Kalman_Filter_GlobalUpdateRoll(accel_roll, gx_f);

    /* 磁力计航向计算（须在 Pitch/Roll 之后，倾斜补偿需要最新角度） */
    Sensor_UpdateMag();

    yaw = Kalman_Filter_GlobalUpdateYawMag(yaw_mag, gz_f * YAW_FLIP, MAG_WEIGHT);
}
