/* 编码器测速: 差分法 + 真实时间戳, 任意调用频率均返回正确的平均线速度 (cm/s) */
#include "encode.h"

/* ---- 内部数据结构 ---- */
typedef struct {
    uint16_t last_cnt;      /* 上次原始计数值(16位) */
    uint32_t last_ms;       /* 上次采样时刻 (ms) */
    float    last_speed;    /* 上次算出的速度(cm/s), 供同一毫秒内重复调用直接返回 */
    uint8_t  first;         /* 1=首次采样, 仅记录基准不计算 */
} Encoder_Data;

static Encoder_Data enc_l;  /* 左轮 -> ENC_TIM_L */
static Encoder_Data enc_r;  /* 右轮 -> ENC_TIM_R */

/* 轮 -> 定时器映射 (实机接线): 左轮 TIM3 (PA6/PA7), 右轮 TIM4 (PB6/PB7)
 * 初始化基准与 Encoder_GetSpeedL/R 全部走这两个别名, 保证映射一致 */
#define ENC_TIM_L   (&htim3)    /* 左轮编码器: PA6/PA7 -> TIM3 */
#define ENC_TIM_R   (&htim4)    /* 右轮编码器: PB6/PB7 -> TIM4 */

/* 采样一个编码器并计算速度 (cm/s):
 * uint16_t 减法 + int16_t 强转可正确处理 16 位计数器回绕;
 * 首次调用只记录基准返回 0; 同一毫秒内重复调用返回缓存值(不丢脉冲) */
static float Encoder_CalcSpeed(TIM_HandleTypeDef *htim, Encoder_Data *enc)
{
    uint32_t now = HAL_GetTick();
    uint16_t cur = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    uint32_t dt_ms;
    int16_t  delta;
    float    speed;

    if (enc->first)          /* 首次采样: 只记录基准 */
    {
        enc->first      = 0;
        enc->last_cnt   = cur;
        enc->last_ms    = now;
        enc->last_speed = 0.0f;
        return 0.0f;
    }

    dt_ms = now - enc->last_ms;

    /* 同一毫秒内重复调用(多调用点共用同一个 now): dt=0 无法算速度, 返回上次结果。
     * 此时不更新 last_cnt/last_ms, 否则会把这段脉冲吞掉, 表现为"该轮速度恒为 0"。 */
    if (dt_ms == 0U)
        return enc->last_speed;

    delta = (int16_t)(cur - enc->last_cnt);
    enc->last_cnt = cur;
    enc->last_ms  = now;

    /* speed = delta / 每转计数(1320) x 周长(cm) x 校准 / dt(s) */
    speed = (float)delta / (float)ENCODER_RESOLUTION
          * WHEEL_PERIMETER_CM * ENCODER_CALIBRATION
          * 1000.0f / (float)dt_ms;

    /* A/B 相接反时速度符号会与实际前进方向相反, 由 ENCODER_DIR_FLIP 统一翻转;
     * 必须与电机转向一致, 否则速度环会正反馈发散。 */
#if ENCODER_DIR_FLIP
    speed = -speed;
#endif

    enc->last_speed = speed;  /* 缓存本次结果, 供同一毫秒内的下一次调用返回 */
    return speed;
}

/* 启动 TIM3/TIM4 编码器接口 (置位 CEN, 计数器才开始随 A/B 相脉冲计数) */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* 记录初始计数值作为差分基准 (必须与 GetSpeedL/R 用的是同一个定时器) */
    enc_l.last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_L);
    enc_r.last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_R);
    enc_l.last_ms  = 0;
    enc_r.last_ms  = 0;
    enc_l.first    = 1;
    enc_r.first    = 1;
    enc_l.last_speed = 0.0f;
    enc_r.last_speed = 0.0f;
}

/* 返回"名字上那个轮子"的真实速度: 转左轮 → Encoder_GetSpeedL() 变化,
 * 这样 pid_speed_L 反馈 / OLED "VL" 行 / Motor_Direction('L') 三者才同侧 */
float Encoder_GetSpeedL(void)
{
    return Encoder_CalcSpeed(ENC_TIM_L, &enc_l);
}

float Encoder_GetSpeedR(void)
{
    return Encoder_CalcSpeed(ENC_TIM_R, &enc_r);
}
