#ifndef __WAVELET_ENERGY_H
#define __WAVELET_ENERGY_H

#include "stm32f10x.h"

/* ===================== 小波能量积分配置 ===================== */

/* 小波窗口长度（必须是2的幂） */
#define WE_WINDOW_SIZE          64

/* DWT分解层数（64→32→16→8） */
#define WE_DWT_LEVELS          3

/* 开机自学习周期（秒） */
#define WE_LEARNING_PERIOD_SEC 30

/* 检测阈值参数：threshold = baseline + k × std */
/* k=3 对应 99.7% 置信区间（正态分布） */
#define WE_STD_FACTOR          3

/* 最小有效能量增量（高于此值才考虑事件） */
#define WE_MIN_ENERGY_DELTA    50

/* ===================== 标定模式配置 ===================== */
/* 标定开关：0=关闭，1=开启（标定时打开，量产时关闭） */
#define WE_CALIBRATION_MODE    1

/* 事件结束后的死区时间，使用原始采样数折算，避免降采样时死区时间漂移。 */
#define WE_INTEGRAL_DEADTIME_US 2500U

/* 多窗口事件包络参数：连续窗口内累计峰值，短暂跌落允许继续归为同一事件。 */
/* Event integration timing for calibration impulse feature. */
#define WE_EVENT_MAX_DURATION_US 12000U
#define WE_EVENT_GAP_US          1000U
#define WE_PRE_TRIGGER_SAMPLES   96U
#define WE_EVENT_BUFFER_SIZE     256U
#define WE_TRIGGER_DELTA_ADC     20U
#define WE_SIGNAL_MIN_ADC        4U

/* 高频比例阈值（permille，0-1000）：
 * hf_ratio ≤ WE_HF_RAIN_MAX → 雨滴（低频为主）
 * hf_ratio ≥ WE_HF_VIB_MIN → 振动/EMI（高频为主）
 * 中间区域保守处理为振动 */
#define WE_HF_RAIN_MAX         400
#define WE_HF_VIB_MIN          450

/* 默认阈值倍数因子（兼容旧接口，检测期实际用 baseline + k×std） */
#define WE_DEFAULT_FACTOR      3.0f

/* ===================== 事件类型定义 ===================== */
typedef enum {
    WE_EVENT_NONE = 0,    /* 无有效事件 */
    WE_EVENT_LEARNING,     /* 自学习中 */
    WE_EVENT_RAIN,        /* 雨滴事件 */
    WE_EVENT_VIB,         /* 振动/风扰 */
    WE_EVENT_NOISE,       /* 噪声/EMI */
    WE_EVENT_SUSPECT      /* 可疑（能量超阈值但高频比高） */
} WE_EventType_t;

/* ===================== 上下文结构体 ===================== */
typedef struct {
    /* 直流基线 */
    uint16_t dc_baseline;        /* 传感器直流偏置（ADC原始值），学习期估算，检测期固定 */

    /* 能量参数 */
    uint32_t energy_total;        /* 当前帧总能量（高频，d1+d2+d3） */
    uint32_t energy_high;         /* 高频能量（别名，等于 energy_total） */
    uint32_t energy_d1;           /* 第1层detail能量 */
    uint32_t energy_d2;           /* 第2层detail能量 */
    uint32_t energy_d3;           /* 第3层detail能量 */
    uint32_t energy_a3;           /* 第3层approx能量 */
    uint16_t hf_ratio;           /* 高频比例（permille） */
    uint32_t energy_delta;        /* 能量增量 = energy_high - baseline_energy */

    /* 自学习基线（基于高频能量统计） */
    uint32_t baseline_energy;     /* 环境基线能量（均值） */
    uint32_t baseline_std;        /* 基线标准差（用于统计阈值） */
    uint32_t baseline_count;      /* 检测期帧计数器（用于慢速跟踪） */

    /* 采样率（Hz），由 AD_Init 时实测注入，0 表示未标定（使用默认 23810） */
    uint32_t actual_sample_rate_hz;

    /* 自学习期累加器 */
    uint64_t energy_sum;          /* 学习期内高频能量累加（64位防溢出） */
    uint64_t energy_sq_sum;       /* 学习期内高频能量平方和（用于计算真实标准差） */
    uint32_t energy_sample_count; /* 学习期样本计数 */
    uint32_t learning_raw_sample_count; /* 学习期覆盖的原始采样数估计 */

    /* 检测阈值 */
    float    energy_factor;        /* 阈值倍数因子（默认3.0，兼容旧接口） */
    uint32_t threshold_energy;    /* 检测阈值 = baseline + k × std */

    /* 事件状态 */
    WE_EventType_t event_type;     /* 当前事件类型 */
    uint8_t  learning_done;       /* 学习完成标志 */
    uint16_t learning_duration_sec; /* 配置的学习周期（秒） */
    uint16_t learning_elapsed_sec;  /* 学习已进行秒数（估算值） */
    uint16_t process_decimation;    /* 当前批处理降采样因子 */

    /* 采样处理统计 */
    uint16_t last_available_samples; /* 最近一次批处理看到的新样本数 */
    uint16_t max_available_samples;  /* 单次批处理最大积压样本数 */
    uint32_t sample_overrun_count;   /* 环形缓冲被写满后仍有新样本的次数 */
    uint32_t dropped_sample_count;   /* 因环形缓冲不够而丢弃的样本估计数 */
    uint32_t processed_window_count; /* 已处理的小波窗口数 */

    /* 事件包络与死区 */
    uint32_t deadtime_raw_remaining; /* 死区剩余原始采样数 */
    uint8_t  event_active;          /* 正在累计事件包络 */
    uint8_t  event_gap_windows;     /* 事件内短暂跌落窗口数 */
    uint8_t  event_window_count;    /* 当前事件包络窗口数 */
    WE_EventType_t event_acc_type;  /* 当前事件包络类型 */
    float    event_peak_impulse;    /* 当前事件包络最大冲量窗口 */
    uint32_t event_peak_energy;     /* 当前事件包络最大高频能量 */
    uint32_t event_peak_delta;      /* 当前事件包络最大能量增量 */
    uint32_t event_peak_norm_ratio; /* 当前事件包络最大归一化比值 */
    uint16_t event_hf_ratio;        /* 当前事件包络代表性高频比例 */

    /* 冲量 */
    float    impulse_current;      /* 当前窗口冲量 */
    float    impulse_rain;        /* 最近一次雨滴事件冲量 */
    uint32_t norm_ratio;          /* 最近一次雨滴事件归一化比值（×1000permille） */
    volatile uint8_t rain_event_pending;
    volatile uint8_t vib_event_pending;
    volatile uint8_t noise_event_pending;

    /* 统计计数 */
    volatile uint32_t rain_count;
    volatile uint32_t vib_count;
    volatile uint32_t noise_count;
    float    event_impulse_sum;
    uint32_t event_gap_raw_samples;
    uint32_t event_raw_samples;
    uint32_t last_event_raw_samples;
    uint16_t event_sample_count;
    uint16_t event_effective_start;
    uint16_t event_effective_end;
    int16_t  event_buffer[WE_EVENT_BUFFER_SIZE];
    volatile uint32_t total_events;
} WaveletEnergyCtx_t;

/* ===================== 函数接口 ===================== */
void WaveletEnergy_Init(WaveletEnergyCtx_t *ctx, uint16_t learning_sec);
void WaveletEnergy_InitWithSampleRate(WaveletEnergyCtx_t *ctx, uint16_t learning_sec, uint32_t actual_sample_rate_hz);
/* ===================== 批量采样处理（替代逐样本ISR调用） ===================== */
/**
  * @brief  主循环调用：处理环形缓冲区中积累的新样本
  * @param  ctx:  小波能量上下文
  * @retval 处理的样本数（0 表示无新数据）
  * @note   替代原来在 DMA ISR 中逐样本调用 WaveletEnergy_Update 的方式，
  *         主循环每 10ms 调用一次，内部批量处理积累的样本。
  *         若样本过多（>64），自动降采样，每 4 个取 1 个处理。
  */
uint32_t WaveletEnergy_ProcessNewSamples(WaveletEnergyCtx_t *ctx);

/* ===================== 旧接口（保留，批量模式下主循环不再直接调用）===================== */
void WaveletEnergy_Update(WaveletEnergyCtx_t *ctx,
                           volatile uint16_t *ring_buf,
                           uint16_t ring_idx);
WE_EventType_t WaveletEnergy_GetEventType(WaveletEnergyCtx_t *ctx);
uint32_t WaveletEnergy_ComputeVolume(WaveletEnergyCtx_t *ctx);
uint8_t  WaveletEnergy_GetLearningProgress(WaveletEnergyCtx_t *ctx);
uint32_t WaveletEnergy_GetNormRatio(WaveletEnergyCtx_t *ctx);
uint32_t WaveletEnergy_GetCurrentEnergy(WaveletEnergyCtx_t *ctx);
uint32_t WaveletEnergy_GetBaselineEnergy(WaveletEnergyCtx_t *ctx);
uint32_t WaveletEnergy_GetThreshold(WaveletEnergyCtx_t *ctx);
uint16_t WaveletEnergy_GetHFRatio(WaveletEnergyCtx_t *ctx);
float    WaveletEnergy_GetRainImpulse(WaveletEnergyCtx_t *ctx);
void     WaveletEnergy_ClearRainPending(WaveletEnergyCtx_t *ctx);
uint8_t  WaveletEnergy_PopVibEvent(WaveletEnergyCtx_t *ctx);
uint8_t  WaveletEnergy_PopNoiseEvent(WaveletEnergyCtx_t *ctx);

#endif
