#include "WaveletEnergy.h"
#include "AD.h"
#include <string.h>

/* 降采样因子：新样本超过此数时每N个取1个（防止主循环被阻塞） */
#define WE_DOWNSAMPLE_THRESHOLD  64
#define WE_DOWNSAMPLE_FACTOR     4

/* ===================== 内部函数原型 ===================== */
static void Compute_DC_Baseline(volatile uint16_t *ring_buf, uint16_t ring_idx, int32_t *out_dc);
static void Fill_Window_Centered(volatile uint16_t *ring_buf, uint16_t ring_idx,
                                  int16_t *window, int32_t dc_baseline);
static void Haar_DWT_Step(int16_t *input, uint16_t even_len, int16_t *approx, int16_t *detail);
static uint32_t Compute_Energy_SumSquares(int16_t *coeffs, uint16_t n);
static void Extract_Energy(WaveletEnergyCtx_t *ctx, int16_t *window);
static uint32_t Compute_RawSamples_FromUs(WaveletEnergyCtx_t *ctx, uint32_t duration_us);
static uint32_t Compute_Deadtime_RawSamples(WaveletEnergyCtx_t *ctx);
static int16_t Clip_To_Int16(int32_t value);
static uint16_t Abs_I16(int16_t value);
static uint8_t Event_IsActiveSample(int16_t sample);
static void EventEnvelope_Reset(WaveletEnergyCtx_t *ctx);
static void EventSnapshot_Start(WaveletEnergyCtx_t *ctx, volatile uint16_t *ring_buf, uint16_t ring_idx, int32_t dc_baseline);
static void EventSnapshot_Append(WaveletEnergyCtx_t *ctx, int16_t sample, uint16_t raw_step);
static void EventSnapshot_Analyze(WaveletEnergyCtx_t *ctx);
static void EventEnvelope_Finalize(WaveletEnergyCtx_t *ctx);

static uint32_t Saturate_U64_To_U32(uint64_t value)
{
    if (value > 0xFFFFFFFFULL)
        return 0xFFFFFFFFU;
    return (uint32_t)value;
}

static uint32_t Compute_RawSamples_FromUs(WaveletEnergyCtx_t *ctx, uint32_t duration_us)
{
    uint32_t sample_rate = ctx->actual_sample_rate_hz;
    if (sample_rate == 0)
        sample_rate = 23810U;

    uint64_t samples = ((uint64_t)sample_rate * duration_us + 999999ULL) / 1000000ULL;
    if (samples < 1)
        samples = 1;
    if (samples > 0xFFFFFFFFULL)
        samples = 0xFFFFFFFFULL;
    return (uint32_t)samples;
}

static uint32_t Compute_Deadtime_RawSamples(WaveletEnergyCtx_t *ctx)
{
    return Compute_RawSamples_FromUs(ctx, WE_INTEGRAL_DEADTIME_US);
}

static int16_t Clip_To_Int16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static uint16_t Abs_I16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-value) : (uint16_t)value;
}

static uint8_t Event_IsActiveSample(int16_t sample)
{
    return (Abs_I16(sample) >= WE_SIGNAL_MIN_ADC) ? 1 : 0;
}

static void EventEnvelope_Reset(WaveletEnergyCtx_t *ctx)
{
    ctx->event_active = 0;
    ctx->event_gap_windows = 0;
    ctx->event_window_count = 0;
    ctx->event_gap_raw_samples = 0;
    ctx->event_raw_samples = 0;
    ctx->event_impulse_sum = 0.0f;
    ctx->event_sample_count = 0;
    ctx->event_effective_start = 0;
    ctx->event_effective_end = 0;
    ctx->event_acc_type = WE_EVENT_NONE;
    ctx->event_peak_impulse = 0.0f;
    ctx->event_peak_energy = 0;
    ctx->event_peak_delta = 0;
    ctx->event_peak_norm_ratio = 0;
    ctx->event_hf_ratio = 0;
}

static void EventSnapshot_Start(WaveletEnergyCtx_t *ctx, volatile uint16_t *ring_buf, uint16_t ring_idx, int32_t dc_baseline)
{
    uint16_t pre = (WE_PRE_TRIGGER_SAMPLES > WE_EVENT_BUFFER_SIZE) ? WE_EVENT_BUFFER_SIZE : WE_PRE_TRIGGER_SAMPLES;

    EventEnvelope_Reset(ctx);
    ctx->event_active = 1;
    ctx->event_acc_type = WE_EVENT_RAIN;
    ctx->event_hf_ratio = ctx->hf_ratio;

    for (uint16_t i = 0; i < pre; i++)
    {
        uint16_t idx = (uint16_t)((ring_idx + RING_BUFFER_SIZE - pre + i) % RING_BUFFER_SIZE);
        ctx->event_buffer[i] = Clip_To_Int16((int32_t)ring_buf[idx] - dc_baseline);
    }

    ctx->event_sample_count = pre;
    ctx->event_raw_samples = pre;
    ctx->event_gap_raw_samples = 0;
}

static void EventSnapshot_Append(WaveletEnergyCtx_t *ctx, int16_t sample, uint16_t raw_step)
{
    if (ctx->event_sample_count < WE_EVENT_BUFFER_SIZE)
    {
        ctx->event_buffer[ctx->event_sample_count++] = sample;
    }

    ctx->event_raw_samples += raw_step;

    if (Event_IsActiveSample(sample))
        ctx->event_gap_raw_samples = 0;
        ctx->event_gap_raw_samples += raw_step;

    if (ctx->event_gap_raw_samples >= Compute_RawSamples_FromUs(ctx, WE_EVENT_GAP_US) ||
        ctx->event_raw_samples >= Compute_RawSamples_FromUs(ctx, WE_EVENT_MAX_DURATION_US) ||
        ctx->event_sample_count >= WE_EVENT_BUFFER_SIZE)
    {
        EventEnvelope_Finalize(ctx);
    }
}

static void EventSnapshot_Analyze(WaveletEnergyCtx_t *ctx)
{
    uint16_t count = ctx->event_sample_count;
    uint16_t start = 0;
    uint16_t end = 0;
    uint16_t peak_idx = 0;
    int16_t peak = 0;
    float impulse = 0.0f;
    int16_t wave_window[WE_WINDOW_SIZE];

    if (count == 0)
        return;

    while (start < count && !Event_IsActiveSample(ctx->event_buffer[start]))
        start++;
    if (start >= count)
        start = 0;

    end = count - 1;
    while (end > start && !Event_IsActiveSample(ctx->event_buffer[end]))
        end--;

    for (uint16_t i = start; i <= end && i < count; i++)
    {
        int16_t sample = ctx->event_buffer[i];
        if (sample > peak)
        {
            peak = sample;
            peak_idx = i;
        }
        if (sample > (int16_t)WE_SIGNAL_MIN_ADC)
            impulse += (float)(sample - (int16_t)WE_SIGNAL_MIN_ADC);
    }

    {
        uint16_t base = 0;
        if (count > WE_WINDOW_SIZE)
        {
            if (peak_idx > (WE_WINDOW_SIZE / 2))
                base = (uint16_t)(peak_idx - (WE_WINDOW_SIZE / 2));
            if ((uint16_t)(base + WE_WINDOW_SIZE) > count)
                base = (uint16_t)(count - WE_WINDOW_SIZE);
        }

        for (uint16_t i = 0; i < WE_WINDOW_SIZE; i++)
        {
            uint16_t src = (uint16_t)(base + i);
            wave_window[i] = (src < count) ? ctx->event_buffer[src] : 0;
        }
    }

    Extract_Energy(ctx, wave_window);
    ctx->energy_delta = (ctx->energy_high >= ctx->baseline_energy) ? (ctx->energy_high - ctx->baseline_energy) : 0;
    ctx->event_peak_energy = ctx->energy_high;
    ctx->event_peak_delta = ctx->energy_delta;
    ctx->event_hf_ratio = ctx->hf_ratio;
    ctx->event_peak_impulse = (float)peak;
    ctx->event_impulse_sum = impulse;
    ctx->event_effective_start = start;
    ctx->event_effective_end = end;
    ctx->last_event_raw_samples = (uint32_t)(end - start + 1);

    if (ctx->baseline_energy > 0 && ctx->energy_high >= ctx->baseline_energy)
        ctx->event_peak_norm_ratio = (uint32_t)(((uint64_t)(ctx->energy_high - ctx->baseline_energy) * 1000ULL) / ctx->baseline_energy);
    else
        ctx->event_peak_norm_ratio = 0;
}

static void EventEnvelope_Finalize(WaveletEnergyCtx_t *ctx)
{
    if (!ctx->event_active)
        return;

    EventSnapshot_Analyze(ctx);
    ctx->impulse_rain = ctx->event_impulse_sum;
    ctx->energy_high = ctx->event_peak_energy;
    ctx->energy_delta = ctx->event_peak_delta;
    ctx->norm_ratio = ctx->event_peak_norm_ratio;
    ctx->hf_ratio = ctx->event_hf_ratio;
    ctx->event_type = WE_EVENT_RAIN;

    ctx->total_events++;
    ctx->rain_count++;
    ctx->rain_event_pending = 1;

    ctx->deadtime_raw_remaining = Compute_Deadtime_RawSamples(ctx);
    EventEnvelope_Reset(ctx);
}

/* ===================== 直流基线估算（学习期用）===================== */
/**
  * @brief  从环形缓冲区取最近 N 个样本的均值作为直流偏置
  * @param  ring_buf: ADC环形缓冲区
  * @param  ring_idx: 当前写入位置（样本已写入此位置）
  * @param  out_dc:   输出直流偏置（ADC原始值）
  * @retval 无
  * @note   学习期用此估算传感器直流偏置，不依赖小波能量
  */
static void Compute_DC_Baseline(volatile uint16_t *ring_buf, uint16_t ring_idx, int32_t *out_dc)
{
    /* 取最近64点的均值作为直流偏置 */
    uint32_t sum = 0;
    for (int16_t i = 63; i >= 0; i--)
    {
        uint16_t idx = (ring_idx + RING_BUFFER_SIZE - 1 - i) % RING_BUFFER_SIZE;
        sum += ring_buf[idx];
    }
    *out_dc = (int32_t)(sum / 64);
}

/* ===================== 信号窗口提取（去直流）===================== */
/**
  * @brief  从环形缓冲区取最近64点，去直流后存入window
  * @param  ring_buf:     ADC环形缓冲区
  * @param  ring_idx:     当前写入位置
  * @param  window:       输出（去直流后的信号）
  * @param  dc_baseline: 学习期得到的直流偏置（ADC原始值）
  * @retval 无
  */
static void Fill_Window_Centered(volatile uint16_t *ring_buf, uint16_t ring_idx,
                                  int16_t *window, int32_t dc_baseline)
{
    for (int16_t i = WE_WINDOW_SIZE - 1; i >= 0; i--)
    {
        uint16_t idx = (ring_idx + RING_BUFFER_SIZE - 1 - i) % RING_BUFFER_SIZE;
        window[i] = Clip_To_Int16((int32_t)ring_buf[idx] - dc_baseline);
    }
}

/* ===================== Haar DWT 单层分解 ===================== */
static void Haar_DWT_Step(int16_t *input, uint16_t even_len,
                          int16_t *approx, int16_t *detail)
{
    for (uint16_t i = 0; i < even_len; i++)
    {
        approx[i] = (int16_t)((input[i * 2] + input[i * 2 + 1]) >> 1);
        detail[i] = (int16_t)((input[i * 2] - input[i * 2 + 1]) >> 1);
    }
}

/* ===================== 能量计算（平方和）===================== */
static uint32_t Compute_Energy_SumSquares(int16_t *coeffs, uint16_t n)
{
    uint64_t acc = 0;
    for (uint16_t i = 0; i < n; i++)
    {
        int32_t v = (int32_t)coeffs[i];
        acc += (uint64_t)((int64_t)v * (int64_t)v);
    }
    return Saturate_U64_To_U32(acc);
}

/* ===================== 提取小波能量特征 ===================== */
/**
  * @brief  64点Haar DWT三层分解，提取高频能量
  * @param  ctx:    上下文
  * @param  window: 64点去直流信号
  * @retval 无
  * @note   能量只用 detail 系数（d1+d2+d3），不含近似系数。
  *         因为近似系数含直流，只在 detail 系数中反映信号起伏能量。
  */
static void Extract_Energy(WaveletEnergyCtx_t *ctx, int16_t *window)
{
    /* 第1层：64→32 */
    static int16_t approx1[32];
    static int16_t detail1[32];
    Haar_DWT_Step(window, 32, approx1, detail1);

    /* 第2层：32→16 */
    static int16_t approx2[16];
    static int16_t detail2[16];
    Haar_DWT_Step(approx1, 16, approx2, detail2);

    /* 第3层：16→8 */
    static int16_t approx3[8];
    static int16_t detail3[8];
    Haar_DWT_Step(approx2, 8, approx3, detail3);

    /* 高频能量 = d1 + d2 + d3（只用detail系数，不含近似） */
    uint32_t e_d1 = Compute_Energy_SumSquares(detail1, 32);
    uint32_t e_d2 = Compute_Energy_SumSquares(detail2, 16);
    uint32_t e_d3 = Compute_Energy_SumSquares(detail3, 8);
    uint32_t e_a3 = Compute_Energy_SumSquares(approx3, 8); /* 近似能量（不用于检测，仅统计） */

    ctx->energy_d1 = e_d1;
    ctx->energy_d2 = e_d2;
    ctx->energy_d3 = e_d3;
    ctx->energy_a3 = e_a3;
    ctx->energy_high = Saturate_U64_To_U32((uint64_t)e_d1 + e_d2 + e_d3);
    ctx->energy_total = ctx->energy_high; /* 检测只用高频能量 */

    /* 高频比例（permille） */
    if (ctx->energy_total > 0)
    {
        uint64_t all_energy = (uint64_t)ctx->energy_high + e_a3;
        uint64_t ratio = ((uint64_t)ctx->energy_high * 1000ULL) / all_energy;
        if (ratio > 1000) ratio = 1000;
        ctx->hf_ratio = (uint16_t)ratio;
    }
    else
    {
        ctx->hf_ratio = 0;
    }
}

/* ===================== 公共函数实现 ===================== */

void WaveletEnergy_Init(WaveletEnergyCtx_t *ctx, uint16_t learning_sec)
{
    memset(ctx, 0, sizeof(WaveletEnergyCtx_t));
    ctx->learning_duration_sec = (learning_sec == 0) ? WE_LEARNING_PERIOD_SEC : learning_sec;
    ctx->learning_elapsed_sec = 0;
    ctx->process_decimation = 1;
    ctx->energy_factor  = WE_DEFAULT_FACTOR;
    ctx->event_type     = WE_EVENT_LEARNING;
    ctx->impulse_current = 0.0f;
    ctx->impulse_rain   = 0.0f;
    ctx->norm_ratio     = 0;
    ctx->rain_event_pending   = 0;
    ctx->vib_event_pending   = 0;
    ctx->noise_event_pending = 0;
    /* actual_sample_rate_hz 默认为 0，在 InitWithSampleRate 中设置 */
}

/**
  * @brief  带实测采样率的初始化（推荐使用）
  * @param  ctx: 上下文
  * @param  learning_sec: 学习时长（秒）
  * @param  actual_sample_rate_hz: 实测 ADC 采样频率（Hz），由 AD_Init 传入
  * @note   采样率用于精确计算学习期时长和死区时间
  */
void WaveletEnergy_InitWithSampleRate(WaveletEnergyCtx_t *ctx, uint16_t learning_sec, uint32_t actual_sample_rate_hz)
{
    memset(ctx, 0, sizeof(WaveletEnergyCtx_t));
    ctx->learning_duration_sec = (learning_sec == 0) ? WE_LEARNING_PERIOD_SEC : learning_sec;
    ctx->learning_elapsed_sec = 0;
    ctx->process_decimation = 1;
    ctx->energy_factor  = WE_DEFAULT_FACTOR;
    ctx->event_type     = WE_EVENT_LEARNING;
    ctx->impulse_current = 0.0f;
    ctx->impulse_rain   = 0.0f;
    ctx->norm_ratio     = 0;
    ctx->rain_event_pending   = 0;
    ctx->vib_event_pending   = 0;
    ctx->noise_event_pending = 0;
    ctx->actual_sample_rate_hz = actual_sample_rate_hz;
}

void WaveletEnergy_Update(WaveletEnergyCtx_t *ctx,
                           volatile uint16_t *ring_buf,
                           uint16_t ring_idx)
{
    uint16_t raw_step = (ctx->process_decimation == 0) ? 1 : ctx->process_decimation;

    /* ===== 积分死区：按原始采样数计时，避免降采样时死区漂移 ===== */
    if (ctx->deadtime_raw_remaining > 0)
    {
        if (ctx->deadtime_raw_remaining > raw_step)
            ctx->deadtime_raw_remaining -= raw_step;
        else
            ctx->deadtime_raw_remaining = 0;
        ctx->event_type = WE_EVENT_NONE;
        return;
    }

    /* ===== 1. 直流基线估算 ===== */
    int32_t dc_baseline = 0;
    if (ctx->learning_done)
    {
        /* 检测期：使用学习期得到的直流偏置 */
        dc_baseline = (int32_t)ctx->dc_baseline;
    }
    else
    {
        /* 学习期：实时估算直流偏置 */
        Compute_DC_Baseline(ring_buf, ring_idx, &dc_baseline);
        ctx->dc_baseline = (uint16_t)dc_baseline;
    }

    /* ===== 2. 取64点去直流信号 ===== */
    int16_t window[WE_WINDOW_SIZE];
    Fill_Window_Centered(ring_buf, ring_idx, window, dc_baseline);

    /* ===== 3. 学习期 ===== */
    if (!ctx->learning_done)
    {
        /* 提取能量特征 */
        Extract_Energy(ctx, window);

        /* 累加能量（用于统计基线均值和方差） */
        ctx->energy_sum += ctx->energy_high;
        ctx->energy_sq_sum += (uint64_t)ctx->energy_high * (uint64_t)ctx->energy_high;
        ctx->energy_sample_count++;
        ctx->learning_raw_sample_count += (ctx->process_decimation == 0) ? 1 : ctx->process_decimation;

        /* 秒数估算（用于显示，进度条） */
        uint32_t sample_rate = ctx->actual_sample_rate_hz;
        if (sample_rate == 0) sample_rate = 23810U; /* 未标定时使用经验值 */
        uint32_t sec_estimate = ctx->learning_raw_sample_count / sample_rate;
        if (sec_estimate > ctx->learning_duration_sec)
            sec_estimate = ctx->learning_duration_sec;
        ctx->learning_elapsed_sec = (uint16_t)sec_estimate;

        /* 检查学习完成条件（样本数 = 采样率 × 学习秒数） */
        uint32_t required = sample_rate * (uint32_t)ctx->learning_duration_sec;
        if (ctx->learning_raw_sample_count >= required)
        {
            /* 计算基线能量均值 */
            ctx->baseline_energy = (uint32_t)(ctx->energy_sum / ctx->energy_sample_count);

            /* 用样本统计计算标准差：std = sqrt(E[X^2] - E[X]^2)
             * Var(X) = E[X^2] - E[X]^2，防止浮点误差导致负数 */
            uint64_t E_X_sq = ctx->energy_sq_sum / ctx->energy_sample_count;
            uint64_t mean_sq = (uint64_t)ctx->baseline_energy * (uint64_t)ctx->baseline_energy;
            uint64_t var = (E_X_sq > mean_sq) ? (E_X_sq - mean_sq) : 0;

            /* 近似开平方（牛顿法，2次迭代足够） */
            uint64_t std = ctx->baseline_std; /* 保留旧值用于初始化 */
            if (var > 0)
            {
                std = (var + 1) >> 1;
                std = (std + var / std) >> 1;
                std = (std + var / std) >> 1;
            }
            if (std < 1) std = 1;
            ctx->baseline_std = (uint32_t)std;

            /* 检测阈值 = baseline + k × std（统计判据，k=3 约99.7%置信区间）*/
            uint32_t thr = Saturate_U64_To_U32((uint64_t)ctx->baseline_energy + (uint64_t)WE_STD_FACTOR * ctx->baseline_std);
            uint32_t min_thr = Saturate_U64_To_U32((uint64_t)ctx->baseline_energy + WE_MIN_ENERGY_DELTA);
            if (thr < min_thr)
                thr = min_thr;
            ctx->threshold_energy = thr;

            if (ctx->baseline_energy < 1)
                ctx->baseline_energy = 1;

            ctx->learning_done = 1;
            ctx->event_type    = WE_EVENT_NONE;
        }
        else
        {
            ctx->event_type = WE_EVENT_LEARNING;
        }
        return;
    }

    /* ===== 4. 检测期 ===== */
    Extract_Energy(ctx, window);

    /* 能量增量 */
    if (ctx->energy_high >= ctx->baseline_energy)
        ctx->energy_delta = ctx->energy_high - ctx->baseline_energy;
    else
        ctx->energy_delta = 0;

    /* 冲量计算：I = Σ max(0, signal[i]) × Ts（去直流后信号，正值积分） */
    /* Calibration impulse contribution: integrate wavelet energy above baseline. */
    uint16_t latest_idx = (ring_idx == 0) ? (RING_BUFFER_SIZE - 1) : (ring_idx - 1);
    uint16_t latest_raw = ring_buf[latest_idx];
    int16_t latest_signal = window[WE_WINDOW_SIZE - 1];
    uint8_t raw_trigger = 0;

    ctx->impulse_current = (latest_signal > 0) ? (float)latest_signal : 0.0f;
    if (latest_signal >= (int16_t)WE_TRIGGER_DELTA_ADC)
        raw_trigger = 1;
    if (latest_raw >= ADC_Threshold && latest_signal >= (int16_t)WE_SIGNAL_MIN_ADC)
        raw_trigger = 1;

    /* 基线慢速跟踪（长时间无事件时轻微调整） */
    if (ctx->baseline_count > 1000 && ctx->energy_high < ctx->baseline_energy)
    {
        ctx->baseline_energy = (ctx->baseline_energy * 999U + ctx->energy_high) / 1000U;
        ctx->threshold_energy = Saturate_U64_To_U32((uint64_t)ctx->baseline_energy + (uint64_t)WE_STD_FACTOR * ctx->baseline_std);
        uint32_t min_thr = Saturate_U64_To_U32((uint64_t)ctx->baseline_energy + WE_MIN_ENERGY_DELTA);
        if (ctx->threshold_energy < min_thr)
            ctx->threshold_energy = min_thr;
    }
    ctx->baseline_count++;

    /* 事件分类 */
    ctx->event_type = WE_EVENT_NONE;
    if (!ctx->event_active)
    {
        if (raw_trigger)
        {
            EventSnapshot_Start(ctx, ring_buf, ring_idx, dc_baseline);
        }
        return;
    }
    EventSnapshot_Append(ctx, latest_signal, raw_step);
    return;
}

/* ===================== 辅助函数 ===================== */
WE_EventType_t WaveletEnergy_GetEventType(WaveletEnergyCtx_t *ctx)
{
    return ctx->event_type;
}

/**
  * @deprecated  旧线性体积模型，已废弃。
  *              请使用 raindrop.c 中的 Raindrop_VoltageToVolume_0p01mm3() LUT 查表法。
  *              当前返回的 ratio 仅供参考。
  * @note   标定后替换为真实映射函数。
  */
uint32_t WaveletEnergy_ComputeVolume(WaveletEnergyCtx_t *ctx)
{
    (void)ctx;
    /* 旧线性模型已废弃，体积计算统一走 raindrop.c 的 LUT 路径。
     * 此函数保留接口兼容，返回 0 让调用方走正确的 LUT 流程。 */
    return 0;
}

uint8_t WaveletEnergy_GetLearningProgress(WaveletEnergyCtx_t *ctx)
{
    if (ctx->learning_done) return 100;
    if (ctx->learning_duration_sec == 0) return 0;
    return (uint8_t)((ctx->learning_elapsed_sec * 100U) / ctx->learning_duration_sec);
}

uint32_t WaveletEnergy_GetNormRatio(WaveletEnergyCtx_t *ctx)
{
    if (ctx->baseline_energy == 0)
        return 0;
    if (ctx->energy_high >= ctx->baseline_energy)
        return (uint32_t)(((uint64_t)(ctx->energy_high - ctx->baseline_energy) * 1000ULL) / ctx->baseline_energy);
    return 0;
}

uint32_t WaveletEnergy_GetCurrentEnergy(WaveletEnergyCtx_t *ctx)
{
    return ctx->energy_high;
}

uint32_t WaveletEnergy_GetBaselineEnergy(WaveletEnergyCtx_t *ctx)
{
    return ctx->baseline_energy;
}

uint32_t WaveletEnergy_GetThreshold(WaveletEnergyCtx_t *ctx)
{
    return ctx->threshold_energy;
}

uint16_t WaveletEnergy_GetHFRatio(WaveletEnergyCtx_t *ctx)
{
    return ctx->hf_ratio;
}

float WaveletEnergy_GetRainImpulse(WaveletEnergyCtx_t *ctx)
{
    return ctx->impulse_rain;
}

void WaveletEnergy_ClearRainPending(WaveletEnergyCtx_t *ctx)
{
    ctx->rain_event_pending = 0;
}

uint8_t WaveletEnergy_PopVibEvent(WaveletEnergyCtx_t *ctx)
{
    if (ctx->vib_event_pending) { ctx->vib_event_pending = 0; return 1; }
    return 0;
}

uint8_t WaveletEnergy_PopNoiseEvent(WaveletEnergyCtx_t *ctx)
{
    if (ctx->noise_event_pending) { ctx->noise_event_pending = 0; return 1; }
    return 0;
}

/* ===================== 批量采样处理（替代逐样本ISR调用） ===================== */
uint32_t WaveletEnergy_ProcessNewSamples(WaveletEnergyCtx_t *ctx)
{
    static uint32_t last_write_total = 0;
    uint16_t current_ring_idx = ring_write_index_ch0;
    uint32_t current_write_total = ring_write_total_ch0;
    uint32_t processed = 0;

    if (current_write_total == last_write_total)
        return 0;  /* 无新数据 */

    /* 用累计写入计数计算真实新增样本数，避免写索引绕回后误判为少量新数据。 */
    uint32_t available_total = current_write_total - last_write_total;
    uint32_t available_report = available_total;
    if (available_report > 0xFFFFU)
        available_report = 0xFFFFU;
    ctx->last_available_samples = (uint16_t)available_report;
    if (ctx->last_available_samples > ctx->max_available_samples)
        ctx->max_available_samples = ctx->last_available_samples;
    if (available_total > RING_BUFFER_SIZE)
    {
        ctx->sample_overrun_count++;
        ctx->dropped_sample_count += (available_total - RING_BUFFER_SIZE);
        available_total = RING_BUFFER_SIZE;
    }

    uint16_t available = (uint16_t)available_total;

    if (available == 0)
    {
        last_write_total = current_write_total;
        return 0;
    }

    /* 降采样：样本过多时每隔 N 个取 1 个，防止主循环阻塞 */
    uint16_t step = 1;
#if WE_CALIBRATION_MODE
    step = 1;
#else
    if (available > WE_DOWNSAMPLE_THRESHOLD)
        step = WE_DOWNSAMPLE_FACTOR;
#endif
    ctx->process_decimation = step;

    /* 逐个处理（降采样后） */
    uint16_t consumed = 0;
    uint16_t start_idx = (uint16_t)((current_ring_idx + RING_BUFFER_SIZE - available) % RING_BUFFER_SIZE);
    while (consumed < available)
    {
        uint16_t idx = (start_idx + consumed) % RING_BUFFER_SIZE;
        /* 每次处理以当前 idx 为"最新位置"，传入 idx+1（= 下一次写入位置），
         * 与原来 ISR 中行为一致：ring_write_index 指向"下一个待写入位置" */
        uint16_t next_idx = (idx + 1) % RING_BUFFER_SIZE;

        /* 复用现有 WaveletEnergy_Update 的核心逻辑，
         * 注意：需要直接操作 ctx 而非通过环形缓冲区的最新点 */
        WaveletEnergy_Update(ctx, adc_ring_buffer_ch0, next_idx);
        processed++;
        consumed += step;
    }

    ctx->processed_window_count += processed;
    last_write_total = current_write_total;
    return processed;
}
