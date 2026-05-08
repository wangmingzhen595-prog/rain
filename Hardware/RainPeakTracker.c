#include "RainPeakTracker.h"
#include "AD.h"

typedef struct {
    uint16_t idx;
    int16_t value;
} RPT_LocalPeak_t;

static uint16_t RPT_AbsDelta(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static int16_t RPT_ClipInt16(int32_t value)
{
    if (value > 32767)
        return 32767;
    if (value < -32768)
        return -32768;
    return (int16_t)value;
}

static uint32_t RPT_UsToSamples(RainPeakTrackerCtx_t *ctx, uint32_t us)
{
    uint32_t rate = ctx->sample_rate_hz;
    uint64_t samples;

    if (rate == 0)
        rate = 23810U;

    samples = ((uint64_t)rate * (uint64_t)us + 999999ULL) / 1000000ULL;
    if (samples < 1ULL)
        samples = 1ULL;
    if (samples > 0xFFFFFFFFULL)
        samples = 0xFFFFFFFFULL;

    return (uint32_t)samples;
}

static void RPT_ResetTriggerFields(RainPeakTrackerCtx_t *ctx)
{
    ctx->trigger_confirm = 0;
}

static void RPT_ResetActiveFields(RainPeakTrackerCtx_t *ctx)
{
    RPT_ResetTriggerFields(ctx);
    ctx->end_confirm = 0;
    ctx->event_saturated = 0;
    ctx->event_truncated = 0;
    ctx->event_sample_count = 0;
    ctx->start_sample = 0;
    ctx->peak_sample = 0;
    ctx->peak_raw_adc = 0;
    ctx->peak_delta_adc = 0;
    ctx->area_adc_samples = 0;
}

static void RPT_AppendEventSample(RainPeakTrackerCtx_t *ctx, int32_t y)
{
    if (ctx->event_sample_count < RPT_EVENT_BUFFER_SIZE)
    {
        ctx->event_buffer[ctx->event_sample_count] = RPT_ClipInt16(y);
        ctx->event_sample_count++;
    }
    else
    {
        ctx->event_truncated = 1;
    }
}

static void RPT_FillEventDerivedArea(RainPeakTrackerCtx_t *ctx, RainPeakEvent_t *event)
{
    event->area_mv_us =
        RainPeakTracker_AreaAdcSamplesToMvUs(ctx, event->area_adc_samples);
    event->impulse_calibrated =
        RainPeakTracker_CalibrateImpulse(ctx, event->area_mv_us);
}

static uint8_t RPT_PushEvent(RainPeakTrackerCtx_t *ctx, RainPeakEvent_t *event)
{
    if (ctx->event_queue_count >= RPT_EVENT_QUEUE_SIZE)
    {
        ctx->queue_overflow_count++;
        ctx->latest_event = *event;
        ctx->latest_event.quality_flags |= RPT_QUALITY_QUEUE_OVERFLOW;
        return 0;
    }

    event->seq = ++ctx->event_seq;
    ctx->latest_event = *event;
    ctx->event_queue[ctx->event_queue_head] = *event;
    ctx->event_queue_head = (uint8_t)((ctx->event_queue_head + 1U) % RPT_EVENT_QUEUE_SIZE);
    ctx->event_queue_count++;

    ctx->published_count++;
    if (event->saturated)
        ctx->saturated_count++;

    return 1;
}

static void RPT_StoreRejected(RainPeakTrackerCtx_t *ctx,
                              uint8_t end_reason,
                              uint8_t reject_reason,
                              uint32_t width)
{
    RainPeakEvent_t event;
    uint8_t *p = (uint8_t *)&event;
    uint32_t i;

    for (i = 0; i < sizeof(RainPeakEvent_t); i++)
        p[i] = 0;

    ctx->last_end_reason = end_reason;
    ctx->last_reject_reason = reject_reason;
    ctx->reject_count++;

    event.valid = 0;
    event.saturated = ctx->event_saturated;
    event.end_reason = end_reason;
    event.reject_reason = reject_reason;
    event.event_kind = RPT_EVENT_KIND_SINGLE;
    event.quality_flags = ctx->event_truncated ? RPT_QUALITY_TRUNCATED : RPT_QUALITY_NONE;
    if (event.saturated)
        event.quality_flags |= RPT_QUALITY_SATURATED;
    event.sub_index = 1;
    event.sub_count = 1;
    event.composite_seq = ctx->trigger_count;
    event.baseline_adc = ctx->baseline_adc;
    event.peak_raw_adc = ctx->peak_raw_adc;
    event.peak_delta_adc = ctx->peak_delta_adc;
    event.start_sample = ctx->start_sample;
    event.peak_sample = ctx->peak_sample;
    event.end_sample = ctx->sample_index;
    event.rise_samples = ctx->peak_sample - ctx->start_sample;
    event.width_samples = width;
    event.area_adc_samples = ctx->area_adc_samples;
    RPT_FillEventDerivedArea(ctx, &event);
    event.peak_mv = RainPeakTracker_AdcDeltaToMv(ctx->peak_delta_adc);
    event.rise_us = RainPeakTracker_SamplesToUs(ctx, event.rise_samples);
    event.width_us = RainPeakTracker_SamplesToUs(ctx, width);

    ctx->latest_event = event;
}

static uint8_t RPT_FindLocalPeaks(RainPeakTrackerCtx_t *ctx,
                                  RPT_LocalPeak_t *peaks,
                                  uint8_t max_peaks)
{
    uint16_t i;
    uint8_t count = 0;
    uint32_t min_gap = RPT_UsToSamples(ctx, RPT_SPLIT_MIN_GAP_US);

    if (ctx->event_sample_count < 3U)
        return 0;

    for (i = 1; i < (uint16_t)(ctx->event_sample_count - 1U); i++)
    {
        int16_t y = ctx->event_buffer[i];
        if (y >= (int16_t)ctx->min_peak_delta_adc &&
            y >= ctx->event_buffer[i - 1U] &&
            y > ctx->event_buffer[i + 1U])
        {
            if (count > 0U &&
                (uint32_t)(i - peaks[count - 1U].idx) < min_gap)
            {
                if (y > peaks[count - 1U].value)
                {
                    peaks[count - 1U].idx = i;
                    peaks[count - 1U].value = y;
                }
            }
            else if (count < max_peaks)
            {
                peaks[count].idx = i;
                peaks[count].value = y;
                count++;
            }
        }
    }

    return count;
}

static uint16_t RPT_FindValleyIndex(RainPeakTrackerCtx_t *ctx,
                                    uint16_t left,
                                    uint16_t right,
                                    int16_t *out_value)
{
    uint16_t i;
    uint16_t best = left;
    int16_t best_value = ctx->event_buffer[left];

    for (i = left; i <= right; i++)
    {
        if (ctx->event_buffer[i] < best_value)
        {
            best = i;
            best_value = ctx->event_buffer[i];
        }
    }

    if (out_value != 0)
        *out_value = best_value;

    return best;
}

static void RPT_BuildSegmentEvent(RainPeakTrackerCtx_t *ctx,
                                  uint16_t seg_start,
                                  uint16_t seg_end,
                                  uint8_t kind,
                                  uint8_t quality,
                                  uint8_t sub_index,
                                  uint8_t sub_count,
                                  uint8_t end_reason,
                                  RainPeakEvent_t *event)
{
    uint32_t area = 0;
    uint16_t i;
    uint16_t peak_idx = seg_start;
    int16_t peak_delta = 0;
    uint8_t saturated = 0;
    uint8_t *p = (uint8_t *)event;

    for (i = 0; i < sizeof(RainPeakEvent_t); i++)
        p[i] = 0;

    if (seg_end >= ctx->event_sample_count)
        seg_end = (uint16_t)(ctx->event_sample_count - 1U);

    for (i = seg_start; i <= seg_end; i++)
    {
        int16_t y = ctx->event_buffer[i];
        int32_t raw_est = (int32_t)ctx->baseline_adc + (int32_t)y;

        if (y > 0)
            area += (uint32_t)y;
        if (y > peak_delta)
        {
            peak_delta = y;
            peak_idx = i;
        }
        if (raw_est >= (int32_t)ctx->saturation_adc)
            saturated = 1;
    }

    event->valid = 1;
    event->saturated = saturated ? 1U : ctx->event_saturated;
    event->end_reason = end_reason;
    event->reject_reason = RPT_REJECT_NONE;
    event->event_kind = kind;
    event->quality_flags = quality;
    if (event->saturated)
        event->quality_flags |= RPT_QUALITY_SATURATED;
    if (ctx->event_truncated)
        event->quality_flags |= RPT_QUALITY_TRUNCATED;
    event->sub_index = sub_index;
    event->sub_count = sub_count;
    event->composite_seq = ctx->trigger_count;
    event->baseline_adc = ctx->baseline_adc;
    event->peak_delta_adc = (uint16_t)((peak_delta > 0) ? peak_delta : 0);
    event->peak_raw_adc = (uint16_t)((uint32_t)ctx->baseline_adc + event->peak_delta_adc);
    if (event->peak_raw_adc > RPT_ADC_MAX_VALUE)
        event->peak_raw_adc = RPT_ADC_MAX_VALUE;
    event->start_sample = ctx->start_sample + seg_start;
    event->peak_sample = ctx->start_sample + peak_idx;
    event->end_sample = ctx->start_sample + seg_end;
    event->rise_samples = event->peak_sample - event->start_sample;
    event->width_samples = (uint32_t)(seg_end - seg_start + 1U);
    event->area_adc_samples = area;
    RPT_FillEventDerivedArea(ctx, event);
    event->peak_mv = RainPeakTracker_AdcDeltaToMv(event->peak_delta_adc);
    event->rise_us = RainPeakTracker_SamplesToUs(ctx, event->rise_samples);
    event->width_us = RainPeakTracker_SamplesToUs(ctx, event->width_samples);
}

static void RPT_FinalizeValidEvent(RainPeakTrackerCtx_t *ctx,
                                   uint8_t end_reason)
{
    RPT_LocalPeak_t peaks[RPT_MAX_SPLIT_PEAKS];
    uint16_t split_points[RPT_MAX_SPLIT_PEAKS - 1U];
    uint8_t peak_count;
    uint8_t split_count = 0;
    uint8_t can_split;
    uint8_t i;
    uint32_t min_gap = RPT_UsToSamples(ctx, RPT_SPLIT_MIN_GAP_US);

    peak_count = RPT_FindLocalPeaks(ctx, peaks, RPT_MAX_SPLIT_PEAKS);
    can_split = (peak_count >= 2U) ? 1U : 0U;

    for (i = 0; i + 1U < peak_count && can_split; i++)
    {
        int16_t valley_value;
        uint16_t valley_idx = RPT_FindValleyIndex(ctx, peaks[i].idx, peaks[i + 1U].idx, &valley_value);
        int32_t smaller_peak = (peaks[i].value < peaks[i + 1U].value) ? peaks[i].value : peaks[i + 1U].value;
        int32_t valley_limit = (smaller_peak * (int32_t)RPT_SPLIT_VALLEY_PERCENT) / 100;

        if ((uint32_t)(peaks[i + 1U].idx - peaks[i].idx) >= min_gap &&
            valley_value <= valley_limit)
        {
            split_points[split_count] = valley_idx;
            split_count++;
        }
        else
        {
            can_split = 0;
        }
    }

    if (can_split && split_count > 0U)
    {
        uint16_t seg_start = 0;
        ctx->split_count++;
        for (i = 0; i <= split_count; i++)
        {
            uint16_t seg_end = (i < split_count) ? split_points[i] : (uint16_t)(ctx->event_sample_count - 1U);
            RainPeakEvent_t event;
            RPT_BuildSegmentEvent(ctx, seg_start, seg_end,
                                  RPT_EVENT_KIND_SPLIT,
                                  RPT_QUALITY_NONE,
                                  (uint8_t)(i + 1U),
                                  (uint8_t)(split_count + 1U),
                                  end_reason,
                                  &event);
            RPT_PushEvent(ctx, &event);
            seg_start = (uint16_t)(seg_end + 1U);
        }
    }
    else
    {
        RainPeakEvent_t event;
        uint8_t kind = (peak_count >= 2U) ? RPT_EVENT_KIND_COMPOSITE : RPT_EVENT_KIND_SINGLE;
        uint8_t quality = (peak_count >= 2U) ? RPT_QUALITY_OVERLAP : RPT_QUALITY_NONE;

        if (kind == RPT_EVENT_KIND_COMPOSITE)
            ctx->composite_count++;

        RPT_BuildSegmentEvent(ctx, 0, (uint16_t)(ctx->event_sample_count - 1U),
                              kind,
                              quality,
                              1,
                              1,
                              end_reason,
                              &event);
        RPT_PushEvent(ctx, &event);
    }
}

static void RPT_FinalizeEvent(RainPeakTrackerCtx_t *ctx,
                              uint8_t end_reason,
                              uint32_t width,
                              uint32_t min_width_samples)
{
    if (end_reason == RPT_END_TIMEOUT)
    {
        RPT_StoreRejected(ctx, end_reason, RPT_REJECT_TIMEOUT, width);
        return;
    }

    if (ctx->peak_delta_adc < ctx->min_peak_delta_adc)
    {
        RPT_StoreRejected(ctx, end_reason, RPT_REJECT_PEAK_TOO_SMALL, width);
        return;
    }

    if (width < min_width_samples)
    {
        RPT_StoreRejected(ctx, end_reason, RPT_REJECT_WIDTH_TOO_SHORT, width);
        return;
    }

    ctx->last_end_reason = end_reason;
    ctx->last_reject_reason = RPT_REJECT_NONE;
    RPT_FinalizeValidEvent(ctx, end_reason);
}

static void RPT_BeginActive(RainPeakTrackerCtx_t *ctx)
{
    uint16_t i;

    ctx->trigger_count++;
    ctx->state = RPT_STATE_ACTIVE;
    ctx->start_sample = ctx->trigger_sample_indices[0];
    ctx->peak_sample = ctx->start_sample;
    ctx->peak_raw_adc = ctx->trigger_raw_samples[0];
    ctx->peak_delta_adc = 0;
    ctx->area_adc_samples = 0;
    ctx->event_saturated = 0;
    ctx->event_truncated = 0;
    ctx->event_sample_count = 0;
    ctx->end_confirm = 0;

    for (i = 0; i < ctx->trigger_confirm; i++)
    {
        int16_t y = ctx->trigger_delta_samples[i];
        RPT_AppendEventSample(ctx, y);
        if (ctx->trigger_raw_samples[i] >= ctx->saturation_adc)
            ctx->event_saturated = 1;
        if (y > 0)
            ctx->area_adc_samples += (uint32_t)y;
        if (y > (int16_t)ctx->peak_delta_adc)
        {
            ctx->peak_delta_adc = (uint16_t)y;
            ctx->peak_raw_adc = ctx->trigger_raw_samples[i];
            ctx->peak_sample = ctx->trigger_sample_indices[i];
            if (ctx->peak_delta_adc > ctx->max_seen_delta_adc)
                ctx->max_seen_delta_adc = ctx->peak_delta_adc;
        }
    }

    RPT_ResetTriggerFields(ctx);
}

uint32_t RainPeakTracker_SamplesToUs(RainPeakTrackerCtx_t *ctx, uint32_t samples)
{
    uint32_t rate = ctx->sample_rate_hz;

    if (rate == 0)
        rate = 23810U;

    return (uint32_t)(((uint64_t)samples * 1000000ULL + rate / 2U) / rate);
}

uint32_t RainPeakTracker_AdcDeltaToMv(uint16_t adc_delta)
{
    uint64_t numerator = (uint64_t)adc_delta * RPT_ADC_REF_MV * RPT_ADC_CAL_NUM;
    uint64_t denominator = (uint64_t)RPT_ADC_MAX_VALUE * RPT_ADC_CAL_DEN;

    return (uint32_t)((numerator + denominator / 2ULL) / denominator);
}

uint32_t RainPeakTracker_AreaAdcSamplesToMvUs(RainPeakTrackerCtx_t *ctx, uint32_t area_adc_samples)
{
    uint32_t rate = ctx->sample_rate_hz;
    uint64_t mv_samples;
    uint64_t mv_us;
    uint64_t denominator;

    if (rate == 0)
        rate = 23810U;

    denominator = (uint64_t)RPT_ADC_MAX_VALUE * RPT_ADC_CAL_DEN;
    mv_samples = ((uint64_t)area_adc_samples * RPT_ADC_REF_MV * RPT_ADC_CAL_NUM
                  + denominator / 2ULL) / denominator;
    mv_us = (mv_samples * 1000000ULL + rate / 2U) / rate;

    if (mv_us > 0xFFFFFFFFULL)
        mv_us = 0xFFFFFFFFULL;

    return (uint32_t)mv_us;
}

uint32_t RainPeakTracker_CalibrateImpulse(RainPeakTrackerCtx_t *ctx, uint32_t area_mv_us)
{
    uint32_t den = ctx->impulse_cal_den;
    uint64_t impulse;

    if (den == 0)
        den = RPT_IMPULSE_CAL_DEN_DEFAULT;

    impulse = ((uint64_t)area_mv_us * ctx->impulse_cal_num + den / 2U) / den;
    if (impulse > 0xFFFFFFFFULL)
        impulse = 0xFFFFFFFFULL;

    return (uint32_t)impulse;
}

void RainPeakTracker_SetImpulseCalibration(RainPeakTrackerCtx_t *ctx, uint32_t num, uint32_t den)
{
    if (num == 0)
        num = RPT_IMPULSE_CAL_NUM_DEFAULT;
    if (den == 0)
        den = RPT_IMPULSE_CAL_DEN_DEFAULT;

    ctx->impulse_cal_num = num;
    ctx->impulse_cal_den = den;
}

uint16_t RainPeakTracker_MvToAdcDelta(uint32_t mv)
{
    uint64_t numerator = (uint64_t)mv * RPT_ADC_MAX_VALUE * RPT_ADC_CAL_DEN;
    uint64_t denominator = (uint64_t)RPT_ADC_REF_MV * RPT_ADC_CAL_NUM;
    uint64_t adc = (numerator + denominator - 1ULL) / denominator;

    if (adc > RPT_ADC_MAX_VALUE)
        adc = RPT_ADC_MAX_VALUE;

    return (uint16_t)adc;
}

void RainPeakTracker_Init(RainPeakTrackerCtx_t *ctx, uint32_t sample_rate_hz)
{
    uint8_t *p = (uint8_t *)ctx;
    uint32_t i;

    for (i = 0; i < sizeof(RainPeakTrackerCtx_t); i++)
        p[i] = 0;

    ctx->state = RPT_STATE_LEARNING;
    ctx->sample_rate_hz = sample_rate_hz;
    ctx->last_write_total = ring_write_total_ch0;
    ctx->trigger_delta_adc = RainPeakTracker_MvToAdcDelta(RPT_TRIGGER_MV);
    ctx->end_delta_adc = RainPeakTracker_MvToAdcDelta(RPT_END_MV);
    ctx->min_peak_delta_adc = RainPeakTracker_MvToAdcDelta(RPT_MIN_PEAK_MV);
    ctx->baseline_idle_band_adc = RainPeakTracker_MvToAdcDelta(RPT_BASELINE_IDLE_BAND_MV);
    ctx->saturation_adc = RPT_SATURATION_ADC;
    RainPeakTracker_SetImpulseCalibration(ctx,
                                          RPT_IMPULSE_CAL_NUM_DEFAULT,
                                          RPT_IMPULSE_CAL_DEN_DEFAULT);
}

uint8_t RainPeakTracker_PopEvent(RainPeakTrackerCtx_t *ctx, RainPeakEvent_t *out_event)
{
    if (ctx->event_queue_count == 0U)
        return 0;

    if (out_event != 0)
        *out_event = ctx->event_queue[ctx->event_queue_tail];

    ctx->event_queue_tail = (uint8_t)((ctx->event_queue_tail + 1U) % RPT_EVENT_QUEUE_SIZE);
    ctx->event_queue_count--;
    return 1;
}

void RainPeakTracker_Update(RainPeakTrackerCtx_t *ctx, uint16_t raw_adc)
{
    int32_t y;
    uint32_t min_width_samples;
    uint32_t max_width_samples;
    uint32_t width;
    uint8_t end_reason = RPT_END_NONE;

    ctx->sample_index++;

    if (ctx->state == RPT_STATE_LEARNING)
    {
        ctx->baseline_acc += raw_adc;
        ctx->baseline_sample_count++;
        if (ctx->baseline_sample_count >= RPT_BASELINE_INIT_SAMPLES)
        {
            ctx->baseline_adc = (uint16_t)(ctx->baseline_acc / RPT_BASELINE_INIT_SAMPLES);
            ctx->state = RPT_STATE_IDLE;
        }
        return;
    }

    y = (int32_t)raw_adc - (int32_t)ctx->baseline_adc;

    if (ctx->state == RPT_STATE_DEADTIME)
    {
        if (ctx->deadtime_remaining > 0)
            ctx->deadtime_remaining--;
        else if (y <= (int32_t)ctx->end_delta_adc)
            ctx->state = RPT_STATE_IDLE;
        return;
    }

    if (ctx->state == RPT_STATE_IDLE)
    {
        if (RPT_AbsDelta(raw_adc, ctx->baseline_adc) <= ctx->baseline_idle_band_adc)
        {
            int32_t base = (int32_t)ctx->baseline_adc;
            base += ((int32_t)raw_adc - base) >> RPT_BASELINE_IIR_SHIFT;
            if (base < 0) base = 0;
            if (base > RPT_ADC_MAX_VALUE) base = RPT_ADC_MAX_VALUE;
            ctx->baseline_adc = (uint16_t)base;
        }

        if (y >= (int32_t)ctx->trigger_delta_adc)
        {
            uint16_t pos = ctx->trigger_confirm;
            if (pos < RPT_TRIGGER_CONFIRM_SAMPLES)
            {
                ctx->trigger_raw_samples[pos] = raw_adc;
                ctx->trigger_delta_samples[pos] = RPT_ClipInt16(y);
                ctx->trigger_sample_indices[pos] = ctx->sample_index;
                ctx->trigger_confirm++;
            }

            if (ctx->trigger_confirm >= RPT_TRIGGER_CONFIRM_SAMPLES)
                RPT_BeginActive(ctx);
        }
        else
        {
            RPT_ResetTriggerFields(ctx);
        }
        return;
    }

    if (ctx->state == RPT_STATE_ACTIVE)
    {
        RPT_AppendEventSample(ctx, y);

        if (raw_adc >= ctx->saturation_adc)
            ctx->event_saturated = 1;

        if (y > 0)
            ctx->area_adc_samples += (uint32_t)y;

        if (y > (int32_t)ctx->peak_delta_adc)
        {
            ctx->peak_delta_adc = (uint16_t)y;
            ctx->peak_raw_adc = raw_adc;
            ctx->peak_sample = ctx->sample_index;
            if (ctx->peak_delta_adc > ctx->max_seen_delta_adc)
                ctx->max_seen_delta_adc = ctx->peak_delta_adc;
        }

        if (ctx->sample_index > ctx->peak_sample && y <= (int32_t)ctx->end_delta_adc)
            ctx->end_confirm++;
        else
            ctx->end_confirm = 0;

        min_width_samples = RPT_UsToSamples(ctx, RPT_MIN_WIDTH_US);
        max_width_samples = RPT_UsToSamples(ctx, RPT_MAX_WIDTH_US);
        width = ctx->sample_index - ctx->start_sample + 1U;

        if (ctx->end_confirm >= RPT_END_CONFIRM_SAMPLES)
            end_reason = RPT_END_POSITIVE_DECAY;
        else if (width >= max_width_samples)
            end_reason = RPT_END_TIMEOUT;

        if (end_reason != RPT_END_NONE)
        {
            RPT_FinalizeEvent(ctx, end_reason, width, min_width_samples);
            RPT_ResetActiveFields(ctx);
            ctx->deadtime_remaining = RPT_UsToSamples(ctx, RPT_DEADTIME_US);
            ctx->state = RPT_STATE_DEADTIME;
        }
    }
}

uint32_t RainPeakTracker_ProcessNewSamples(RainPeakTrackerCtx_t *ctx)
{
    uint32_t current_total = ring_write_total_ch0;
    uint32_t available = current_total - ctx->last_write_total;
    uint32_t processed = 0;
    uint16_t start_idx;

    if (available == 0)
        return 0;

    if (available > RING_BUFFER_SIZE)
        available = RING_BUFFER_SIZE;

    start_idx = (uint16_t)((ring_write_index_ch0 + RING_BUFFER_SIZE - (uint16_t)available) % RING_BUFFER_SIZE);
    while (processed < available)
    {
        uint16_t idx = (uint16_t)((start_idx + (uint16_t)processed) % RING_BUFFER_SIZE);
        RainPeakTracker_Update(ctx, adc_ring_buffer_ch0[idx]);
        processed++;
    }

    ctx->last_write_total = current_total;
    return processed;
}
