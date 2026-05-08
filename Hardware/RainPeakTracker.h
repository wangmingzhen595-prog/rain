#ifndef __RAIN_PEAK_TRACKER_H
#define __RAIN_PEAK_TRACKER_H

#include "stm32f10x.h"
#include <stdint.h>

#define RPT_ENABLE_PRIMARY_RAIN_PATH      1
#define RPT_BASELINE_INIT_SAMPLES         64U
#define RPT_BASELINE_IIR_SHIFT            8U
#define RPT_TRIGGER_MV                    100U
#define RPT_END_MV                        80U
#define RPT_MIN_PEAK_MV                   300U
#define RPT_BASELINE_IDLE_BAND_MV         20U
#define RPT_ADC_REF_MV                    3300U
#define RPT_ADC_MAX_VALUE                 4095U
#define RPT_ADC_CAL_NUM                   9155U
#define RPT_ADC_CAL_DEN                   10000U
#define RPT_SATURATION_ADC                4000U
#define RPT_IMPULSE_CAL_NUM_DEFAULT       1U
#define RPT_IMPULSE_CAL_DEN_DEFAULT       1U
#define RPT_TRIGGER_CONFIRM_SAMPLES       2U
#define RPT_END_CONFIRM_SAMPLES           3U
#define RPT_MIN_WIDTH_US                  80U
#define RPT_MAX_WIDTH_US                  12000U
#define RPT_DEADTIME_US                   2500U
#define RPT_EVENT_BUFFER_SIZE             384U
#define RPT_EVENT_QUEUE_SIZE              8U
#define RPT_SPLIT_MIN_GAP_US              200U
#define RPT_SPLIT_VALLEY_PERCENT          45U
#define RPT_MAX_SPLIT_PEAKS               4U

typedef enum {
    RPT_EVENT_KIND_SINGLE = 1,
    RPT_EVENT_KIND_SPLIT = 2,
    RPT_EVENT_KIND_COMPOSITE = 3
} RainPeakEventKind_t;

#define RPT_QUALITY_NONE                  0x00U
#define RPT_QUALITY_SATURATED             0x01U
#define RPT_QUALITY_OVERLAP               0x02U
#define RPT_QUALITY_TRUNCATED             0x04U
#define RPT_QUALITY_QUEUE_OVERFLOW        0x08U

typedef enum {
    RPT_STATE_LEARNING = 0,
    RPT_STATE_IDLE,
    RPT_STATE_ACTIVE,
    RPT_STATE_DEADTIME
} RainPeakState_t;

typedef enum {
    RPT_END_NONE = 0,
    RPT_END_POSITIVE_DECAY,
    RPT_END_TIMEOUT
} RainPeakEndReason_t;

typedef enum {
    RPT_REJECT_NONE = 0,
    RPT_REJECT_PEAK_TOO_SMALL,
    RPT_REJECT_WIDTH_TOO_SHORT,
    RPT_REJECT_TIMEOUT
} RainPeakRejectReason_t;

typedef struct {
    uint8_t  valid;
    uint8_t  saturated;
    uint8_t  end_reason;
    uint8_t  reject_reason;
    uint8_t  event_kind;
    uint8_t  quality_flags;
    uint8_t  sub_index;
    uint8_t  sub_count;
    uint32_t seq;
    uint32_t composite_seq;
    uint16_t baseline_adc;
    uint16_t peak_raw_adc;
    uint16_t peak_delta_adc;
    uint32_t start_sample;
    uint32_t peak_sample;
    uint32_t end_sample;
    uint32_t rise_samples;
    uint32_t width_samples;
    uint32_t area_adc_samples;
    uint32_t area_mv_us;
    uint32_t impulse_calibrated;
    uint32_t peak_mv;
    uint32_t rise_us;
    uint32_t width_us;
} RainPeakEvent_t;

typedef struct {
    RainPeakState_t state;
    uint32_t sample_rate_hz;
    uint32_t sample_index;
    uint32_t last_write_total;
    uint32_t event_seq;
    uint32_t baseline_acc;
    uint16_t baseline_adc;
    uint16_t baseline_sample_count;
    uint16_t trigger_confirm;
    uint16_t end_confirm;
    uint16_t trigger_delta_adc;
    uint16_t end_delta_adc;
    uint16_t min_peak_delta_adc;
    uint16_t baseline_idle_band_adc;
    uint16_t saturation_adc;
    uint32_t impulse_cal_num;
    uint32_t impulse_cal_den;
    uint8_t  event_saturated;
    uint8_t  last_end_reason;
    uint8_t  last_reject_reason;
    uint32_t deadtime_remaining;
    uint32_t start_sample;
    uint32_t peak_sample;
    uint16_t peak_raw_adc;
    uint16_t peak_delta_adc;
    uint32_t area_adc_samples;
    uint16_t trigger_raw_samples[RPT_TRIGGER_CONFIRM_SAMPLES];
    int16_t  trigger_delta_samples[RPT_TRIGGER_CONFIRM_SAMPLES];
    uint32_t trigger_sample_indices[RPT_TRIGGER_CONFIRM_SAMPLES];
    int16_t  event_buffer[RPT_EVENT_BUFFER_SIZE];
    uint16_t event_sample_count;
    uint8_t  event_truncated;
    uint32_t trigger_count;
    uint32_t published_count;
    uint32_t reject_count;
    uint32_t saturated_count;
    uint32_t split_count;
    uint32_t composite_count;
    uint32_t queue_overflow_count;
    uint32_t max_seen_delta_adc;
    RainPeakEvent_t latest_event;
    RainPeakEvent_t event_queue[RPT_EVENT_QUEUE_SIZE];
    volatile uint8_t event_queue_head;
    volatile uint8_t event_queue_tail;
    volatile uint8_t event_queue_count;
} RainPeakTrackerCtx_t;

void RainPeakTracker_Init(RainPeakTrackerCtx_t *ctx, uint32_t sample_rate_hz);
uint32_t RainPeakTracker_ProcessNewSamples(RainPeakTrackerCtx_t *ctx);
void RainPeakTracker_Update(RainPeakTrackerCtx_t *ctx, uint16_t raw_adc);
uint8_t RainPeakTracker_PopEvent(RainPeakTrackerCtx_t *ctx, RainPeakEvent_t *out_event);
uint32_t RainPeakTracker_AdcDeltaToMv(uint16_t adc_delta);
uint32_t RainPeakTracker_AreaAdcSamplesToMvUs(RainPeakTrackerCtx_t *ctx, uint32_t area_adc_samples);
uint32_t RainPeakTracker_CalibrateImpulse(RainPeakTrackerCtx_t *ctx, uint32_t area_mv_us);
void RainPeakTracker_SetImpulseCalibration(RainPeakTrackerCtx_t *ctx, uint32_t num, uint32_t den);
uint16_t RainPeakTracker_MvToAdcDelta(uint32_t mv);
uint32_t RainPeakTracker_SamplesToUs(RainPeakTrackerCtx_t *ctx, uint32_t samples);

#endif
