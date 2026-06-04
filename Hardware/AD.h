#ifndef __AD_H
#define __AD_H

#include "stm32f10x.h"

extern uint16_t AD_Value[200];  // 双通道扫描模式：100个PA0+100个PA1（交替存储）

void AD_Init(void);

/* ADC sampling profile:
 * ADC clock = 72MHz / 6 = 12MHz.
 * Two-channel scan mode means PA0 receives one sample every two conversions.
 * 239.5 cycles: about 42.00us/PA0 sample, original conservative setting.
 *  71.5 cycles: about 14.00us/PA0 sample, recommended first fast setting.
 *  55.5 cycles: about 11.33us/PA0 sample, use after analog front-end verification.
 */
#define RAIN_ADC_SAMPLE_TIME        ADC_SampleTime_71Cycles5
#define ADC_SAMPLE_INTERVAL_NS      14000UL
#define ADC_SAMPLE_INTERVAL_US      14.0f

/* 采样与峰值抓取扩展 */
#define RING_BUFFER_SIZE 500
#define ISR_CAPTURE_PRE_US          350U
#define ISR_CAPTURE_POST_US         1400U
#define ISR_CAPTURE_PRE_SAMPLES     ((((uint32_t)ISR_CAPTURE_PRE_US * 1000UL) + ADC_SAMPLE_INTERVAL_NS - 1UL) / ADC_SAMPLE_INTERVAL_NS)
#define ISR_CAPTURE_POST_SAMPLES    ((((uint32_t)ISR_CAPTURE_POST_US * 1000UL) + ADC_SAMPLE_INTERVAL_NS - 1UL) / ADC_SAMPLE_INTERVAL_NS)
#define ISR_CAPTURE_WINDOW_SIZE     (ISR_CAPTURE_PRE_SAMPLES + 1U + ISR_CAPTURE_POST_SAMPLES)
#define ISR_CAPTURE_BASELINE_SAMPLES 32U
#define SNAPSHOT_PRE_SAMPLES 200
#define SNAPSHOT_POST_SAMPLES 300
#define SNAPSHOT_SIZE (SNAPSHOT_PRE_SAMPLES + SNAPSHOT_POST_SAMPLES)

/* 双通道环形缓冲区（分别为两路信号） */
extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];  // 通道0环形缓冲区
extern volatile uint16_t adc_ring_buffer_ch1[RING_BUFFER_SIZE];  // 通道1环形缓冲区
extern volatile uint16_t ring_write_index_ch0;  // 通道0写索引
extern volatile uint16_t ring_write_index_ch1;  // 通道1写索引
extern volatile uint16_t last_peak_window_from_isr[ISR_CAPTURE_WINDOW_SIZE]; // ISR锁定峰值时复制的主脉冲窗口
extern volatile uint16_t last_peak_window_len_from_isr; // ISR窗口实际有效长度
extern volatile int32_t last_peak_baseline_from_isr; // ISR窗口对应的峰前基线


extern volatile uint8_t snapshot_ready;
extern volatile uint16_t snapshot_buffer_high[SNAPSHOT_SIZE];
extern volatile uint16_t snapshot_buffer_low[SNAPSHOT_SIZE];
extern volatile uint16_t snapshot_write_index;
extern volatile uint8_t snapshot_collecting;
extern volatile uint16_t snapshot_peak_value;
extern volatile uint16_t snapshot_peak_index;
extern volatile uint16_t snapshot_trigger_index_ch0; /* 快照触发时的PA0环形缓冲区索引 */

extern volatile uint32_t sampling_tick_counter;

/* Keil Array Visualization 可观察数组（用于调试） */
extern volatile uint16_t ADC_Visualize_Buffer[500];  // ADC可视化缓冲区（通道0数据）
extern volatile uint16_t ADC_Visualize_Index;         // 可视化缓冲区写索引

extern volatile uint16_t dbg_pair_pa0_last;
extern volatile uint16_t dbg_pair_pa1_last;
extern volatile uint16_t dbg_pair_write_idx;
extern volatile uint32_t dbg_pair_write_count;

/* 阈值接口（ADC单位，默认200，可在main中覆盖） */
extern volatile uint16_t ADC_Threshold;
void AD_SetThreshold(uint16_t threshold);

/* 自恢复接口 */
void AD_Restart(void);

#endif
