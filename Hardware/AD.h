#ifndef __AD_H
#define __AD_H

#include "stm32f10x.h"

extern uint16_t AD_Value[100];

void AD_Init(void);

/* 采样与峰值抓取扩展 */
#define RING_BUFFER_SIZE 1000
#define SNAPSHOT_PRE_SAMPLES 200
#define SNAPSHOT_POST_SAMPLES 800
#define SNAPSHOT_SIZE (SNAPSHOT_PRE_SAMPLES + SNAPSHOT_POST_SAMPLES)

/* 双通道环形缓冲区（分别为两路信号） */
extern volatile uint16_t adc_ring_buffer_ch0[RING_BUFFER_SIZE];  // 通道0环形缓冲区
extern volatile uint16_t adc_ring_buffer_ch1[RING_BUFFER_SIZE];  // 通道1环形缓冲区
extern volatile uint16_t ring_write_index_ch0;  // 通道0写索引
extern volatile uint16_t ring_write_index_ch1;  // 通道1写索引


extern volatile uint8_t snapshot_ready;
extern volatile uint16_t snapshot_buffer_high[SNAPSHOT_SIZE];
extern volatile uint16_t snapshot_buffer_low[SNAPSHOT_SIZE];
extern volatile uint16_t snapshot_write_index;
extern volatile uint8_t snapshot_collecting;
extern volatile uint16_t snapshot_peak_value;
extern volatile uint16_t snapshot_peak_index;

extern volatile uint32_t sampling_tick_counter;

/* Keil Array Visualization 可观察数组（用于调试） */
extern volatile uint16_t ADC_Visualize_Buffer[1000];  // ADC可视化缓冲区（通道0数据）
extern volatile uint16_t ADC_Visualize_Index;         // 可视化缓冲区写索引

/* 阈值接口（ADC单位，默认200，可在main中覆盖） */
extern volatile uint16_t ADC_Threshold;
void AD_SetThreshold(uint16_t threshold);

/* 自恢复接口 */
void AD_Restart(void);

#endif
