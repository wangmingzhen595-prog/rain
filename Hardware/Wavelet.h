#ifndef __WAVELET_H
#define __WAVELET_H

#include "stm32f10x.h"

// 小波配置
#define WAVELET_WINDOW_SIZE    64      // 小波窗口长度（2^6，便于DWT）
#define WAVELET_DWT_LEVELS     3       // DWT分解层数

// 特征结构体
typedef struct {
    uint32_t energy_high;      // 高频细节系数能量（Ed1+Ed2+Ed3）
    uint16_t hf_ratio;         // 高频能量比例（permille，0-1000）
    uint8_t success;           // 提取成功标志（1=成功，0=失败）
} WaveletFeatures_t;

// 函数声明（收敛接口）
uint8_t Wavelet_ExtractFeatures(uint16_t *signal, uint16_t signal_len,
                                uint16_t win_start, int32_t baseline,
                                WaveletFeatures_t *features);

// 兼容接口（保留end_index参数，但不使用）
// 注意：end_index参数保留是为了兼容主流程，但小波分析不使用
uint8_t Wavelet_ExtractFeatures_Compat(uint16_t *signal, uint16_t signal_len,
                                      uint16_t start_index, uint16_t end_index,
                                      int32_t baseline, WaveletFeatures_t *features);

#endif
