#ifndef __RAINDROP_H
#define __RAINDROP_H

#include <stdint.h>

/* ===================== 编译期配置 ===================== */
/* 灰区策略开关：默认灰区不计量；若设为 1，则 300–360mV 视为最小雨滴体积并计入 */
#define RAIN_GRAY_AS_MIN_DROP  0

/* ADC参考电压（mV），默认3300mV */
#define RAIN_ADC_REF_VOLTAGE_MV  3300

/* ADC分辨率：12位，最大值4095 */
#define RAIN_ADC_MAX_VALUE  4095

/* 死区时间（毫秒），建议30-80ms，可配置 */
#define RAIN_REFRACTORY_MS  50

/* ===================== 状态标志定义 ===================== */
#define RAIN_STATUS_NOISE  0  /* 无效/噪声 */
#define RAIN_STATUS_GRAY   1  /* 灰区可疑 */
#define RAIN_STATUS_OK     2  /* 计量有效 */

/* ===================== 函数接口 ===================== */

/**
 * @brief  初始化雨滴体积换算模块
 * @note   初始化LUT、累计量等，应在系统启动时调用一次
 */
void Raindrop_Init(void);

/**
 * @brief  处理一次雨滴事件（从ADC峰值到体积累计的完整流程）
 * @param  adc_peak: 单次事件的峰值ADC值（12位，0-4095）
 * @note   内部完成：ADC→mV、门限判定、查表+插值、累计
 *         包含死区时间控制，避免同一滴雨被多次计量
 */
void Raindrop_ProcessOneDrop(uint16_t adc_peak);

/**
 * @brief  电压转体积（定点整数实现）
 * @param  U_meas_mV: 测量电压值（单位：mV）
 * @param  status_flag: 输出参数，状态标志（0=NOISE, 1=GRAY, 2=OK）
 * @retval 体积值（单位：0.01mm³）
 * @note   使用查表+二分查找+线性插值，全程定点整数计算
 */
uint32_t Raindrop_VoltageToVolume_0p01mm3(uint32_t U_meas_mV, uint8_t *status_flag);

/**
 * @brief  获取累计体积总量
 * @retval 累计体积（单位：0.01mm³）
 * @note   主控可通过此函数读取累计值，然后做差分计算雨量增量
 */
uint32_t Raindrop_GetTotalVolume_0p01mm3(void);

/**
 * @brief  重置累计体积（用于清零或标定）
 */
void Raindrop_ResetTotalVolume(void);

/**
 * @brief  直接累加体积（简化接口）
 * @param  volume_0p01mm3: 要累加的体积值（单位：0.01mm³）
 */
void Raindrop_AddVolume(uint32_t volume_0p01mm3);

/**
 * @brief  死区时间递减更新（应在主循环中每10ms调用一次）
 * @note   用于递减死区计数器，允许下一次雨滴事件进入计量流程
 */
void Raindrop_UpdateRefractory(void);

#endif /* __RAINDROP_H */
