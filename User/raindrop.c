#include "raindrop.h"
#include <stdint.h>

/* 电压校准系数（与main.c保持一致） */
#ifndef ADC_VOLTAGE_CALIBRATION_FACTOR
#define ADC_VOLTAGE_CALIBRATION_FACTOR  0.9155f  // 电压校准系数（示波器2.60V对应单片机2.84V：2.60/2.84≈0.9155）
#endif

/* ===================== LUT数据表（电压mV，体积0.01mm³） ===================== */
/* 基于实验数据：U(V): 0.36, 1.00, 1.10, 1.16, 1.24, 1.36, 1.40, 1.54, 1.56, 1.60, 1.80, 2.54, 2.84, 3.00 */
/* V(mm³): 4.17, 8.474, 9.259, 10.416, 11.494, 15.625, 17.857, 19.047, 20.979, 25.000, 30.075, 31.847, 35.714, 38.461 */
/* 转换为整数：电压以mV，体积以0.01mm³为单位 */

#define LUT_SIZE  14

/* 电压LUT（mV），严格单调递增 */
static const uint16_t U_tab_mv[LUT_SIZE] = {
    360,  1000, 1100, 1160, 1240, 1360, 1400, 
    1540, 1560, 1600, 1800, 2540, 2840, 3000
};

/* 体积LUT（0.01mm³），严格单调递增 */
static const uint32_t V_tab_0p01mm3[LUT_SIZE] = {
    417,  847,  926,  1042, 1149, 1563, 1786,
    1905, 2098, 2500, 3008, 3185, 3571, 3846
};

/* ===================== 全局变量 ===================== */
/* 累计体积（单位：0.01mm³），单调递增，供外部通过 Raindrop_GetTotalVolume_0p01mm3() 读取 */
uint32_t g_V_sum_0p01mm3 = 0;

/* 死区时间控制 */
static uint32_t g_refractory_counter = 0;  /* 死区计数器（主循环次数，每10ms递减） */
#define REFRACTORY_COUNTER_MAX  (RAIN_REFRACTORY_MS / 10)  /* 死区时间对应的主循环次数 */

/* ===================== 内部函数 ===================== */

/**
 * @brief  二分查找：在U_tab_mv中找到U_meas_mV所在的区间
 * @param  U_meas_mV: 测量电压值（mV）
 * @retval 区间索引k，满足 U_tab_mv[k] <= U_meas_mV <= U_tab_mv[k+1]
 *         如果U_meas_mV < U_tab_mv[0]，返回0
 *         如果U_meas_mV >= U_tab_mv[last]，返回LUT_SIZE-2（最后一个有效区间）
 */
static uint8_t BinarySearch_Voltage(uint32_t U_meas_mV)
{
    uint8_t left = 0;
    uint8_t right = LUT_SIZE - 1;
    uint8_t mid;
    
    /* 边界处理：小于最小值 */
    if (U_meas_mV < U_tab_mv[0])
    {
        return 0;
    }
    
    /* 边界处理：大于等于最大值 */
    if (U_meas_mV >= U_tab_mv[LUT_SIZE - 1])
    {
        return LUT_SIZE - 2;  /* 返回最后一个有效区间 */
    }
    
    /* 二分查找 */
    while (left < right - 1)
    {
        mid = (left + right) / 2;
        if (U_tab_mv[mid] <= U_meas_mV)
        {
            left = mid;
        }
        else
        {
            right = mid;
        }
    }
    
    return left;
}

/**
 * @brief  ADC值转换为电压（mV）
 * @param  adc_value: ADC原始值（0-4095）
 * @retval 电压值（mV）
 * @note   使用统一公式：U_mV = adc * Vref_mV / 4095 * ADC_VOLTAGE_CALIBRATION_FACTOR
 *         与main.c中的Compute_Voltage_From_ADC保持一致
 */
static uint32_t ADC_To_Voltage_mV(uint16_t adc_value)
{
    /* 先乘后除，避免精度损失，并应用校准系数 */
    /* 注意：使用float中间计算，然后转换为整数，确保与main.c一致 */
    /* 公式：U_mV = (adc / 4095 * 3.3V) * 校准系数 * 1000 */
    /* 与main.c中的Compute_Voltage_From_ADC完全一致 */
    float voltage_V = ((float)adc_value / (float)RAIN_ADC_MAX_VALUE) * ((float)RAIN_ADC_REF_VOLTAGE_MV / 1000.0f) * ADC_VOLTAGE_CALIBRATION_FACTOR;
    uint32_t voltage_mV = (uint32_t)(voltage_V * 1000.0f + 0.5f);  /* 四舍五入到mV */
    return voltage_mV;
}

/* ===================== 公共函数实现 ===================== */

void Raindrop_Init(void)
{
    g_V_sum_0p01mm3 = 0;
    g_refractory_counter = 0;
}

uint32_t Raindrop_VoltageToVolume_0p01mm3(uint32_t U_meas_mV, uint8_t *status_flag)
{
    /* 严格边界处理：U < 300mV -> NOISE */
    if (U_meas_mV < 300)
    {
        *status_flag = RAIN_STATUS_NOISE;
        return 0;
    }

    /* 灰区处理：300 <= U < 360mV (U_tab_mv[0]) */
    if (U_meas_mV < U_tab_mv[0])  /* U_tab_mv[0] = 360 */
    {
        *status_flag = RAIN_STATUS_GRAY;
        #if (RAIN_GRAY_AS_MIN_DROP == 1)
            return V_tab_0p01mm3[0];
        #else
            return 0;
        #endif
    }

    /* 上限处理：U >= U_tab_mv[last] -> 上夹紧 */
    if (U_meas_mV >= U_tab_mv[LUT_SIZE - 1])
    {
        *status_flag = RAIN_STATUS_OK;
        return V_tab_0p01mm3[LUT_SIZE - 1];
    }

    /* 正常区间：U_tab_mv[0] <= U < U_tab_mv[last] -> 二分查找+插值 */
    *status_flag = RAIN_STATUS_OK;
    uint8_t k = BinarySearch_Voltage(U_meas_mV);
    
    /* 获取区间端点 */
    uint32_t U0 = U_tab_mv[k];
    uint32_t U1 = U_tab_mv[k + 1];
    uint32_t V0 = V_tab_0p01mm3[k];
    uint32_t V1 = V_tab_0p01mm3[k + 1];
    
    /* 线性插值：V = V0 + (V1-V0) * (U-U0) / (U1-U0) */
    /* 使用int64_t防止乘法溢出 */
    int64_t num = (int64_t)(V1 - V0) * (int64_t)(U_meas_mV - U0);
    int32_t den = (int32_t)(U1 - U0);
    
    /* 计算插值结果 */
    int32_t V = (int32_t)V0 + (int32_t)(num / den);
    
    /* 确保结果非负 */
    if (V < 0)
    {
        V = 0;
    }
    
    return (uint32_t)V;
}

uint32_t Raindrop_PeakMvToVolume_0p01mm3(uint32_t peak_mv, uint8_t *status_flag)
{
    return Raindrop_VoltageToVolume_0p01mm3(peak_mv, status_flag);
}

void Raindrop_ProcessOneDrop(uint16_t adc_peak)
{
    /* 死区时间检查：如果还在死区内，不处理新事件 */
    if (g_refractory_counter > 0)
    {
        return;
    }
    
    /* ADC转电压（mV） */
    uint32_t U_meas_mV = ADC_To_Voltage_mV(adc_peak);
    
    /* 电压转体积 */
    uint8_t status_flag;
    uint32_t V_drop_0p01mm3 = Raindrop_VoltageToVolume_0p01mm3(U_meas_mV, &status_flag);
    
    /* 累计条件判断 */
    uint8_t should_accumulate = 0;
    
    if (status_flag == RAIN_STATUS_OK)
    {
        should_accumulate = 1;
    }
    else if (status_flag == RAIN_STATUS_GRAY && RAIN_GRAY_AS_MIN_DROP == 1)
    {
        should_accumulate = 1;
    }
    
    /* 累计体积 */
    if (should_accumulate && V_drop_0p01mm3 > 0)
    {
        g_V_sum_0p01mm3 += V_drop_0p01mm3;
        
        /* 启动死区时间 */
        g_refractory_counter = REFRACTORY_COUNTER_MAX;
    }
    
    /* 调试输出（可选，用于排查问题） */
    /* 可以通过串口输出：ADC值、电压值、状态、体积值、累计值 */
}

uint32_t Raindrop_GetTotalVolume_0p01mm3(void)
{
    return g_V_sum_0p01mm3;
}

void Raindrop_ResetTotalVolume(void)
{
    g_V_sum_0p01mm3 = 0;
}

void Raindrop_AddVolume(uint32_t volume_0p01mm3)
{
    if (volume_0p01mm3 > 0)
    {
        g_V_sum_0p01mm3 += volume_0p01mm3;
    }
}

/**
 * @brief  死区时间递减（应在主循环中每10ms调用一次）
 * @note   此函数需要外部调用，建议在main.c的主循环中调用
 */
void Raindrop_UpdateRefractory(void)
{
    if (g_refractory_counter > 0)
    {
        g_refractory_counter--;
    }
}

/* 冲量→体积换算参数（实验标定，待现场数据验证） */
#ifndef RAIN_IMPULSE_SCALE
#define RAIN_IMPULSE_SCALE  2.0f  /* scale系数：V = scale × impulse / baseline */
#endif

uint32_t Raindrop_ImpulseToVolume(float impulse, uint32_t baseline)
{
    /* 避免除零 */
    if (baseline == 0)
        return 0;

    /* 归一化冲量比值 × 1000 */
    float ratio = impulse / (float)baseline;

    /* 最小有效比值：ratio ≥ 0.01 才计入 */
    if (ratio < 0.01f)
        return 0;

    /* 线性模型：V = scale × impulse / baseline */
    float volume = RAIN_IMPULSE_SCALE * impulse / (float)baseline;

    /* 换算到 0.01mm³ 单位（四舍五入）*/
    uint32_t volume_0p01mm3 = (uint32_t)(volume * 1000.0f + 0.5f);

    /* 上限保护：单事件最大体积不超过 5000 (即50mm³) */
    if (volume_0p01mm3 > 5000)
        volume_0p01mm3 = 5000;

    return volume_0p01mm3;
}
