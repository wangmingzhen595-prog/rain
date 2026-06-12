#include "raindrop.h"

/* ===================== 标定LUT（积分值mV·us → 体积0.01mm³） =====================
 * 2026-06 三针头标定（0.50 / 1.20 / 2.00 mm）：
 *   integral_value, volume_mm3
 *   572954,  10.416
 *   1367650, 30.075
 *   2226520, 38.461
 * 注意：三段斜率并不完全线性，1.20→2.00mm 区间斜率明显下降，可能与大滴
 * 二次脉冲、溅射、振铃尾巴或主脉冲窗口包含更多能量有关；当前按分段线性
 * 查表执行，后续补测约1.6mm针头验证第三段后再修表。 */

#define RAIN_LUT_SIZE  3

/* 积分值LUT（mV·us），严格单调递增 */
static const uint32_t integral_tab[RAIN_LUT_SIZE] = {
    572954,
    1367650,
    2226520
};

/* 体积LUT（0.01mm³），与integral_tab逐点对应 */
static const uint32_t volume_tab_0p01mm3[RAIN_LUT_SIZE] = {
    1042,
    3008,
    3846
};

/* ===================== 模块状态 ===================== */
static uint32_t g_last_volume_0p01mm3 = 0;   /* 最近一滴体积 */
static uint32_t g_total_volume_0p01mm3 = 0;  /* 累计体积，单调递增 */

/* ===================== 公共函数实现 ===================== */

void Raindrop_Init(void)
{
    g_last_volume_0p01mm3 = 0;
    g_total_volume_0p01mm3 = 0;
}

uint32_t Raindrop_IntegralToVolume_0p01mm3(uint32_t integral)
{
    uint8_t k;

    if (integral == 0U)
    {
        return 0;
    }

    /* 低于第一个标定点：按 (0,0)→(integral_tab[0],volume_tab[0]) 线性换算 */
    if (integral < integral_tab[0])
    {
        uint64_t num = (uint64_t)integral * volume_tab_0p01mm3[0];
        uint32_t den = integral_tab[0];
        return (uint32_t)((num + den / 2U) / den);
    }

    /* 高于最大标定点：暂时上夹紧（待补测大滴段后扩表） */
    if (integral >= integral_tab[RAIN_LUT_SIZE - 1])
    {
        return volume_tab_0p01mm3[RAIN_LUT_SIZE - 1];
    }

    /* 标定点范围内：分段线性插值，乘法用uint64_t防溢出，除法四舍五入 */
    for (k = 0; k < RAIN_LUT_SIZE - 1; k++)
    {
        if (integral < integral_tab[k + 1])
        {
            uint32_t dx = integral_tab[k + 1] - integral_tab[k];
            uint32_t dy = volume_tab_0p01mm3[k + 1] - volume_tab_0p01mm3[k];
            uint32_t offset = integral - integral_tab[k];
            uint64_t num = (uint64_t)offset * dy;
            return volume_tab_0p01mm3[k] + (uint32_t)((num + dx / 2U) / dx);
        }
    }

    /* 不可达（上方已覆盖全部区间），防御性兜底 */
    return volume_tab_0p01mm3[RAIN_LUT_SIZE - 1];
}

void Raindrop_CommitByIntegral(uint32_t integral)
{
    uint32_t volume = Raindrop_IntegralToVolume_0p01mm3(integral);

    g_last_volume_0p01mm3 = volume;
    g_total_volume_0p01mm3 += volume;
}

uint32_t Raindrop_GetLastVolume_0p01mm3(void)
{
    return g_last_volume_0p01mm3;
}

uint32_t Raindrop_GetTotalVolume_0p01mm3(void)
{
    return g_total_volume_0p01mm3;
}

void Raindrop_ResetTotalVolume(void)
{
    g_total_volume_0p01mm3 = 0;
}
