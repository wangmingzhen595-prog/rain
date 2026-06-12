#ifndef __RAINDROP_H
#define __RAINDROP_H

#include <stdint.h>

/* =====================================================================
 * 雨滴体积换算模块（积分值标定方案）
 *
 * 输入：主脉冲电压时间积分值 integral（mV·us，由主流程算好传入，
 *       本模块不做任何 ADC/采样/积分计算）。
 * 输出：体积统一使用 0.01mm³ 的 uint32_t 整数单位，全程无 float。
 *
 * 模块只负责：积分值查表转体积、保存最近一滴体积、累加累计体积。
 * ===================================================================== */

/**
 * @brief  初始化（清零最近一滴体积与累计体积）
 */
void Raindrop_Init(void);

/**
 * @brief  积分值转体积（分段线性查表+插值，定点整数）
 * @param  integral: 主脉冲积分值（mV·us）
 * @retval 体积（单位：0.01mm³）
 */
uint32_t Raindrop_IntegralToVolume_0p01mm3(uint32_t integral);

/**
 * @brief  提交一滴有效雨滴：换算体积、保存最近值并累加累计值
 * @param  integral: 本滴主脉冲积分值（mV·us）
 * @note   每个有效雨滴只能调用一次（调用点在事件确认链路，不在显示刷新）
 */
void Raindrop_CommitByIntegral(uint32_t integral);

/**
 * @brief  读取最近一滴体积（单位：0.01mm³）
 */
uint32_t Raindrop_GetLastVolume_0p01mm3(void);

/**
 * @brief  读取累计体积（单位：0.01mm³）
 * @note   uint32_t 上限约 42,949,672.95mm³（约43L），当前阶段够用
 */
uint32_t Raindrop_GetTotalVolume_0p01mm3(void);

/**
 * @brief  累计体积清零（定期清零或人工清零用）
 */
void Raindrop_ResetTotalVolume(void);

#endif /* __RAINDROP_H */
