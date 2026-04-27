#include "RegisterMap.h"
#include "WaveletEnergy.h"

/* 复用原 Modbus_Slave.c 中的外部数据源 */
extern volatile uint32_t sampling_tick_counter;
extern volatile uint32_t watchdog_trigger_count;
extern volatile uint32_t g_total_volume_0p01mm3;

/* 小波能量上下文外部引用（定义在 main.c） */
extern WaveletEnergyCtx_t g_we_ctx;

/* 内部镜像：从站地址配置 */
static volatile uint8_t s_slave_addr = 1;

void RegisterMap_Init(uint8_t slave_addr_init)
{
    if (slave_addr_init == 0)
        slave_addr_init = 1;
    s_slave_addr = slave_addr_init;
}

uint8_t RegisterMap_GetSlaveAddr(void)
{
    return s_slave_addr;
}

void RegisterMap_SetSlaveAddr(uint8_t addr)
{
    if (addr == 0)
        return;
    s_slave_addr = addr;
}

uint16_t RegisterMap_ReadHolding(uint16_t addr, uint8_t *exception)
{
    if (exception)
        *exception = 0;

    switch (addr)
    {
        case 0x0000:
            return (uint16_t)s_slave_addr;
        default:
            if (exception)
                *exception = 0x02;
            return 0;
    }
}

uint8_t RegisterMap_WriteHolding(uint16_t addr, uint16_t value, uint8_t *exception)
{
    if (exception)
        *exception = 0;

    switch (addr)
    {
        case 0x0000:
            if (value != 0 && value <= 247)
            {
                s_slave_addr = (uint8_t)value;
                return 1;
            }
            else
            {
                if (exception)
                    *exception = 0x03;
                return 0;
            }
        default:
            if (exception)
                *exception = 0x02;
            return 0;
    }
}

uint16_t RegisterMap_ReadInput(uint16_t addr, uint8_t *exception)
{
    if (exception)
    {
        *exception = 0;
    }

    switch (addr)
    {
        /* ===== 小波能量模块寄存器（0x00~0x10） ===== */

        /* 学习进度 0~100% */
        case 0x0000:
            return WaveletEnergy_GetLearningProgress((WaveletEnergyCtx_t *)&g_we_ctx);

        /* 当前高频能量（uint32_t，高16位） */
        case 0x0001:
            return (uint16_t)(WaveletEnergy_GetCurrentEnergy((WaveletEnergyCtx_t *)&g_we_ctx) >> 16);
        /* 当前高频能量（uint32_t，低16位） */
        case 0x0002:
            return (uint16_t)(WaveletEnergy_GetCurrentEnergy((WaveletEnergyCtx_t *)&g_we_ctx) & 0xFFFF);

        /* 基线能量（uint32_t，高16位） */
        case 0x0003:
            return (uint16_t)(WaveletEnergy_GetBaselineEnergy((WaveletEnergyCtx_t *)&g_we_ctx) >> 16);
        /* 基线能量（uint32_t，低16位） */
        case 0x0004:
            return (uint16_t)(WaveletEnergy_GetBaselineEnergy((WaveletEnergyCtx_t *)&g_we_ctx) & 0xFFFF);

        /* 检测阈值（uint32_t，高16位） */
        case 0x0005:
            return (uint16_t)(WaveletEnergy_GetThreshold((WaveletEnergyCtx_t *)&g_we_ctx) >> 16);
        /* 检测阈值（uint32_t，低16位） */
        case 0x0006:
            return (uint16_t)(WaveletEnergy_GetThreshold((WaveletEnergyCtx_t *)&g_we_ctx) & 0xFFFF);

        /* 归一化能量比值 × 1000（uint32_t） */
        case 0x0007:
            return (uint16_t)(WaveletEnergy_GetNormRatio((WaveletEnergyCtx_t *)&g_we_ctx) >> 16);
        case 0x0008:
            return (uint16_t)(WaveletEnergy_GetNormRatio((WaveletEnergyCtx_t *)&g_we_ctx) & 0xFFFF);

        /* 高频比例（uint16_t，permille） */
        case 0x0009:
            return WaveletEnergy_GetHFRatio((WaveletEnergyCtx_t *)&g_we_ctx);

        /* 雨滴计数（uint32_t） */
        case 0x000A:
            return (uint16_t)(g_we_ctx.rain_count >> 16);
        case 0x000B:
            return (uint16_t)(g_we_ctx.rain_count & 0xFFFF);

        /* 振动计数（uint32_t） */
        case 0x000C:
            return (uint16_t)(g_we_ctx.vib_count >> 16);
        case 0x000D:
            return (uint16_t)(g_we_ctx.vib_count & 0xFFFF);

        /* 噪声计数（uint32_t） */
        case 0x000E:
            return (uint16_t)(g_we_ctx.noise_count >> 16);
        case 0x000F:
            return (uint16_t)(g_we_ctx.noise_count & 0xFFFF);

        /* 总事件数（uint32_t） */
        case 0x0010:
            return (uint16_t)(g_we_ctx.total_events >> 16);
        case 0x0011:
            return (uint16_t)(g_we_ctx.total_events & 0xFFFF);

        /* ===== 系统状态寄存器（0x20~0x2F） ===== */

        /* 累计体积（uint32_t，单位0.01mm³） */
        case 0x0020:
            return (uint16_t)(g_total_volume_0p01mm3 >> 16);
        case 0x0021:
            return (uint16_t)(g_total_volume_0p01mm3 & 0xFFFF);

        /* 采样计数（uint32_t） */
        case 0x0022:
            return (uint16_t)(sampling_tick_counter >> 16);
        case 0x0023:
            return (uint16_t)(sampling_tick_counter & 0xFFFF);

        /* 看门狗触发计数（uint32_t） */
        case 0x0024:
            return (uint16_t)(watchdog_trigger_count >> 16);
        case 0x0025:
            return (uint16_t)(watchdog_trigger_count & 0xFFFF);

        /* 最近一次批处理积压样本数 */
        case 0x0026:
            return g_we_ctx.last_available_samples;

        /* 单次批处理最大积压样本数 */
        case 0x0027:
            return g_we_ctx.max_available_samples;

        /* 当前降采样因子 */
        case 0x0028:
            return g_we_ctx.process_decimation;

        /* 环形缓冲溢出次数（uint32_t） */
        case 0x0029:
            return (uint16_t)(g_we_ctx.sample_overrun_count >> 16);
        case 0x002A:
            return (uint16_t)(g_we_ctx.sample_overrun_count & 0xFFFF);

        /* 估计丢弃样本数（uint32_t） */
        case 0x002B:
            return (uint16_t)(g_we_ctx.dropped_sample_count >> 16);
        case 0x002C:
            return (uint16_t)(g_we_ctx.dropped_sample_count & 0xFFFF);

        /* 已处理小波窗口数（uint32_t） */
        case 0x002D:
            return (uint16_t)(g_we_ctx.processed_window_count >> 16);
        case 0x002E:
            return (uint16_t)(g_we_ctx.processed_window_count & 0xFFFF);

        /* 当前死区剩余原始采样数 */
        case 0x002F:
            return (uint16_t)(g_we_ctx.deadtime_raw_remaining & 0xFFFF);

        default:
            if (exception)
            {
                *exception = 0x02;
            }
            return 0;
    }
}
