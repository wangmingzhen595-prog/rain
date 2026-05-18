#include "RegisterMap.h"

/* 复用原 Modbus_Slave.c 中的外部数据源 */
extern volatile uint32_t effective_drop_count;
extern volatile uint32_t raw_event_count;
extern volatile float    total_rain_mm;
extern float             voltage_sum;           /* 累计电压 */
extern uint16_t          current_peak;
extern float             current_voltage;
extern float             current_intensity_mmh;
extern volatile uint16_t dynamic_threshold;
extern char              last_gain_used;
extern volatile uint32_t sampling_tick_counter;
extern volatile uint32_t watchdog_trigger_count;
extern volatile uint32_t snapshot_valid_count;
extern volatile uint32_t cnt_rain_clean;
extern volatile uint32_t cnt_rain_fast;
extern volatile uint32_t cnt_vib;
extern volatile uint32_t cnt_emi;
extern volatile uint32_t cnt_bg;
extern volatile uint32_t cnt_bad;
/* 雨滴体积换算模块外部接口 */
extern uint32_t Raindrop_GetTotalVolume_0p01mm3(void);

/* 内部镜像：从站地址与阈值配置（与原 Modbus 保持语义一致） */
static volatile uint8_t  s_slave_addr   = 1;  /* Holding 0x0000 */
static volatile uint16_t s_threshold_cfg = 0; /* Holding 0x0001 */

void RegisterMap_Init(uint8_t slave_addr_init)
{
    if (slave_addr_init == 0)
    {
        slave_addr_init = 1;
    }
    s_slave_addr   = slave_addr_init;
    s_threshold_cfg = dynamic_threshold; /* 初始配置阈值取当前动态阈值 */
}

uint8_t RegisterMap_GetSlaveAddr(void)
{
    return s_slave_addr;
}

void RegisterMap_SetSlaveAddr(uint8_t addr)
{
    if (addr == 0)
    {
        return;
    }
    s_slave_addr = addr;
}

uint16_t RegisterMap_GetThresholdCfg(void)
{
    return s_threshold_cfg;
}

void RegisterMap_SetThresholdCfg(uint16_t value)
{
    s_threshold_cfg = value;
}

uint16_t RegisterMap_ReadHolding(uint16_t addr, uint8_t *exception)
{
    if (exception)
    {
        *exception = 0;
    }

    switch (addr)
    {
        case 0x0000:
            return (uint16_t)s_slave_addr;

        case 0x0001:
            return s_threshold_cfg;

        case 0x0002:
        case 0x0003:
        case 0x0004:
            /* 预留：暂不使用，返回0 */
            return 0;

        default:
            if (exception)
            {
                *exception = 0x02; /* Illegal Data Address */
            }
            return 0;
    }
}

uint8_t RegisterMap_WriteHolding(uint16_t addr, uint16_t value, uint8_t *exception)
{
    if (exception)
    {
        *exception = 0;
    }

    switch (addr)
    {
        case 0x0000:
            /* 从站地址修改：仅更新本地镜像，具体是否生效由上层决定 */
            if (value != 0 && value <= 247)
            {
                s_slave_addr = (uint8_t)value;
                return 1;
            }
            else
            {
                if (exception)
                {
                    *exception = 0x03; /* Illegal Data Value */
                }
                return 0;
            }

        case 0x0001:
            /* 阈值配置：只更新镜像，不直接改 dynamic_threshold，由上层决定是否应用 */
            s_threshold_cfg = value;
            return 1;

        default:
            if (exception)
            {
                *exception = 0x02; /* Illegal Data Address */
            }
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
        case 0x0000: return (uint16_t)((effective_drop_count >> 16) & 0xFFFF);
        case 0x0001: return (uint16_t)(effective_drop_count & 0xFFFF);

        case 0x0002: return (uint16_t)((raw_event_count >> 16) & 0xFFFF);
        case 0x0003: return (uint16_t)(raw_event_count & 0xFFFF);

        case 0x0004:
        case 0x0005:
        {
            union { float f; uint32_t u; } conv;
            conv.f = total_rain_mm;
            if (addr == 0x0004)
                return (uint16_t)((conv.u >> 16) & 0xFFFF);
            else
                return (uint16_t)(conv.u & 0xFFFF);
        }

        case 0x0006:
            return current_peak;

        case 0x0007:
        case 0x0008:
        {
            union { float f; uint32_t u; } conv;
            conv.f = current_voltage;
            if (addr == 0x0007)
                return (uint16_t)((conv.u >> 16) & 0xFFFF);
            else
                return (uint16_t)(conv.u & 0xFFFF);
        }

        case 0x0009:
        case 0x000A:
        {
            union { float f; uint32_t u; } conv;
            conv.f = current_intensity_mmh;
            if (addr == 0x0009)
                return (uint16_t)((conv.u >> 16) & 0xFFFF);
            else
                return (uint16_t)(conv.u & 0xFFFF);
        }

        case 0x000B:
            return dynamic_threshold;

        case 0x000C:
            return (last_gain_used == 'L') ? 1 : 0;

        case 0x000D: return (uint16_t)((sampling_tick_counter >> 16) & 0xFFFF);
        case 0x000E: return (uint16_t)(sampling_tick_counter & 0xFFFF);

        case 0x000F: return (uint16_t)((watchdog_trigger_count >> 16) & 0xFFFF);
        case 0x0010: return (uint16_t)(watchdog_trigger_count & 0xFFFF);

        case 0x0011: return (uint16_t)((snapshot_valid_count >> 16) & 0xFFFF);
        case 0x0012: return (uint16_t)(snapshot_valid_count & 0xFFFF);

        case 0x0013: return (uint16_t)((cnt_rain_clean >> 16) & 0xFFFF);
        case 0x0014: return (uint16_t)(cnt_rain_clean & 0xFFFF);

        case 0x0015: return (uint16_t)((cnt_rain_fast >> 16) & 0xFFFF);
        case 0x0016: return (uint16_t)(cnt_rain_fast & 0xFFFF);

        case 0x0017: return (uint16_t)((cnt_vib >> 16) & 0xFFFF);
        case 0x0018: return (uint16_t)(cnt_vib & 0xFFFF);

        case 0x0019: return (uint16_t)((cnt_emi >> 16) & 0xFFFF);
        case 0x001A: return (uint16_t)(cnt_emi & 0xFFFF);

        case 0x001B: return (uint16_t)((cnt_bg >> 16) & 0xFFFF);
        case 0x001C: return (uint16_t)(cnt_bg & 0xFFFF);

        case 0x001D: return (uint16_t)((cnt_bad >> 16) & 0xFFFF);
        case 0x001E: return (uint16_t)(cnt_bad & 0xFFFF);

        /* 累计电压 (float, 2个寄存器) */
        case 0x001F:
        case 0x0020:
        {
            union { float f; uint32_t u; } conv;
            conv.f = voltage_sum;
            if (addr == 0x001F)
                return (uint16_t)((conv.u >> 16) & 0xFFFF);
            else
                return (uint16_t)(conv.u & 0xFFFF);
        }

        /* 累计体积 (uint32_t, 单位0.01mm³, 2个寄存器) */
        case 0x0021:
        case 0x0022:
        {
            uint32_t volume_sum = Raindrop_GetTotalVolume_0p01mm3();
            if (addr == 0x0021)
                return (uint16_t)((volume_sum >> 16) & 0xFFFF);
            else
                return (uint16_t)(volume_sum & 0xFFFF);
        }

        default:
            if (exception)
            {
                *exception = 0x02; /* Illegal Data Address */
            }
            return 0;
    }
}

