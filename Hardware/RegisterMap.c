#include "RegisterMap.h"

#include "AD.h"
#include "RainAreaBuffer.h"

static volatile uint8_t  s_slave_addr = 1;   /* Holding 0x0000 */
static volatile uint16_t s_threshold_cfg = 0;/* Kept for legacy local access only. */

static uint16_t Read_U32_High(uint32_t value)
{
    return (uint16_t)((value >> 16) & 0xFFFFU);
}

static uint16_t Read_U32_Low(uint32_t value)
{
    return (uint16_t)(value & 0xFFFFU);
}

static uint16_t Read_Event_Field(const RainAreaEvent_t *evt, uint16_t base, uint16_t addr)
{
    switch ((uint16_t)(addr - base))
    {
        case 0x0000: return evt->seq;
        case 0x0001: return evt->peak_adc;
        case 0x0002: return evt->baseline_adc;
        case 0x0003: return evt->pulse_width_samples;
        case 0x0004: return Read_U32_High(evt->area_adc_us);
        case 0x0005: return Read_U32_Low(evt->area_adc_us);
        case 0x0006: return Read_U32_High(evt->area_adc_samples);
        case 0x0007: return Read_U32_Low(evt->area_adc_samples);
        case 0x0008: return evt->flags;
        case 0x0009: return evt->source_channel;
        case 0x000A: return evt->max_sat_count;
        case 0x000B: return evt->gain_x100;
        case 0x000C: return Read_U32_High(evt->raw_integral_adc_us);
        case 0x000D: return Read_U32_Low(evt->raw_integral_adc_us);
        case 0x000E: return Read_U32_High(evt->scaled_integral_adc_us);
        case 0x000F: return Read_U32_Low(evt->scaled_integral_adc_us);
        default:     return 0;
    }
}

void RegisterMap_Init(uint8_t slave_addr_init)
{
    if (slave_addr_init == 0)
    {
        slave_addr_init = 1;
    }
    s_slave_addr = slave_addr_init;
    s_threshold_cfg = 0;
}

uint8_t RegisterMap_GetSlaveAddr(void)
{
    return s_slave_addr;
}

void RegisterMap_SetSlaveAddr(uint8_t addr)
{
    if (addr == 0 || addr > 247U)
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
        case 0x0002:
        case 0x0003:
            return 0;

        default:
            if (exception)
            {
                *exception = 0x02;
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
            if (value != 0U && value <= 247U)
            {
                s_slave_addr = (uint8_t)value;
                return 1;
            }
            if (exception)
            {
                *exception = 0x03;
            }
            return 0;

        case 0x0001:
            return 1;

        case 0x0002:
            if (value == 1U)
            {
                (void)RainAreaBuffer_PopOldest();
                return 1;
            }
            if (value == 0U)
            {
                return 1;
            }
            if (exception)
            {
                *exception = 0x03;
            }
            return 0;

        case 0x0003:
            if (value == 0xA55AU)
            {
                RainAreaBuffer_ClearStats();
                return 1;
            }
            if (value == 0U)
            {
                return 1;
            }
            if (exception)
            {
                *exception = 0x03;
            }
            return 0;

        default:
            if (exception)
            {
                *exception = 0x02;
            }
            return 0;
    }
}

uint16_t RegisterMap_ReadInput(uint16_t addr, uint8_t *exception)
{
    RainAreaEvent_t evt;
    uint32_t value32;

    if (exception)
    {
        *exception = 0;
    }

    switch (addr)
    {
        case 0x0000:
            return (uint16_t)RAIN_AREA_FIRMWARE_VERSION;
        case 0x0001:
            return (uint16_t)RAIN_AREA_PROTOCOL_VERSION;
        case 0x0002:
            return (uint16_t)s_slave_addr;
        case 0x0003:
            return RainAreaBuffer_GetCount();
        case 0x0004:
            value32 = RainAreaBuffer_GetTotalSavedCount();
            return Read_U32_High(value32);
        case 0x0005:
            value32 = RainAreaBuffer_GetTotalSavedCount();
            return Read_U32_Low(value32);
        case 0x0006:
            value32 = RainAreaBuffer_GetOverflowCount();
            return Read_U32_High(value32);
        case 0x0007:
            value32 = RainAreaBuffer_GetOverflowCount();
            return Read_U32_Low(value32);
        case 0x0008:
            return (uint16_t)((ADC_SAMPLE_INTERVAL_NS + 500UL) / 1000UL);
        case 0x0009:
            return 0;

        case 0x0010:
        case 0x0011:
        case 0x0012:
        case 0x0013:
        case 0x0014:
        case 0x0015:
        case 0x0016:
        case 0x0017:
        case 0x0018:
        case 0x0019:
        case 0x001A:
        case 0x001B:
        case 0x001C:
        case 0x001D:
        case 0x001E:
        case 0x001F:
            (void)RainAreaBuffer_GetLatest(&evt);
            return Read_Event_Field(&evt, 0x0010U, addr);

        case 0x0020:
        case 0x0021:
        case 0x0022:
        case 0x0023:
        case 0x0024:
        case 0x0025:
        case 0x0026:
        case 0x0027:
        case 0x0028:
        case 0x0029:
        case 0x002A:
        case 0x002B:
        case 0x002C:
        case 0x002D:
        case 0x002E:
        case 0x002F:
            (void)RainAreaBuffer_PeekOldest(&evt);
            return Read_Event_Field(&evt, 0x0020U, addr);

        default:
            if (exception)
            {
                *exception = 0x02;
            }
            return 0;
    }
}
