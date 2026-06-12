#include "RainAreaBuffer.h"

#include "stm32f10x.h"

static RainAreaEvent_t s_queue[RAIN_AREA_QUEUE_SIZE];
static RainAreaEvent_t s_latest_event;
static volatile uint8_t s_head = 0;
static volatile uint8_t s_tail = 0;
static volatile uint8_t s_count = 0;
static volatile uint8_t s_latest_valid = 0;
static volatile uint16_t s_seq = 0;
static volatile uint32_t s_total_saved_count = 0;
static volatile uint32_t s_overflow_count = 0;

static void RainAreaBuffer_ClearEvent(RainAreaEvent_t *evt)
{
    evt->seq = 0;
    evt->peak_adc = 0;
    evt->baseline_adc = 0;
    evt->pulse_width_samples = 0;
    evt->area_adc_samples = 0;
    evt->area_adc_us = 0;
    evt->flags = 0;
    evt->source_channel = 0;
    evt->max_sat_count = 0;
    evt->gain_x100 = 0;
    evt->raw_integral_adc_us = 0;
    evt->scaled_integral_adc_us = 0;
    evt->impulse_mv_us = 0;
    evt->volume_0p01mm3 = 0;
}

static uint32_t RainAreaBuffer_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void RainAreaBuffer_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

void RainAreaBuffer_Init(void)
{
    uint32_t primask = RainAreaBuffer_EnterCritical();

    s_head = 0;
    s_tail = 0;
    s_count = 0;
    s_latest_valid = 0;
    s_seq = 0;
    s_total_saved_count = 0;
    s_overflow_count = 0;
    RainAreaBuffer_ClearEvent(&s_latest_event);

    RainAreaBuffer_ExitCritical(primask);
}

uint16_t RainAreaBuffer_NextSeq(void)
{
    uint16_t seq;
    uint32_t primask = RainAreaBuffer_EnterCritical();

    s_seq++;
    if (s_seq == 0U)
    {
        s_seq = 1U;
    }
    seq = s_seq;

    RainAreaBuffer_ExitCritical(primask);
    return seq;
}

void RainAreaBuffer_UpdateLatest(const RainAreaEvent_t *evt)
{
    uint32_t primask;

    if (evt == 0)
    {
        return;
    }

    primask = RainAreaBuffer_EnterCritical();
    s_latest_event = *evt;
    s_latest_valid = 1;
    RainAreaBuffer_ExitCritical(primask);
}

uint8_t RainAreaBuffer_Push(const RainAreaEvent_t *evt)
{
    uint32_t primask;

    if (evt == 0)
    {
        return 0;
    }

    primask = RainAreaBuffer_EnterCritical();
    if (s_count >= RAIN_AREA_QUEUE_SIZE)
    {
        RainAreaBuffer_ExitCritical(primask);
        return 0;
    }

    s_queue[s_tail] = *evt;
    s_tail = (uint8_t)((s_tail + 1U) % RAIN_AREA_QUEUE_SIZE);
    s_count++;
    s_total_saved_count++;

    RainAreaBuffer_ExitCritical(primask);
    return 1;
}

void RainAreaBuffer_RecordOverflow(void)
{
    uint32_t primask = RainAreaBuffer_EnterCritical();
    s_overflow_count++;
    RainAreaBuffer_ExitCritical(primask);
}

uint8_t RainAreaBuffer_GetLatest(RainAreaEvent_t *evt)
{
    uint8_t valid;
    uint32_t primask;

    if (evt == 0)
    {
        return 0;
    }

    primask = RainAreaBuffer_EnterCritical();
    valid = s_latest_valid;
    if (valid)
    {
        *evt = s_latest_event;
    }
    else
    {
        RainAreaBuffer_ClearEvent(evt);
    }
    RainAreaBuffer_ExitCritical(primask);

    return valid;
}

uint8_t RainAreaBuffer_PeekOldest(RainAreaEvent_t *evt)
{
    uint8_t valid;
    uint32_t primask;

    if (evt == 0)
    {
        return 0;
    }

    primask = RainAreaBuffer_EnterCritical();
    valid = (s_count > 0U);
    if (valid)
    {
        *evt = s_queue[s_head];
    }
    else
    {
        RainAreaBuffer_ClearEvent(evt);
    }
    RainAreaBuffer_ExitCritical(primask);

    return valid;
}

uint8_t RainAreaBuffer_PopOldest(void)
{
    uint32_t primask = RainAreaBuffer_EnterCritical();

    if (s_count == 0U)
    {
        RainAreaBuffer_ExitCritical(primask);
        return 0;
    }

    s_head = (uint8_t)((s_head + 1U) % RAIN_AREA_QUEUE_SIZE);
    s_count--;

    RainAreaBuffer_ExitCritical(primask);
    return 1;
}

uint16_t RainAreaBuffer_GetCount(void)
{
    uint16_t count;
    uint32_t primask = RainAreaBuffer_EnterCritical();
    count = s_count;
    RainAreaBuffer_ExitCritical(primask);
    return count;
}

uint32_t RainAreaBuffer_GetTotalSavedCount(void)
{
    uint32_t count;
    uint32_t primask = RainAreaBuffer_EnterCritical();
    count = s_total_saved_count;
    RainAreaBuffer_ExitCritical(primask);
    return count;
}

uint32_t RainAreaBuffer_GetOverflowCount(void)
{
    uint32_t count;
    uint32_t primask = RainAreaBuffer_EnterCritical();
    count = s_overflow_count;
    RainAreaBuffer_ExitCritical(primask);
    return count;
}

void RainAreaBuffer_ClearStats(void)
{
    uint32_t primask = RainAreaBuffer_EnterCritical();

    s_head = 0;
    s_tail = 0;
    s_count = 0;
    s_latest_valid = 0;
    s_seq = 0;
    s_total_saved_count = 0;
    s_overflow_count = 0;
    RainAreaBuffer_ClearEvent(&s_latest_event);

    RainAreaBuffer_ExitCritical(primask);
}
