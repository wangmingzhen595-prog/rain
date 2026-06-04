#ifndef __RAIN_AREA_BUFFER_H
#define __RAIN_AREA_BUFFER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RAIN_AREA_QUEUE_SIZE        16U
#define RAIN_AREA_FIRMWARE_VERSION  0x0100U
#define RAIN_AREA_PROTOCOL_VERSION  0x0002U

#define RAIN_AREA_SOURCE_PA0        0U
#define RAIN_AREA_SOURCE_PA1        1U

#define RAIN_AREA_FLAG_PA0_SAT      0x0001U
#define RAIN_AREA_FLAG_USE_PA1      0x0002U
#define RAIN_AREA_FLAG_SCALED_VALID 0x0004U
#define RAIN_AREA_FLAG_RAW_VALID    0x0008U

typedef struct
{
    uint16_t seq;
    uint16_t peak_adc;
    uint16_t baseline_adc;
    uint16_t pulse_width_samples;
    uint32_t area_adc_samples;
    uint32_t area_adc_us;
    uint16_t flags;
    uint16_t source_channel;
    uint16_t max_sat_count;
    uint16_t gain_x100;
    uint32_t raw_integral_adc_us;
    uint32_t scaled_integral_adc_us;
} RainAreaEvent_t;

void RainAreaBuffer_Init(void);
uint16_t RainAreaBuffer_NextSeq(void);
void RainAreaBuffer_UpdateLatest(const RainAreaEvent_t *evt);
uint8_t RainAreaBuffer_Push(const RainAreaEvent_t *evt);
void RainAreaBuffer_RecordOverflow(void);
uint8_t RainAreaBuffer_GetLatest(RainAreaEvent_t *evt);
uint8_t RainAreaBuffer_PeekOldest(RainAreaEvent_t *evt);
uint8_t RainAreaBuffer_PopOldest(void);
uint16_t RainAreaBuffer_GetCount(void);
uint32_t RainAreaBuffer_GetTotalSavedCount(void);
uint32_t RainAreaBuffer_GetOverflowCount(void);
void RainAreaBuffer_ClearStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __RAIN_AREA_BUFFER_H */
