#ifndef __RAIN_AREA_BUFFER_H
#define __RAIN_AREA_BUFFER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RAIN_AREA_QUEUE_SIZE        16U
#define RAIN_AREA_FIRMWARE_VERSION  0x0101U
#define RAIN_AREA_PROTOCOL_VERSION  0x0003U  /* v3：事件新增impulse_mv_us/volume_0p01mm3，新增累计寄存器 */

#define RAIN_AREA_SOURCE_PA0        0U
#define RAIN_AREA_SOURCE_PA1        1U

#define RAIN_AREA_FLAG_PA0_SAT      0x0001U
#define RAIN_AREA_FLAG_USE_PA1      0x0002U
#define RAIN_AREA_FLAG_SCALED_VALID 0x0004U
#define RAIN_AREA_FLAG_RAW_VALID    0x0008U
#define RAIN_AREA_FLAG_PULSE_FAIL   0x0010U  /* 主脉冲提取失败，面积/积分为前部窗口保守现算值 */
#define RAIN_AREA_FLAG_RELAXED      0x0020U  /* 严格形状验证未通过，由核心物理闸门（幅度/脉宽/持续时间）计入 */

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
    uint32_t impulse_mv_us;       /* 统一口径积分（mV·us），体积换算的输入 */
    uint32_t volume_0p01mm3;      /* 本滴体积（0.01mm³） */
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
