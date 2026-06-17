#ifndef __RAIN_DISPLAY_H
#define __RAIN_DISPLAY_H

#include "stm32f10x.h"

/* ============================================================================
 * Rain monitor UI on the 320x240 ILI9341 (landscape).
 *
 * Layout
 *   title bar      y 0..21    : "RAIN MONITOR", intensity, gain badge
 *   waveform plot  y 26..171  : grid + baseline + threshold + trace + peak
 *   stats          y 176..239 : Volt / Imp(integral) / Drop / Vol / Total
 *
 * Waveform Y axis is fixed full scale 0..3.3V (ADC 0..4095) for now.
 * ========================================================================== */

void RainDisplay_Init(void);          /* LCD init + draw the static frame */
void RainDisplay_DrawStatic(void);    /* redraw title/box/grid/labels once */

/* Redraw the waveform region from an ADC snapshot.
 *   buf/len      : ADC samples (export_buffer, SNAPSHOT_SIZE pts ~21ms)
 *   baseline     : pre-trigger baseline (ADC code) -> gray line
 *   threshold    : trigger threshold (ADC code)    -> yellow dashed line
 *   peak_index   : index of the peak in buf        -> red marker
 *   pa1_takeover : 1 = clipped/PA1 event (trace drawn orange) */
void RainDisplay_DrawWaveform(const uint16_t *buf, uint16_t len,
                              uint16_t baseline, uint16_t threshold,
                              uint16_t peak_index, uint8_t pa1_takeover);

/* Refresh the numeric fields (units fixed: V, mV*us, mm3, mm3, mm/h).
 *   last_vol_0p01mm3 / total_0p01mm3 are in 0.01mm^3 internal units. */
void RainDisplay_UpdateStats(float volt, uint32_t integral_mv_us,
                             uint32_t drops, uint32_t last_vol_0p01mm3,
                             uint32_t total_0p01mm3, float intensity_mmh,
                             char gain);

/* Self-test: draw the static frame, a synthetic pulse and dummy values.
 * Used to verify wiring / driver / layout before wiring in real data. */
void RainDisplay_SelfTest(void);

#endif /* __RAIN_DISPLAY_H */
