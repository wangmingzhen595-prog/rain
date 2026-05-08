# Rain Main Peak Tracker Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an MCU-side main impact peak extractor that captures each raindrop's positive 0-peak-0 pulse and uses that peak as the primary analysis feature.

**Architecture:** Keep the existing ADC DMA ring buffer as the data source. Add a small `RainPeakTracker` state machine that tracks baseline, detects the positive pulse, updates the maximum until the signal returns to the zero band, then publishes one event. Keep `WaveletEnergy` available for diagnostics during transition, but prevent it from double-counting rainfall when the peak tracker is enabled.

**Tech Stack:** STM32F10x Standard Peripheral Library C, Keil uVision project files, existing ADC DMA ring buffer in `Hardware/AD.c`, existing rain volume conversion in `User/raindrop.c`.

---

## Confidence

Peak tracking is the correct primary method for the current oscilloscope waveform. In `D:/实验数据/DEFAULT0.csv`, the baseline is about `-3.35 mV`, noise standard deviation is about `1.36 mV`, and the positive main peak is about `117 mV` above baseline. A thresholded state machine that starts above the noise band, updates the maximum during the active pulse, and finalizes at the first stable return to the zero band will capture this main peak.

The method is reliable for the target signal shape: one dominant positive impact lobe followed by tail drift or ringing. The implementation must guard against noise, double hits, and ADC clipping with trigger confirmation, minimum peak threshold, pulse width limits, and a deadtime after each finalized event.

## File Structure

- Create `Hardware/RainPeakTracker.h`: public config, event struct, context struct, and API.
- Create `Hardware/RainPeakTracker.c`: baseline tracking, threshold state machine, event finalization, ring-buffer processing.
- Modify `User/main.c`: initialize tracker, process new ADC samples, handle peak events as the primary rain event path, add debug output fields.
- Modify `User/raindrop.h` and `User/raindrop.c`: add a peak-amplitude-to-volume wrapper that reuses the existing LUT path.
- Modify `Hardware/RegisterMap.c`: expose latest peak fields in Modbus input registers `0x0030` through `0x003B`.
- Modify `Project.uvprojx`: add `RainPeakTracker.c` and `RainPeakTracker.h` to the Hardware group.
- Create `tools/peak_tracker_csv_check.py`: offline parameter check using `D:/实验数据/DEFAULT0.csv`.

## Task 1: Offline CSV Reference Check

**Files:**
- Create: `tools/peak_tracker_csv_check.py`
- Read input: `D:/实验数据/DEFAULT0.csv`

- [ ] **Step 1: Add the CSV check script**

Create `tools/peak_tracker_csv_check.py` with this behavior:

```python
from pathlib import Path
import statistics

CSV = Path(r"D:/实验数据/DEFAULT0.csv")
SAMPLE_RATE_HZ = 4_000_000
DT_US = 1_000_000 / SAMPLE_RATE_HZ

def load_scope_csv(path):
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    return [float(x.strip()) for x in lines[6:] if x.strip()]

def main():
    y = load_scope_csv(CSV)
    baseline_samples = y[:5000]
    baseline = statistics.median(baseline_samples)
    noise = statistics.pstdev(baseline_samples)
    centered = [v - baseline for v in y]
    trigger = max(10.0, 5.0 * noise)
    zero_band = max(4.0, 3.0 * noise)

    peak_i = max(range(len(centered)), key=lambda i: centered[i])
    peak = centered[peak_i]

    start = peak_i
    while start > 0 and centered[start] > trigger:
        start -= 1

    end = peak_i
    stable = 0
    while end < len(centered) - 1:
        if abs(centered[end]) <= zero_band:
            stable += 1
            if stable >= 8:
                break
        else:
            stable = 0
        end += 1

    width_us = (end - start) * DT_US
    rise_us = (peak_i - start) * DT_US

    print(f"baseline_mV={baseline:.3f}")
    print(f"noise_mV={noise:.3f}")
    print(f"trigger_mV={trigger:.3f}")
    print(f"zero_band_mV={zero_band:.3f}")
    print(f"peak_mV={peak:.3f}")
    print(f"rise_us={rise_us:.2f}")
    print(f"width_us={width_us:.2f}")
    print(f"start={start} peak_index={peak_i} end={end}")

    assert 100.0 <= peak <= 140.0
    assert 300.0 <= rise_us <= 800.0
    assert 1500.0 <= width_us <= 3500.0

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the CSV check**

Run:

```powershell
python tools/peak_tracker_csv_check.py
```

Expected: prints a main peak near `117 mV`, rise time near `270 us`, and positive width around `2.3 ms`. The assertions pass.

## Task 2: Add the Peak Tracker API

**Files:**
- Create: `Hardware/RainPeakTracker.h`

- [ ] **Step 1: Define the public types and thresholds**

Create `Hardware/RainPeakTracker.h`:

```c
#ifndef __RAIN_PEAK_TRACKER_H
#define __RAIN_PEAK_TRACKER_H

#include "stm32f10x.h"
#include <stdint.h>

#define RPT_ENABLE_PRIMARY_RAIN_PATH      1
#define RPT_BASELINE_INIT_SAMPLES         64U
#define RPT_BASELINE_IIR_SHIFT            8U
#define RPT_TRIGGER_DELTA_ADC             16
#define RPT_ZERO_BAND_ADC                 5
#define RPT_MIN_PEAK_DELTA_ADC            30
#define RPT_TRIGGER_CONFIRM_SAMPLES       2U
#define RPT_ZERO_CONFIRM_SAMPLES          6U
#define RPT_MIN_WIDTH_US                  80U
#define RPT_MAX_WIDTH_US                  12000U
#define RPT_DEADTIME_US                   2500U

typedef enum {
    RPT_STATE_LEARNING = 0,
    RPT_STATE_IDLE,
    RPT_STATE_ACTIVE,
    RPT_STATE_DEADTIME
} RainPeakState_t;

typedef struct {
    uint8_t  valid;
    uint32_t seq;
    uint16_t baseline_adc;
    uint16_t peak_raw_adc;
    uint16_t peak_delta_adc;
    uint32_t start_sample;
    uint32_t peak_sample;
    uint32_t end_sample;
    uint32_t rise_samples;
    uint32_t width_samples;
    uint32_t area_adc_samples;
    uint32_t peak_mv;
    uint32_t rise_us;
    uint32_t width_us;
} RainPeakEvent_t;

typedef struct {
    RainPeakState_t state;
    uint32_t sample_rate_hz;
    uint32_t sample_index;
    uint32_t last_write_total;
    uint32_t event_seq;
    uint32_t baseline_acc;
    uint16_t baseline_adc;
    uint16_t baseline_sample_count;
    uint16_t trigger_confirm;
    uint16_t zero_confirm;
    uint32_t deadtime_remaining;
    uint32_t start_sample;
    uint32_t peak_sample;
    uint16_t peak_raw_adc;
    uint16_t peak_delta_adc;
    uint32_t area_adc_samples;
    RainPeakEvent_t latest_event;
    volatile uint8_t event_pending;
} RainPeakTrackerCtx_t;

void RainPeakTracker_Init(RainPeakTrackerCtx_t *ctx, uint32_t sample_rate_hz);
uint32_t RainPeakTracker_ProcessNewSamples(RainPeakTrackerCtx_t *ctx);
void RainPeakTracker_Update(RainPeakTrackerCtx_t *ctx, uint16_t raw_adc);
uint8_t RainPeakTracker_PopEvent(RainPeakTrackerCtx_t *ctx, RainPeakEvent_t *out_event);
uint32_t RainPeakTracker_AdcDeltaToMv(uint16_t adc_delta);
uint32_t RainPeakTracker_SamplesToUs(RainPeakTrackerCtx_t *ctx, uint32_t samples);

#endif
```

- [ ] **Step 2: Add the header to the Keil project**

Modify `Project.uvprojx` Hardware group after `AD.h` with:

```xml
<File>
  <FileName>RainPeakTracker.h</FileName>
  <FileType>5</FileType>
  <FilePath>.\Hardware\RainPeakTracker.h</FilePath>
</File>
```

Expected: Keil project shows `RainPeakTracker.h` under Hardware.

## Task 3: Implement the Peak Tracker State Machine

**Files:**
- Create: `Hardware/RainPeakTracker.c`

- [ ] **Step 1: Implement baseline, conversion, and event helpers**

Create `Hardware/RainPeakTracker.c` with:

```c
#include "RainPeakTracker.h"
#include "AD.h"

#define RPT_ADC_REF_MV 3300U
#define RPT_ADC_MAX    4095U

static uint16_t RPT_AbsDelta(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint32_t RPT_UsToSamples(RainPeakTrackerCtx_t *ctx, uint32_t us)
{
    uint32_t rate = ctx->sample_rate_hz;
    uint64_t samples;
    if (rate == 0)
        rate = 23810U;
    samples = ((uint64_t)rate * (uint64_t)us + 999999ULL) / 1000000ULL;
    if (samples < 1ULL)
        samples = 1ULL;
    if (samples > 0xFFFFFFFFULL)
        samples = 0xFFFFFFFFULL;
    return (uint32_t)samples;
}

uint32_t RainPeakTracker_SamplesToUs(RainPeakTrackerCtx_t *ctx, uint32_t samples)
{
    uint32_t rate = ctx->sample_rate_hz;
    if (rate == 0)
        rate = 23810U;
    return (uint32_t)(((uint64_t)samples * 1000000ULL + rate / 2U) / rate);
}

uint32_t RainPeakTracker_AdcDeltaToMv(uint16_t adc_delta)
{
    return (uint32_t)(((uint64_t)adc_delta * RPT_ADC_REF_MV + RPT_ADC_MAX / 2U) / RPT_ADC_MAX);
}
```

- [ ] **Step 2: Implement init and event pop**

Append:

```c
void RainPeakTracker_Init(RainPeakTrackerCtx_t *ctx, uint32_t sample_rate_hz)
{
    uint8_t *p = (uint8_t *)ctx;
    uint32_t i;
    for (i = 0; i < sizeof(RainPeakTrackerCtx_t); i++)
        p[i] = 0;
    ctx->state = RPT_STATE_LEARNING;
    ctx->sample_rate_hz = sample_rate_hz;
}

uint8_t RainPeakTracker_PopEvent(RainPeakTrackerCtx_t *ctx, RainPeakEvent_t *out_event)
{
    if (!ctx->event_pending)
        return 0;
    if (out_event != 0)
        *out_event = ctx->latest_event;
    ctx->event_pending = 0;
    return 1;
}
```

- [ ] **Step 3: Implement the per-sample state machine**

Append:

```c
void RainPeakTracker_Update(RainPeakTrackerCtx_t *ctx, uint16_t raw_adc)
{
    int32_t y;
    uint32_t min_width_samples;
    uint32_t max_width_samples;

    ctx->sample_index++;

    if (ctx->state == RPT_STATE_LEARNING)
    {
        ctx->baseline_acc += raw_adc;
        ctx->baseline_sample_count++;
        if (ctx->baseline_sample_count >= RPT_BASELINE_INIT_SAMPLES)
        {
            ctx->baseline_adc = (uint16_t)(ctx->baseline_acc / RPT_BASELINE_INIT_SAMPLES);
            ctx->state = RPT_STATE_IDLE;
        }
        return;
    }

    y = (int32_t)raw_adc - (int32_t)ctx->baseline_adc;

    if (ctx->state == RPT_STATE_DEADTIME)
    {
        if (ctx->deadtime_remaining > 0)
            ctx->deadtime_remaining--;
        else
            ctx->state = RPT_STATE_IDLE;
        return;
    }

    if (ctx->state == RPT_STATE_IDLE)
    {
        if (RPT_AbsDelta(raw_adc, ctx->baseline_adc) <= RPT_ZERO_BAND_ADC)
        {
            int32_t base = (int32_t)ctx->baseline_adc;
            base += ((int32_t)raw_adc - base) >> RPT_BASELINE_IIR_SHIFT;
            if (base < 0) base = 0;
            if (base > 4095) base = 4095;
            ctx->baseline_adc = (uint16_t)base;
        }

        if (y >= RPT_TRIGGER_DELTA_ADC)
        {
            ctx->trigger_confirm++;
            if (ctx->trigger_confirm >= RPT_TRIGGER_CONFIRM_SAMPLES)
            {
                ctx->state = RPT_STATE_ACTIVE;
                ctx->start_sample = ctx->sample_index - ctx->trigger_confirm + 1U;
                ctx->peak_sample = ctx->sample_index;
                ctx->peak_raw_adc = raw_adc;
                ctx->peak_delta_adc = (uint16_t)y;
                ctx->area_adc_samples = (uint32_t)y;
                ctx->zero_confirm = 0;
            }
        }
        else
        {
            ctx->trigger_confirm = 0;
        }
        return;
    }

    if (ctx->state == RPT_STATE_ACTIVE)
    {
        if (y > 0)
            ctx->area_adc_samples += (uint32_t)y;

        if (y > (int32_t)ctx->peak_delta_adc)
        {
            ctx->peak_delta_adc = (uint16_t)y;
            ctx->peak_raw_adc = raw_adc;
            ctx->peak_sample = ctx->sample_index;
        }

        if (ctx->sample_index > ctx->peak_sample && RPT_AbsDelta(raw_adc, ctx->baseline_adc) <= RPT_ZERO_BAND_ADC)
            ctx->zero_confirm++;
        else
            ctx->zero_confirm = 0;

        min_width_samples = RPT_UsToSamples(ctx, RPT_MIN_WIDTH_US);
        max_width_samples = RPT_UsToSamples(ctx, RPT_MAX_WIDTH_US);

        if (ctx->zero_confirm >= RPT_ZERO_CONFIRM_SAMPLES ||
            (ctx->sample_index - ctx->start_sample) >= max_width_samples)
        {
            uint32_t width = ctx->sample_index - ctx->start_sample + 1U;
            if (ctx->peak_delta_adc >= RPT_MIN_PEAK_DELTA_ADC && width >= min_width_samples && width <= max_width_samples)
            {
                RainPeakEvent_t ev;
                ev.valid = 1;
                ev.seq = ++ctx->event_seq;
                ev.baseline_adc = ctx->baseline_adc;
                ev.peak_raw_adc = ctx->peak_raw_adc;
                ev.peak_delta_adc = ctx->peak_delta_adc;
                ev.start_sample = ctx->start_sample;
                ev.peak_sample = ctx->peak_sample;
                ev.end_sample = ctx->sample_index;
                ev.rise_samples = ctx->peak_sample - ctx->start_sample;
                ev.width_samples = width;
                ev.area_adc_samples = ctx->area_adc_samples;
                ev.peak_mv = RainPeakTracker_AdcDeltaToMv(ctx->peak_delta_adc);
                ev.rise_us = RainPeakTracker_SamplesToUs(ctx, ev.rise_samples);
                ev.width_us = RainPeakTracker_SamplesToUs(ctx, ev.width_samples);
                ctx->latest_event = ev;
                ctx->event_pending = 1;
            }
            ctx->trigger_confirm = 0;
            ctx->zero_confirm = 0;
            ctx->deadtime_remaining = RPT_UsToSamples(ctx, RPT_DEADTIME_US);
            ctx->state = RPT_STATE_DEADTIME;
        }
    }
}
```

- [ ] **Step 4: Implement ring-buffer batch processing**

Append:

```c
uint32_t RainPeakTracker_ProcessNewSamples(RainPeakTrackerCtx_t *ctx)
{
    uint32_t current_total = ring_write_total_ch0;
    uint32_t available = current_total - ctx->last_write_total;
    uint32_t processed = 0;
    uint16_t start_idx;

    if (available == 0)
        return 0;
    if (available > RING_BUFFER_SIZE)
        available = RING_BUFFER_SIZE;

    start_idx = (uint16_t)((ring_write_index_ch0 + RING_BUFFER_SIZE - (uint16_t)available) % RING_BUFFER_SIZE);
    while (processed < available)
    {
        uint16_t idx = (uint16_t)((start_idx + (uint16_t)processed) % RING_BUFFER_SIZE);
        RainPeakTracker_Update(ctx, adc_ring_buffer_ch0[idx]);
        processed++;
    }

    ctx->last_write_total = current_total;
    return processed;
}
```

- [ ] **Step 5: Add the C file to the Keil project**

Modify `Project.uvprojx` Hardware group after `AD.c` with:

```xml
<File>
  <FileName>RainPeakTracker.c</FileName>
  <FileType>1</FileType>
  <FilePath>.\Hardware\RainPeakTracker.c</FilePath>
</File>
```

Expected: Keil project compiles `RainPeakTracker.c`.

## Task 4: Route Rain Analysis Through Main Peak Events

**Files:**
- Modify: `User/main.c`
- Modify: `User/raindrop.h`
- Modify: `User/raindrop.c`

- [ ] **Step 1: Add peak tracker globals and init**

In `User/main.c`, add:

```c
#include "RainPeakTracker.h"

RainPeakTrackerCtx_t g_peak_ctx;
static RainPeakEvent_t g_last_peak_event;
static uint32_t g_last_peak_volume_0p01mm3 = 0;
```

After `WaveletEnergy_InitWithSampleRate(&g_we_ctx, WE_LEARNING_PERIOD_SEC, actual_sample_rate);`, add:

```c
RainPeakTracker_Init(&g_peak_ctx, actual_sample_rate);
```

- [ ] **Step 2: Add a peak-amplitude volume wrapper**

In `User/raindrop.h`, add:

```c
uint32_t Raindrop_PeakMvToVolume_0p01mm3(uint32_t peak_mv, uint8_t *status_flag);
```

In `User/raindrop.c`, add:

```c
uint32_t Raindrop_PeakMvToVolume_0p01mm3(uint32_t peak_mv, uint8_t *status_flag)
{
    return Raindrop_VoltageToVolume_0p01mm3(peak_mv, status_flag);
}
```

- [ ] **Step 3: Handle peak events in the main loop**

In `User/main.c`, immediately after `WaveletEnergy_ProcessNewSamples(&g_we_ctx);`, add:

```c
RainPeakEvent_t peak_event;
RainPeakTracker_ProcessNewSamples(&g_peak_ctx);
if (RainPeakTracker_PopEvent(&g_peak_ctx, &peak_event))
{
    uint8_t peak_status = RAIN_STATUS_NOISE;
    uint32_t vol = Raindrop_PeakMvToVolume_0p01mm3(peak_event.peak_mv, &peak_status);
    g_last_peak_event = peak_event;
    g_last_peak_volume_0p01mm3 = vol;

    if (peak_status == RAIN_STATUS_OK && vol > 0)
    {
        Raindrop_AddVolume(vol);
        g_total_volume_0p01mm3 = Raindrop_GetTotalVolume_0p01mm3();
        if (sec_index < SECONDS_WINDOW)
            drops_per_second[sec_index]++;
        g_cal_detected_count++;
    }
}
```

- [ ] **Step 4: Prevent double-counting from WaveletEnergy**

In the existing `if (g_we_ctx.learning_done && g_we_ctx.rain_event_pending)` branch, wrap the volume accumulation part:

```c
#if !RPT_ENABLE_PRIMARY_RAIN_PATH
/* existing WaveletEnergy volume accumulation block remains here */
#endif
```

Keep WaveletEnergy counters and debug telemetry intact. Expected: one physical rain pulse adds volume once, through the peak tracker.

## Task 5: Expose Main Peak Results

**Files:**
- Modify: `User/main.c`
- Modify: `Hardware/RegisterMap.c`

- [ ] **Step 1: Add serial debug fields**

In `Send_Debug_Statistics()`, append:

```c
USART1_SendString("peak_mv=");
USART1_SendUint32(g_last_peak_event.peak_mv);
USART1_SendString(" peak_adc=");
USART1_SendUint32(g_last_peak_event.peak_delta_adc);
USART1_SendString(" rise_us=");
USART1_SendUint32(g_last_peak_event.rise_us);
USART1_SendString(" width_us=");
USART1_SendUint32(g_last_peak_event.width_us);
USART1_SendString(" area=");
USART1_SendUint32(g_last_peak_event.area_adc_samples);
USART1_SendString(" peak_vol=");
USART1_SendUint32(g_last_peak_volume_0p01mm3);
USART1_SendString("\r\n");
```

- [ ] **Step 2: Add Modbus input registers**

In `Hardware/RegisterMap.c`, include the tracker header and externs:

```c
#include "RainPeakTracker.h"
extern RainPeakTrackerCtx_t g_peak_ctx;
extern RainPeakEvent_t g_last_peak_event;
```

Add cases in `RegisterMap_ReadInput()`:

```c
case 0x0030: return (uint16_t)(g_last_peak_event.peak_mv & 0xFFFF);
case 0x0031: return g_last_peak_event.peak_delta_adc;
case 0x0032: return (uint16_t)(g_last_peak_event.rise_us >> 16);
case 0x0033: return (uint16_t)(g_last_peak_event.rise_us & 0xFFFF);
case 0x0034: return (uint16_t)(g_last_peak_event.width_us >> 16);
case 0x0035: return (uint16_t)(g_last_peak_event.width_us & 0xFFFF);
case 0x0036: return (uint16_t)(g_last_peak_event.area_adc_samples >> 16);
case 0x0037: return (uint16_t)(g_last_peak_event.area_adc_samples & 0xFFFF);
case 0x0038: return g_last_peak_event.baseline_adc;
case 0x0039: return g_last_peak_event.peak_raw_adc;
case 0x003A: return (uint16_t)(g_last_peak_event.seq >> 16);
case 0x003B: return (uint16_t)(g_last_peak_event.seq & 0xFFFF);
```

Expected: host software can read the latest main peak amplitude, rise time, width, area, baseline, and event sequence.

## Task 6: Verify on PC and MCU

**Files:**
- Read: `D:/实验数据/DEFAULT0.csv`
- Build: `Project.uvprojx`

- [ ] **Step 1: Run the offline CSV check**

Run:

```powershell
python tools/peak_tracker_csv_check.py
```

Expected: assertions pass and the extracted positive pulse matches the oscilloscope main peak.

- [ ] **Step 2: Build in Keil**

Open `Project.uvprojx` in Keil uVision and build Target 1.

Expected:
- `RainPeakTracker.c` compiles without warnings about missing prototypes.
- `main.c` resolves `g_peak_ctx`, `g_last_peak_event`, and `RainPeakTracker_*` symbols.
- Code size fits the existing target.

- [ ] **Step 3: Bench test with serial output**

Flash the MCU, send `HELP` then observe periodic debug output.

Expected after one raindrop:
- `peak_mv` is nonzero.
- `peak_adc` is above `RPT_MIN_PEAK_DELTA_ADC`.
- `rise_us` and `width_us` are stable and physically plausible.
- Rain count increments once per drop.
- Volume increments once per drop.

- [ ] **Step 4: Check double-count prevention**

Drop one raindrop and compare:

```text
before total_volume = X
after total_volume  = X + one peak-derived volume
```

Expected: no second increment appears from the WaveletEnergy branch.

## Self-Review

- Spec coverage: the plan captures the 0-peak-0 main pulse, uses peak tracking as the primary rain path, preserves existing diagnostics, and exposes results for later analysis.
- Placeholder scan: no implementation step relies on unnamed functions or deferred behavior.
- Type consistency: `RainPeakEvent_t` fields are defined in Task 2 and used consistently in Tasks 4 and 5.
- Scope check: this is one bounded change centered on main peak extraction; it does not redesign the ADC driver or the complete rain calibration model.
