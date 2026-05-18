# Main Pulse Window Sampling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Raise ADC sampling density within STM32F103C8T6 limits, then use the detected peak as an anchor to extract and analyze the complete main positive raindrop pulse.

**Architecture:** Keep the current DMA/AWD/diff-trigger snapshot pipeline. Change ADC sample time through one compile-time profile, derive all time windows from the real sample interval, and add a main-pulse feature extractor in `User/main.c` that produces peak delta, positive area, positive energy, width, rise time, and fall time. Existing peak-based rain accumulation stays compatible during the first code change, while new features are exported through debug/event energy for calibration.

**Tech Stack:** STM32F103C8T6, STM32F10x Standard Peripheral Library, ADC1 scan mode with DMA circular buffer, C89-style embedded C, Keil/uVision project.

---

## Scope And Constraints

- Target MCU: STM32F103C8T6, 72 MHz core, 20 KB SRAM.
- Current ADC clock: APB2 72 MHz / 6 = 12 MHz.
- Current ADC sample time: `ADC_SampleTime_239Cycles5`.
- Current PA0 sample interval: about `42us` per PA0 sample in two-channel scan mode.
- Recommended first sampling profile: `ADC_SampleTime_71Cycles5`, about `14us` per PA0 sample.
- Optional faster profile: `ADC_SampleTime_55Cycles5`, about `11.33us` per PA0 sample, only if the analog front end is buffered or verified on oscilloscope.
- Do not do heavy computation in the DMA ISR. The DMA ISR continues to store samples and start snapshots; main pulse analysis runs in `Process_Snapshot_IfReady()`.
- Keep `SNAPSHOT_SIZE = 500`. At `14us/sample`, the snapshot covers about `7ms`, enough for the main pulse and early ringing.
- Main pulse target from the oscilloscope photos:
  - Rise time: about `140.5us` to `181.5us`.
  - Fall time: about `651us` to `831us`.
  - Complete main positive pulse: about `0.8ms` to `1.1ms`.
- New analysis window:
  - Peak pre-window: `350us`.
  - Peak post-window: `1400us`.
  - At `14us/sample`, this is about `25 + 100` samples.
  - At `11.33us/sample`, this is about `31 + 124` samples.

## File Structure

- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\Hardware\AD.h`
  - Owns ADC sampling profile macros and sample interval constants.
- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\Hardware\AD.c`
  - Uses the configured ADC sample time macro for PA0 and PA1.
- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\User\main.c`
  - Removes the hard-coded `42.0f` sample interval.
  - Adds main pulse window macros, feature struct, and extraction helpers.
  - Replaces trigger-centered front-window analysis with peak-centered main-pulse analysis.
  - Emits/uses the new main-pulse features while preserving compatibility for current rain accumulation.
- No new C source file in this iteration.
  - Reason: the affected logic is tightly coupled to existing snapshot processing and static helpers in `main.c`. Splitting can wait until the feature is stable.

---

### Task 1: Centralize ADC Sampling Profile

**Files:**
- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\Hardware\AD.h`
- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\Hardware\AD.c`
- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\User\main.c`

- [ ] **Step 1: Add ADC profile macros to `Hardware/AD.h`**

Insert after `void AD_Init(void);`:

```c
/* ADC sampling profile
 * ADC clock = 72MHz / 6 = 12MHz.
 * Two-channel scan mode means PA0 receives one sample every two conversions.
 *
 * 239.5 cycles: 42.00us/PA0 sample, original conservative setting.
 *  71.5 cycles: 14.00us/PA0 sample, recommended first fast setting.
 *  55.5 cycles: 11.33us/PA0 sample, use after analog-front-end verification.
 */
#define RAIN_ADC_SAMPLE_TIME        ADC_SampleTime_71Cycles5
#define ADC_SAMPLE_INTERVAL_NS      14000UL
#define ADC_SAMPLE_INTERVAL_US      14.0f
```

- [ ] **Step 2: Use the profile in `Hardware/AD.c`**

Replace both channel configuration lines:

```c
ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
```

with:

```c
ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, RAIN_ADC_SAMPLE_TIME);
ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, RAIN_ADC_SAMPLE_TIME);
```

- [ ] **Step 3: Remove the duplicate sample interval macro from `User/main.c`**

Delete this line:

```c
#define ADC_SAMPLE_INTERVAL_US   42.0f
```

`User/main.c` already includes `AD.h`, so it will use the centralized value.

- [ ] **Step 4: Build in Keil**

Run the normal uVision build for `Project.uvprojx`.

Expected:

```text
0 Error(s)
```

- [ ] **Step 5: Commit**

```bash
git add Hardware/AD.h Hardware/AD.c User/main.c
git commit -m "feat: add fast adc sampling profile"
```

---

### Task 2: Add Time-Based Main Pulse Window Constants

**Files:**
- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\User\main.c`

- [ ] **Step 1: Add integer time-to-sample macros**

Insert near the existing front-analysis macros:

```c
#define US_TO_SAMPLES_CEIL(us) \
	((uint16_t)((((uint32_t)(us) * 1000UL) + ADC_SAMPLE_INTERVAL_NS - 1UL) / ADC_SAMPLE_INTERVAL_NS))

#define MAIN_PULSE_PRE_US          350U
#define MAIN_PULSE_POST_US         1400U
#define MAIN_PULSE_PRE_SAMPLES     US_TO_SAMPLES_CEIL(MAIN_PULSE_PRE_US)
#define MAIN_PULSE_POST_SAMPLES    US_TO_SAMPLES_CEIL(MAIN_PULSE_POST_US)
#define MAIN_PULSE_BASELINE_DELTA  8U
#define MAIN_PULSE_SETTLE_COUNT    3U
#define MAIN_PULSE_MIN_WIDTH       8U
```

- [ ] **Step 2: Keep old front-window macros temporarily**

Leave these existing macros in place for fallback and comparison:

```c
#define FRONT_ANALYSIS_TIME_MS  2.0f
#define FRONT_ANALYSIS_SAMPLES   ((uint16_t)(FRONT_ANALYSIS_TIME_MS * 1000.0f / ADC_SAMPLE_INTERVAL_US))
```

They will still compile with the centralized interval.

- [ ] **Step 3: Build in Keil**

Expected:

```text
0 Error(s)
```

- [ ] **Step 4: Commit**

```bash
git add User/main.c
git commit -m "feat: define time based main pulse window"
```

---

### Task 3: Add Main Pulse Feature Data Structure And Prototypes

**Files:**
- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\User\main.c`

- [ ] **Step 1: Add the feature struct**

Insert near the event type definitions:

```c
typedef struct
{
	uint8_t valid;
	uint16_t win_start;
	uint16_t win_end;
	uint16_t pulse_start;
	uint16_t pulse_end;
	uint16_t peak_index;
	uint16_t peak_value;
	uint16_t peak_delta;
	uint16_t rise_samples;
	uint16_t fall_samples;
	uint16_t width20_samples;
	uint16_t width50_samples;
	uint32_t area_pos;
	uint32_t energy_pos;
} MainPulseFeatures_t;
```

- [ ] **Step 2: Add helper prototypes**

Insert beside the existing static prototypes:

```c
static uint8_t Extract_MainPulse_Features(uint16_t *buf, uint16_t len,
                                          uint16_t anchor_peak_index,
                                          int32_t baseline,
                                          MainPulseFeatures_t *features);
static uint16_t Clamp_U16_From_I32(int32_t value);
```

- [ ] **Step 3: Build in Keil**

Expected:

```text
0 Error(s)
```

- [ ] **Step 4: Commit**

```bash
git add User/main.c
git commit -m "feat: add main pulse feature model"
```

---

### Task 4: Implement Main Pulse Feature Extraction

**Files:**
- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\User\main.c`

- [ ] **Step 1: Add `Clamp_U16_From_I32`**

Insert before `Find_Peak_In_Buffer()`:

```c
static uint16_t Clamp_U16_From_I32(int32_t value)
{
	if (value <= 0)
	{
		return 0;
	}
	if (value > 65535)
	{
		return 65535;
	}
	return (uint16_t)value;
}
```

- [ ] **Step 2: Add `Extract_MainPulse_Features`**

Insert after `Clamp_U16_From_I32()`:

```c
static uint8_t Extract_MainPulse_Features(uint16_t *buf, uint16_t len,
                                          uint16_t anchor_peak_index,
                                          int32_t baseline,
                                          MainPulseFeatures_t *features)
{
	uint16_t i;
	uint16_t settle;
	uint16_t threshold_near_base = Clamp_U16_From_I32(baseline + MAIN_PULSE_BASELINE_DELTA);

	if (features == 0 || buf == 0 || len == 0 || anchor_peak_index >= len)
	{
		return 0;
	}

	features->valid = 0;
	features->win_start = (anchor_peak_index > MAIN_PULSE_PRE_SAMPLES) ?
		(anchor_peak_index - MAIN_PULSE_PRE_SAMPLES) : 0;
	features->win_end = (anchor_peak_index + MAIN_PULSE_POST_SAMPLES < len) ?
		(anchor_peak_index + MAIN_PULSE_POST_SAMPLES) : (uint16_t)(len - 1);

	features->peak_index = anchor_peak_index;
	features->peak_value = buf[anchor_peak_index];
	for (i = features->win_start; i <= features->win_end; i++)
	{
		if (buf[i] > features->peak_value)
		{
			features->peak_value = buf[i];
			features->peak_index = i;
		}
	}

	features->peak_delta = Clamp_U16_From_I32((int32_t)features->peak_value - baseline);
	if (features->peak_delta < MIN_PEAK_DELTA_OVER_THR)
	{
		return 0;
	}

	features->pulse_start = features->win_start;
	settle = 0;
	i = features->peak_index;
	while (i > features->win_start)
	{
		if (buf[i] <= threshold_near_base)
		{
			if (settle < 255)
			{
				settle++;
			}
			if (settle >= MAIN_PULSE_SETTLE_COUNT)
			{
				features->pulse_start = (uint16_t)(i + MAIN_PULSE_SETTLE_COUNT);
				break;
			}
		}
		else
		{
			settle = 0;
		}
		i--;
	}
	if (features->pulse_start > features->peak_index)
	{
		features->pulse_start = features->win_start;
	}

	features->pulse_end = features->win_end;
	settle = 0;
	for (i = features->peak_index; i <= features->win_end; i++)
	{
		if (buf[i] <= threshold_near_base)
		{
			if (settle < 255)
			{
				settle++;
			}
			if (settle >= MAIN_PULSE_SETTLE_COUNT)
			{
				features->pulse_end = (uint16_t)(i - MAIN_PULSE_SETTLE_COUNT);
				break;
			}
		}
		else
		{
			settle = 0;
		}
	}
	if (features->pulse_end <= features->peak_index)
	{
		features->pulse_end = features->peak_index;
	}

	features->rise_samples = (features->peak_index > features->pulse_start) ?
		(uint16_t)(features->peak_index - features->pulse_start) : 1;
	features->fall_samples = (features->pulse_end > features->peak_index) ?
		(uint16_t)(features->pulse_end - features->peak_index) : 1;

	features->area_pos = 0;
	features->energy_pos = 0;
	features->width20_samples = 0;
	features->width50_samples = 0;

	uint16_t threshold20 = (uint16_t)(((uint32_t)features->peak_delta * 20U) / 100U);
	uint16_t threshold50 = (uint16_t)(((uint32_t)features->peak_delta * 50U) / 100U);

	for (i = features->pulse_start; i <= features->pulse_end; i++)
	{
		int32_t delta_i32 = (int32_t)buf[i] - baseline;
		uint16_t delta = Clamp_U16_From_I32(delta_i32);

		features->area_pos += delta;
		features->energy_pos += (uint32_t)delta * (uint32_t)delta;

		if (delta >= threshold20)
		{
			features->width20_samples++;
		}
		if (delta >= threshold50)
		{
			features->width50_samples++;
		}
	}

	if ((uint16_t)(features->pulse_end - features->pulse_start + 1) < MAIN_PULSE_MIN_WIDTH)
	{
		return 0;
	}

	features->valid = 1;
	return 1;
}
```

- [ ] **Step 3: Build in Keil**

Expected:

```text
0 Error(s)
```

- [ ] **Step 4: Commit**

```bash
git add User/main.c
git commit -m "feat: extract main pulse features"
```

---

### Task 5: Wire Peak-Centered Main Pulse Analysis Into Snapshot Processing

**Files:**
- Modify: `C:\Users\汪明珍\Desktop\从控\2026.2.4\User\main.c`

- [ ] **Step 1: Create features in `Process_Snapshot_IfReady()`**

After the first `Find_Peak_In_Buffer(...)` call that fills `high_peak_idx` and `high_peak_val`, add:

```c
	MainPulseFeatures_t pulse_features;
	uint8_t pulse_ok = Extract_MainPulse_Features((uint16_t *)snapshot_buffer_high,
	                                              len,
	                                              high_peak_idx,
	                                              baseline_high,
	                                              &pulse_features);
```

- [ ] **Step 2: Replace the trigger-centered front pulse boundaries when extraction succeeds**

After the existing `front_peak_index/front_peak_value` calculation block and before `snapshot_peak_value = front_peak_value;`, add:

```c
	if (pulse_ok)
	{
		start_index = pulse_features.pulse_start;
		end_index = pulse_features.pulse_end;
		front_peak_index = pulse_features.peak_index;
		front_peak_value = pulse_features.peak_value;
	}
```

- [ ] **Step 3: Add debug output for calibration**

After the existing switch debug lines that print `front_peak_value`, add:

```c
	if (pulse_ok)
	{
		USART1_SendString("Pulse: area=");
		USART1_SendUint32(pulse_features.area_pos);
		USART1_SendString(", energy=");
		USART1_SendUint32(pulse_features.energy_pos);
		USART1_SendString(", rise=");
		USART1_SendUint32(pulse_features.rise_samples);
		USART1_SendString(", fall=");
		USART1_SendUint32(pulse_features.fall_samples);
		USART1_SendString(", w20=");
		USART1_SendUint32(pulse_features.width20_samples);
		USART1_SendString(", w50=");
		USART1_SendUint32(pulse_features.width50_samples);
		USART1_SendString("\r\n");
	}
```

- [ ] **Step 4: Use main-pulse area as event energy when available**

Replace:

```c
uint32_t energy = (uint32_t)front_peak_value * (uint32_t)pulse_samples;
```

with:

```c
uint32_t energy = pulse_ok ? pulse_features.area_pos :
	((uint32_t)front_peak_value * (uint32_t)pulse_samples);
```

- [ ] **Step 5: Keep current volume conversion compatible**

Keep this assignment unchanged for now:

```c
adc_peak_for_volume = front_peak_value;
```

Reason: `Raindrop_VoltageToVolume_0p01mm3()` is calibrated for voltage peak, not area. The next field calibration should fit a new table or formula for `area_pos`.

- [ ] **Step 6: Build in Keil**

Expected:

```text
0 Error(s)
```

- [ ] **Step 7: Commit**

```bash
git add User/main.c
git commit -m "feat: use peak centered main pulse window"
```

---

### Task 6: Bench Verification On Oscilloscope-Like Signals

**Files:**
- Observe serial output from firmware.
- No source changes unless the measured numbers contradict expectations.

- [ ] **Step 1: Flash firmware**

Flash the built firmware to STM32F103C8T6.

Expected:

```text
Device programs successfully
```

- [ ] **Step 2: Test with the small needle/drop setup**

For drops similar to the two oscilloscope photos, check serial output.

Expected with `ADC_SAMPLE_INTERVAL_US = 14.0f`:

```text
Pulse: area=...
rise=10..14
fall=46..60
w20=...
w50=...
```

Acceptable range:

- Rise samples: `8..16`
- Fall samples: `40..70`
- Main pulse width: `50..90`
- `area_pos` changes smoothly between similar drops
- No negative rebound is included in `area_pos`

- [ ] **Step 3: Verify CPU load symptoms**

Observe:

- No watchdog reset.
- No missed OLED refresh caused by ADC ISR load.
- No serial output flooding during continuous rain tests.
- `snapshot_ready` still clears after processing.

- [ ] **Step 4: If amplitude drops after shortening ADC sample time**

Change only these two lines in `Hardware/AD.h`:

```c
#define RAIN_ADC_SAMPLE_TIME        ADC_SampleTime_239Cycles5
#define ADC_SAMPLE_INTERVAL_NS      42000UL
#define ADC_SAMPLE_INTERVAL_US      42.0f
```

Then rebuild and compare amplitude. If the short-sample version measures lower peak/area on the same oscilloscope input, the analog front end is too high impedance for the faster sample time and needs either `ADC_SampleTime_239Cycles5` or a buffer op amp.

- [ ] **Step 5: Commit verified tuning**

```bash
git add Hardware/AD.h User/main.c
git commit -m "chore: tune main pulse sampling parameters"
```

---

## Self-Review

- Spec coverage: The plan raises sampling density, preserves F103C8T6 performance by keeping heavy work out of the DMA ISR, and extracts a peak-centered window that covers the complete main positive pulse from the oscilloscope data.
- Placeholder scan: No unresolved placeholders remain.
- Type consistency: New macros use integer nanosecond sample intervals for compile-time sample counts; runtime timing still uses `ADC_SAMPLE_INTERVAL_US` as float where existing validation expects float seconds.
- Risk: `ADC_SampleTime_71Cycles5` can under-sample high-impedance analog sources. The plan includes a direct fallback to `239Cycles5` and an oscilloscope comparison step.
