const fs = require("fs");

const CSV = "D:/实验数据/DEFAULT0.csv";
const CSV_CANDIDATES = [
  "D:/实验数据/DEFAULT0.csv",
  CSV,
];
const SCOPE_RATE_HZ = 4_000_000;
const MCU_RATE_HZ = 23810;
const PHASE_STEP = Math.round(SCOPE_RATE_HZ / MCU_RATE_HZ);

const ADC_REF_MV = 3300;
const ADC_MAX = 4095;
const ADC_CAL_NUM = 9155;
const ADC_CAL_DEN = 10000;
const IMPULSE_CAL_NUM = 1;
const IMPULSE_CAL_DEN = 1;

const TRIGGER_MV = 100;
const END_MV = 80;
const MIN_PEAK_MV = 300;
const BASELINE_IDLE_BAND_MV = 20;
const SATURATION_ADC = 4000;
const TRIGGER_CONFIRM = 2;
const END_CONFIRM = 3;
const MIN_WIDTH_US = 80;
const MAX_WIDTH_US = 12000;
const DEADTIME_US = 2500;
const KIND_SINGLE = "single";
const KIND_SPLIT = "split";
const KIND_COMPOSITE = "composite";
const QUALITY_OVERLAP = 0x02;
const QUALITY_TRUNCATED = 0x04;
const EVENT_BUFFER_SIZE = 384;
const SPLIT_MIN_GAP_US = 200;
const SPLIT_VALLEY_PERCENT = 45;

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function adcDeltaToMv(adc) {
  return Math.round((adc * ADC_REF_MV * ADC_CAL_NUM) / (ADC_MAX * ADC_CAL_DEN));
}

function areaAdcSamplesToMvUs(areaAdcSamples, rate) {
  const mvSamples = Math.round((areaAdcSamples * ADC_REF_MV * ADC_CAL_NUM) / (ADC_MAX * ADC_CAL_DEN));
  return Math.round((mvSamples * 1_000_000) / rate);
}

function calibrateImpulse(areaMvUs) {
  return Math.round((areaMvUs * IMPULSE_CAL_NUM) / IMPULSE_CAL_DEN);
}

function mvToAdcDelta(mv) {
  const adc = Math.ceil((mv * ADC_MAX * ADC_CAL_DEN) / (ADC_REF_MV * ADC_CAL_NUM));
  return Math.max(0, Math.min(ADC_MAX, adc));
}

function signedMvToAdc(mv) {
  return mv >= 0 ? mvToAdcDelta(mv) : -mvToAdcDelta(-mv);
}

function usToSamples(rate, us) {
  return Math.max(1, Math.ceil((rate * us) / 1_000_000));
}

function samplesToUs(rate, samples) {
  return Math.round((samples * 1_000_000) / rate);
}

function loadCsv(path) {
  return fs.readFileSync(path, "utf8")
    .split(/\r?\n/)
    .slice(6)
    .filter(Boolean)
    .map(Number);
}

function resolveCsvPath() {
  const found = CSV_CANDIDATES.find((path) => fs.existsSync(path));
  if (!found) {
    throw new Error(`CSV not found. Tried: ${CSV_CANDIDATES.join(", ")}`);
  }
  return found;
}

function median(values) {
  const copy = [...values].sort((a, b) => a - b);
  const mid = Math.floor(copy.length / 2);
  return copy.length % 2 ? copy[mid] : (copy[mid - 1] + copy[mid]) / 2;
}

function makeAdcWaveform(scopeMv, targetPeakMv, baselineAdc) {
  const baseMv = median(scopeMv.slice(0, 5000));
  const centered = scopeMv.map((v) => v - baseMv);
  const originalPeak = Math.max(...centered);
  const scale = targetPeakMv / originalPeak;

  return centered.map((mv) => {
    const raw = baselineAdc + signedMvToAdc(mv * scale);
    return Math.max(0, Math.min(ADC_MAX, raw));
  });
}

function createTracker(rate) {
  return {
    state: "learning",
    sampleRate: rate,
    sampleIndex: 0,
    baselineAcc: 0,
    baselineAdc: 0,
    baselineSampleCount: 0,
    triggerConfirm: 0,
    endConfirm: 0,
    deadtimeRemaining: 0,
    startSample: 0,
    peakSample: 0,
    peakRawAdc: 0,
    peakDeltaAdc: 0,
    areaAdcSamples: 0,
    triggerSamples: [],
    eventBuffer: [],
    eventTruncated: false,
    eventSaturated: 0,
    triggerDeltaAdc: mvToAdcDelta(TRIGGER_MV),
    endDeltaAdc: mvToAdcDelta(END_MV),
    minPeakDeltaAdc: mvToAdcDelta(MIN_PEAK_MV),
    baselineIdleBandAdc: mvToAdcDelta(BASELINE_IDLE_BAND_MV),
    saturationAdc: SATURATION_ADC,
    triggerCount: 0,
    publishedCount: 0,
    rejectCount: 0,
    saturatedCount: 0,
    maxSeenDeltaAdc: 0,
    lastRejectReason: "none",
    events: [],
  };
}

function makeEvent(t, endReason, startIdx, endIdx, kind, qualityFlags, subIndex, subCount) {
  let area = 0;
  let peak = 0;
  let peakIdx = startIdx;
  for (let i = startIdx; i <= endIdx; i += 1) {
    const y = t.eventBuffer[i] ?? 0;
    if (y > 0) area += y;
    if (y > peak) {
      peak = y;
      peakIdx = i;
    }
  }
  const width = endIdx - startIdx + 1;
  const areaMvUs = areaAdcSamplesToMvUs(area, t.sampleRate);
  const event = {
    valid: true,
    saturated: t.eventSaturated,
    endReason,
    rejectReason: "none",
    kind,
    qualityFlags: qualityFlags | (t.eventTruncated ? QUALITY_TRUNCATED : 0),
    subIndex,
    subCount,
    peakAdc: peak,
    peakMv: adcDeltaToMv(peak),
    riseUs: samplesToUs(t.sampleRate, peakIdx - startIdx),
    widthUs: samplesToUs(t.sampleRate, width),
    area,
    areaMvUs,
    impulseCalibrated: calibrateImpulse(areaMvUs),
  };
  return event;
}

function findPeaks(t) {
  const minGap = usToSamples(t.sampleRate, SPLIT_MIN_GAP_US);
  const peaks = [];
  for (let i = 1; i < t.eventBuffer.length - 1; i += 1) {
    const y = t.eventBuffer[i];
    if (y >= t.minPeakDeltaAdc && y >= t.eventBuffer[i - 1] && y > t.eventBuffer[i + 1]) {
      const last = peaks[peaks.length - 1];
      if (last && i - last.idx < minGap) {
        if (y > last.value) {
          last.idx = i;
          last.value = y;
        }
      } else {
        peaks.push({ idx: i, value: y });
      }
    }
  }
  return peaks;
}

function valleyBetween(buffer, a, b) {
  let idx = a;
  let value = Infinity;
  for (let i = a; i <= b; i += 1) {
    if (buffer[i] < value) {
      value = buffer[i];
      idx = i;
    }
  }
  return { idx, value };
}

function finalize(t, endReason, width) {
  const minWidth = usToSamples(t.sampleRate, MIN_WIDTH_US);

  if (endReason === "timeout") {
    t.rejectCount += 1;
    t.lastRejectReason = "timeout";
  } else if (t.peakDeltaAdc < t.minPeakDeltaAdc) {
    t.rejectCount += 1;
    t.lastRejectReason = "peak_too_small";
  } else if (width < minWidth) {
    t.rejectCount += 1;
    t.lastRejectReason = "width_too_short";
  } else {
    const peaks = findPeaks(t);
    const minGap = usToSamples(t.sampleRate, SPLIT_MIN_GAP_US);
    const splitPoints = [];
    let canSplit = peaks.length >= 2;

    for (let i = 0; i < peaks.length - 1 && canSplit; i += 1) {
      const left = peaks[i];
      const right = peaks[i + 1];
      const valley = valleyBetween(t.eventBuffer, left.idx, right.idx);
      const limit = Math.min(left.value, right.value) * SPLIT_VALLEY_PERCENT / 100;
      if (right.idx - left.idx >= minGap && valley.value <= limit) {
        splitPoints.push(valley.idx);
      } else {
        canSplit = false;
      }
    }

    const out = [];
    if (canSplit && splitPoints.length > 0) {
      let segStart = 0;
      for (let i = 0; i <= splitPoints.length; i += 1) {
        const segEnd = i < splitPoints.length ? splitPoints[i] : t.eventBuffer.length - 1;
        out.push(makeEvent(t, endReason, segStart, segEnd, KIND_SPLIT, 0, i + 1, splitPoints.length + 1));
        segStart = segEnd + 1;
      }
    } else {
      const kind = peaks.length >= 2 ? KIND_COMPOSITE : KIND_SINGLE;
      const quality = peaks.length >= 2 ? QUALITY_OVERLAP : 0;
      out.push(makeEvent(t, endReason, 0, t.eventBuffer.length - 1, kind, quality, 1, 1));
    }

    for (const event of out) {
      t.events.push(event);
      t.publishedCount += 1;
      if (event.saturated) t.saturatedCount += 1;
    }
  }
}

function resetActive(t) {
  t.triggerConfirm = 0;
  t.triggerSamples = [];
  t.endConfirm = 0;
  t.eventSaturated = 0;
  t.startSample = 0;
  t.peakSample = 0;
  t.peakRawAdc = 0;
  t.peakDeltaAdc = 0;
  t.areaAdcSamples = 0;
  t.eventBuffer = [];
  t.eventTruncated = false;
}

function appendEventSample(t, y) {
  if (t.eventBuffer.length < EVENT_BUFFER_SIZE) t.eventBuffer.push(y);
  else t.eventTruncated = true;
}

function beginActive(t) {
  t.triggerCount += 1;
  t.state = "active";
  t.startSample = t.triggerSamples[0].sampleIndex;
  t.peakSample = t.startSample;
  t.peakRawAdc = t.triggerSamples[0].raw;
  t.peakDeltaAdc = 0;
  t.areaAdcSamples = 0;
  t.eventBuffer = [];
  t.eventTruncated = false;
  t.endConfirm = 0;
  t.eventSaturated = 0;

  for (const sample of t.triggerSamples) {
    appendEventSample(t, sample.y);
    if (sample.raw >= t.saturationAdc) t.eventSaturated = 1;
    if (sample.y > 0) t.areaAdcSamples += sample.y;
    if (sample.y > t.peakDeltaAdc) {
      t.peakDeltaAdc = sample.y;
      t.peakRawAdc = sample.raw;
      t.peakSample = sample.sampleIndex;
      t.maxSeenDeltaAdc = Math.max(t.maxSeenDeltaAdc, t.peakDeltaAdc);
    }
  }
  t.triggerSamples = [];
  t.triggerConfirm = 0;
}

function update(t, raw) {
  t.sampleIndex += 1;

  if (t.state === "learning") {
    t.baselineAcc += raw;
    t.baselineSampleCount += 1;
    if (t.baselineSampleCount >= 64) {
      t.baselineAdc = Math.round(t.baselineAcc / 64);
      t.state = "idle";
    }
    return;
  }

  const y = raw - t.baselineAdc;

  if (t.state === "deadtime") {
    if (t.deadtimeRemaining > 0) t.deadtimeRemaining -= 1;
    else if (y <= t.endDeltaAdc) t.state = "idle";
    return;
  }

  if (t.state === "idle") {
    if (Math.abs(raw - t.baselineAdc) <= t.baselineIdleBandAdc) {
      t.baselineAdc += (raw - t.baselineAdc) >> 8;
    }

    if (y >= t.triggerDeltaAdc) {
      t.triggerSamples.push({ raw, y, sampleIndex: t.sampleIndex });
      t.triggerConfirm = t.triggerSamples.length;
      if (t.triggerConfirm >= TRIGGER_CONFIRM) {
        beginActive(t);
      }
    } else {
      t.triggerConfirm = 0;
      t.triggerSamples = [];
    }
    return;
  }

  if (t.state === "active") {
    appendEventSample(t, y);
    if (raw >= t.saturationAdc) t.eventSaturated = 1;
    if (y > 0) t.areaAdcSamples += y;
    if (y > t.peakDeltaAdc) {
      t.peakDeltaAdc = y;
      t.peakRawAdc = raw;
      t.peakSample = t.sampleIndex;
      t.maxSeenDeltaAdc = Math.max(t.maxSeenDeltaAdc, t.peakDeltaAdc);
    }

    if (t.sampleIndex > t.peakSample && y <= t.endDeltaAdc) t.endConfirm += 1;
    else t.endConfirm = 0;

    const width = t.sampleIndex - t.startSample + 1;
    const maxWidth = usToSamples(t.sampleRate, MAX_WIDTH_US);
    let endReason = null;
    if (t.endConfirm >= END_CONFIRM) endReason = "positive_decay";
    else if (width >= maxWidth) endReason = "timeout";

    if (endReason) {
      finalize(t, endReason, width);
      resetActive(t);
      t.deadtimeRemaining = usToSamples(t.sampleRate, DEADTIME_US);
      t.state = "deadtime";
    }
  }
}

function replayPhase(adcWaveform, phase) {
  const t = createTracker(MCU_RATE_HZ);
  for (let i = phase; i < adcWaveform.length; i += PHASE_STEP) {
    update(t, adcWaveform[i]);
  }
  return t;
}

function rawFromDeltaMv(baselineAdc, mv) {
  return Math.max(0, Math.min(ADC_MAX, baselineAdc + signedMvToAdc(mv)));
}

function runSynthetic(samplesMv, baselineAdc = 300) {
  const t = createTracker(MCU_RATE_HZ);
  for (let i = 0; i < 80; i += 1) update(t, baselineAdc);
  for (const mv of samplesMv) update(t, rawFromDeltaMv(baselineAdc, mv));
  for (let i = 0; i < 24; i += 1) update(t, baselineAdc);
  return t.events.filter((ev) => ev.valid);
}

function runSyntheticRegressionTests() {
  const single = runSynthetic([0, 140, 360, 650, 420, 180, 70, 40, 30]);
  assert(single.length === 1, `single pulse should publish 1 event, got ${single.length}`);
  assert(single[0].kind === KIND_SINGLE, `single pulse should be kind=single, got ${single[0].kind}`);
  assert(single[0].areaMvUs > 0, "single pulse should have positive areaMvUs");

  const split = runSynthetic([0, 150, 520, 880, 420, 70, 60, 410, 760, 430, 120, 50, 40]);
  assert(split.length === 2, `separable near-simultaneous pulses should publish 2 split events, got ${split.length}`);
  assert(split.every((ev) => ev.kind === KIND_SPLIT && ev.subCount === 2), "split pulses should be marked as split sub-events");
  assert(split[0].subIndex === 1 && split[1].subIndex === 2, "split subIndex should be 1 then 2");

  const composite = runSynthetic([0, 160, 650, 900, 520, 520, 520, 520, 760, 620, 260, 90, 50, 40]);
  assert(composite.length === 1, `overlapped pulses should publish 1 composite event, got ${composite.length}`);
  assert(composite[0].kind === KIND_COMPOSITE, `overlapped pulses should be kind=composite, got ${composite[0].kind}`);
  assert((composite[0].qualityFlags & QUALITY_OVERLAP) !== 0, "composite event should carry overlap quality flag");

  const saturated = runSynthetic([0, 200, 3200, 3400, 3100, 400, 90, 40]);
  assert(saturated.length === 1, `saturated pulse should publish 1 event, got ${saturated.length}`);
  assert(saturated[0].saturated, "saturated pulse should set saturated flag");

  const stuckHigh = runSynthetic(Array(400).fill(500));
  assert(stuckHigh.length === 0, `stuck-high timeout should be rejected, got ${stuckHigh.length} events`);
}

function run(targetPeakMv, baselineAdc) {
  const scopeMv = loadCsv(resolveCsvPath());
  const adcWaveform = makeAdcWaveform(scopeMv, targetPeakMv, baselineAdc);
  let captured = 0;
  let saturated = 0;
  let rejected = 0;
  let peakMin = Infinity;
  let peakMax = 0;
  let areaMvUsMin = Infinity;
  let areaMvUsMax = 0;

  for (let phase = 0; phase < PHASE_STEP; phase += 1) {
    const t = replayPhase(adcWaveform, phase);
    const valid = t.events.find((ev) => ev.valid);
    if (valid) {
      if (!(Number.isFinite(valid.areaMvUs) && valid.areaMvUs > 0)) {
        throw new Error(`missing areaMvUs for phase ${phase}`);
      }
      if (valid.impulseCalibrated !== valid.areaMvUs) {
        throw new Error(`default impulse calibration mismatch for phase ${phase}`);
      }
      captured += 1;
      saturated += valid.saturated ? 1 : 0;
      peakMin = Math.min(peakMin, valid.peakMv);
      peakMax = Math.max(peakMax, valid.peakMv);
      areaMvUsMin = Math.min(areaMvUsMin, valid.areaMvUs);
      areaMvUsMax = Math.max(areaMvUsMax, valid.areaMvUs);
    } else if (t.rejectCount > 0) {
      rejected += 1;
    }
  }

  const capturePct = (captured * 100) / PHASE_STEP;
  console.log(`target=${targetPeakMv}mV baseline_adc=${baselineAdc}`);
  console.log(`phases=${PHASE_STEP} captured=${captured}/${PHASE_STEP} (${capturePct.toFixed(1)}%) rejected_phases=${rejected} saturated=${saturated}`);
  console.log(`peak_mv_range=${Number.isFinite(peakMin) ? peakMin : 0}..${peakMax}`);
  console.log(`area_mv_us_range=${Number.isFinite(areaMvUsMin) ? areaMvUsMin : 0}..${areaMvUsMax}`);

  if (targetPeakMv >= 500 && capturePct < 95) {
    process.exitCode = 1;
  }
}

const baselineArg = process.argv.find((arg) => arg.startsWith("--baseline-adc="));
const baselineAdc = baselineArg ? Number(baselineArg.split("=")[1]) : 300;
runSyntheticRegressionTests();
if (process.argv.includes("--self-test")) {
  console.log("synthetic regression tests passed");
  process.exit(0);
}
run(500, baselineAdc);
run(2800, baselineAdc);
