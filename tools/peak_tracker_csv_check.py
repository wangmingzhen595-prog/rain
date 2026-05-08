from pathlib import Path
import statistics

CSV = Path("D:/\u5b9e\u9a8c\u6570\u636e/DEFAULT0.csv")
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
    assert 250.0 <= rise_us <= 900.0
    assert 1500.0 <= width_us <= 3500.0


if __name__ == "__main__":
    main()
