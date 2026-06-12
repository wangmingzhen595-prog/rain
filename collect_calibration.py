#!/usr/bin/env python3
"""Collect rain-drop calibration EVT lines from a serial port."""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
from datetime import datetime


NEEDLES = {
    "0.50": {
        "volume_mm3": 10.416,
        "impulse_uNs": 17.87,
        "csv_file": "calib_0p50mm.csv",
    },
    "1.20": {
        "volume_mm3": 30.075,
        "impulse_uNs": 51.59,
        "csv_file": "calib_1p20mm.csv",
    },
    "2.00": {
        "volume_mm3": 38.461,
        "impulse_uNs": 65.98,
        "csv_file": "calib_2p00mm.csv",
    },
}

DROP_FIELDS = [
    "timestamp",
    "needle_mm",
    "volume_mm3",
    "impulse_uNs",
    "drop_id",
    "integral",
    "peak",
    "width",
    "rise",
    "fall",
    "raw_line",
]

SUMMARY_FILE = "calibration_summary.csv"
SUMMARY_FIELDS = [
    "timestamp",
    "needle_mm",
    "volume_mm3",
    "impulse_uNs",
    "target_drops",
    "collected_drops",
    "excluded_first_drops",
    "candidate_drops",
    "outlier_drops",
    "valid_drops",
    "integral_median_after_skip",
    "filter_low",
    "filter_high",
    "integral_median_valid",
    "integral_mean_valid",
    "integral_std_valid",
    "integral_cv_valid_pct",
    "csv_file",
]


def normalize_needle(value: str | float) -> str:
    """Return the canonical needle key for supported sizes."""
    try:
        numeric = float(str(value).strip())
    except ValueError as exc:
        raise ValueError("needle must be one of: 0.50, 1.20, 2.00") from exc

    for key in NEEDLES:
        if abs(float(key) - numeric) < 1e-9:
            return key
    raise ValueError("needle must be one of: 0.50, 1.20, 2.00")


def parse_evt_line(line: str) -> dict[str, object] | None:
    """Parse one firmware EVT line, ignoring all other lines.

    Two firmware layouts are supported (detected by field count):
      v3 (9 fields): EVT,drop_id,integral,vol_0p01mm3,total_0p01mm3,peak,width,rise,fall
      legacy (5-7 fields): EVT,drop_id,integral,peak,width[,rise[,fall]]
    """
    raw_line = line.strip()
    if not raw_line:
        return None

    try:
        fields = next(csv.reader([raw_line]))
    except csv.Error:
        return None

    fields = [field.strip() for field in fields]
    if not fields or fields[0] != "EVT" or len(fields) < 5:
        return None

    integral_text = fields[2]
    try:
        integral = float(integral_text)
    except ValueError:
        return None

    if len(fields) >= 9:
        peak, width, rise, fall = fields[5], fields[6], fields[7], fields[8]
    else:
        peak = fields[3]
        width = fields[4]
        rise = fields[5] if len(fields) >= 6 else ""
        fall = fields[6] if len(fields) >= 7 else ""

    return {
        "drop_id": fields[1],
        "integral": integral,
        "integral_text": integral_text,
        "peak": peak,
        "width": width,
        "rise": rise,
        "fall": fall,
        "raw_line": raw_line,
    }


def validate_target(value: str | int) -> int:
    try:
        target = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("target must be an integer from 30 to 50") from exc
    if not 30 <= target <= 50:
        raise argparse.ArgumentTypeError("target must be from 30 to 50 drops")
    return target


def now_timestamp() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def format_float(value: object, digits: int = 6) -> str:
    if value is None or value == "":
        return ""
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.{digits}g}"
    return str(value)


def sample_std(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) >= 2 else 0.0


def calculate_summary_stats(samples: list[dict[str, object]]) -> dict[str, object]:
    integrals = [float(sample["integral"]) for sample in samples]
    excluded_first = min(5, len(integrals))
    candidates = integrals[5:]

    stats: dict[str, object] = {
        "collected_drops": len(integrals),
        "excluded_first_drops": excluded_first,
        "candidate_drops": len(candidates),
        "outlier_drops": 0,
        "valid_drops": 0,
        "integral_median_after_skip": None,
        "filter_low": None,
        "filter_high": None,
        "integral_median_valid": None,
        "integral_mean_valid": None,
        "integral_std_valid": None,
        "integral_cv_valid_pct": None,
    }

    if not candidates:
        return stats

    median_after_skip = statistics.median(candidates)
    low = median_after_skip * 0.70
    high = median_after_skip * 1.30
    valid = [value for value in candidates if low <= value <= high]

    stats.update(
        {
            "outlier_drops": len(candidates) - len(valid),
            "valid_drops": len(valid),
            "integral_median_after_skip": median_after_skip,
            "filter_low": low,
            "filter_high": high,
        }
    )

    if valid:
        mean_valid = statistics.mean(valid)
        std_valid = sample_std(valid)
        cv_valid = (std_valid / mean_valid * 100.0) if mean_valid else 0.0
        stats.update(
            {
                "integral_median_valid": statistics.median(valid),
                "integral_mean_valid": mean_valid,
                "integral_std_valid": std_valid,
                "integral_cv_valid_pct": cv_valid,
            }
        )

    return stats


def open_drop_csv(path: str):
    needs_header = not os.path.exists(path) or os.path.getsize(path) == 0
    handle = open(path, "a", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=DROP_FIELDS)
    if needs_header:
        writer.writeheader()
        handle.flush()
    return handle, writer


def make_drop_row(needle_key: str, sample: dict[str, object]) -> dict[str, object]:
    needle = NEEDLES[needle_key]
    return {
        "timestamp": sample["timestamp"],
        "needle_mm": needle_key,
        "volume_mm3": needle["volume_mm3"],
        "impulse_uNs": needle["impulse_uNs"],
        "drop_id": sample["drop_id"],
        "integral": sample["integral_text"],
        "peak": sample["peak"],
        "width": sample["width"],
        "rise": sample["rise"],
        "fall": sample["fall"],
        "raw_line": sample["raw_line"],
    }


def print_progress(samples: list[dict[str, object]]) -> None:
    values = [float(sample["integral"]) for sample in samples]
    mean_value = statistics.mean(values)
    median_value = statistics.median(values)
    std_value = sample_std(values)
    cv_value = (std_value / mean_value * 100.0) if mean_value else 0.0
    current = values[-1]
    print(
        "drops={count:02d} integral={current:.6g} mean={mean:.6g} "
        "median={median:.6g} std={std:.6g} CV={cv:.2f}%".format(
            count=len(samples),
            current=current,
            mean=mean_value,
            median=median_value,
            std=std_value,
            cv=cv_value,
        ),
        flush=True,
    )


def build_summary_row(
    needle_key: str,
    target: int,
    stats: dict[str, object],
) -> dict[str, str]:
    needle = NEEDLES[needle_key]
    row: dict[str, str] = {
        "timestamp": now_timestamp(),
        "needle_mm": needle_key,
        "volume_mm3": format_float(needle["volume_mm3"]),
        "impulse_uNs": format_float(needle["impulse_uNs"]),
        "target_drops": str(target),
        "csv_file": str(needle["csv_file"]),
    }
    for field in SUMMARY_FIELDS:
        if field not in row:
            row[field] = format_float(stats.get(field))
    return row


def update_summary_csv(needle_key: str, target: int, stats: dict[str, object]) -> None:
    current_row = build_summary_row(needle_key, target, stats)
    rows_by_needle: dict[str, dict[str, str]] = {}

    if os.path.exists(SUMMARY_FILE) and os.path.getsize(SUMMARY_FILE) > 0:
        with open(SUMMARY_FILE, "r", newline="", encoding="utf-8-sig") as handle:
            for row in csv.DictReader(handle):
                needle_value = row.get("needle_mm", "").strip()
                if needle_value:
                    rows_by_needle[needle_value] = {
                        field: row.get(field, "") for field in SUMMARY_FIELDS
                    }

    rows_by_needle[needle_key] = current_row

    ordered_keys = [key for key in NEEDLES if key in rows_by_needle]
    ordered_keys.extend(
        key for key in sorted(rows_by_needle) if key not in ordered_keys
    )

    with open(SUMMARY_FILE, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for key in ordered_keys:
            writer.writerow(
                {field: rows_by_needle[key].get(field, "") for field in SUMMARY_FIELDS}
            )
        handle.flush()


def prompt_for_needle() -> str:
    print("Select needle size:")
    for index, key in enumerate(NEEDLES, start=1):
        needle = NEEDLES[key]
        print(
            f"  {index}. {key} mm, volume {needle['volume_mm3']} mm^3, "
            f"impulse {needle['impulse_uNs']} uN*s"
        )

    while True:
        choice = input("Needle [0.50/1.20/2.00 or 1/2/3]: ").strip()
        if choice in {"1", "2", "3"}:
            return list(NEEDLES)[int(choice) - 1]
        try:
            return normalize_needle(choice)
        except ValueError as exc:
            print(exc)


def import_serial_module():
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError:
        print(
            "pyserial is not installed. Install it first:\n"
            "  pip install pyserial",
            file=sys.stderr,
        )
        return None
    return serial


def collect_from_serial(port: str, baud: int, needle_key: str, target: int) -> int:
    serial = import_serial_module()
    if serial is None:
        return 2

    data_file = str(NEEDLES[needle_key]["csv_file"])
    samples: list[dict[str, object]] = []
    completed = False

    print(
        f"Opening {port} at {baud} baud, needle {needle_key} mm, target {target} drops.",
        flush=True,
    )
    print(f"Writing drop CSV: {data_file}", flush=True)
    print("Only EVT lines are recorded. Press Ctrl+C to stop.", flush=True)

    try:
        with serial.Serial(port=port, baudrate=baud, timeout=1) as ser:
            handle, writer = open_drop_csv(data_file)
            try:
                while len(samples) < target:
                    raw_bytes = ser.readline()
                    if not raw_bytes:
                        continue
                    line = raw_bytes.decode("utf-8", errors="replace")
                    sample = parse_evt_line(line)
                    if sample is None:
                        continue

                    sample["timestamp"] = now_timestamp()
                    writer.writerow(make_drop_row(needle_key, sample))
                    handle.flush()
                    samples.append(sample)
                    print_progress(samples)
                completed = True
            finally:
                handle.flush()
                handle.close()
    except KeyboardInterrupt:
        print("\nInterrupted by Ctrl+C. Flushed rows are already saved.", flush=True)
    except serial.SerialException as exc:
        print(f"Serial error on {port}: {exc}", file=sys.stderr, flush=True)
        return 3

    if not samples:
        print("No EVT drops were recorded.", flush=True)
        return 1 if not completed else 0

    if not completed:
        print(
            f"Stopped after {len(samples)} drops. Summary was not updated because "
            f"target {target} was not reached.",
            flush=True,
        )
        return 130

    stats = calculate_summary_stats(samples)
    update_summary_csv(needle_key, target, stats)
    print_summary(needle_key, target, stats)
    return 0


def print_summary(needle_key: str, target: int, stats: dict[str, object]) -> None:
    print("\nCollection complete.", flush=True)
    print(
        "needle={needle}mm target={target} collected={collected} "
        "valid={valid} outliers={outliers}".format(
            needle=needle_key,
            target=target,
            collected=stats["collected_drops"],
            valid=stats["valid_drops"],
            outliers=stats["outlier_drops"],
        ),
        flush=True,
    )
    print(
        "valid integral median={median} mean={mean} std={std} CV={cv}%".format(
            median=format_float(stats["integral_median_valid"]),
            mean=format_float(stats["integral_mean_valid"]),
            std=format_float(stats["integral_std_valid"]),
            cv=format_float(stats["integral_cv_valid_pct"]),
        ),
        flush=True,
    )
    print(f"Updated {SUMMARY_FILE}", flush=True)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect calibration EVT lines from STM32 rain sensor UART."
    )
    parser.add_argument("--port", default="COM3", help="serial port, default COM3")
    parser.add_argument(
        "--baud", type=int, default=115200, help="serial baud rate, default 115200"
    )
    parser.add_argument(
        "--needle",
        help="needle size in mm: 0.50, 1.20, or 2.00",
    )
    parser.add_argument(
        "--target",
        type=validate_target,
        default=40,
        help="target drops from 30 to 50, default 40",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    try:
        needle_key = normalize_needle(args.needle) if args.needle else prompt_for_needle()
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2

    return collect_from_serial(args.port, args.baud, needle_key, args.target)


if __name__ == "__main__":
    raise SystemExit(main())
