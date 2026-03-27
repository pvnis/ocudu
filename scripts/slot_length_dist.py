#!/usr/bin/env python3
"""
Compute the distribution of slot lengths from srsRAN lower PHY uplink log output.

Parses lines of the form:
  2024-03-27T10:00:00.123456 [ERROR] [LOWER PHY ] Slot.indication : RU/UL-slot=N

Usage:
  ./slot_length_dist.py gnb.log
  journalctl -u gnb | ./slot_length_dist.py
  cat gnb.log | ./slot_length_dist.py -
"""

import sys
import re
import argparse
from datetime import datetime
from collections import Counter

# srslog timestamp: 2024-03-27T10:00:00.123456
PATTERN = re.compile(
    r"^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d+)"   # timestamp
    r".*?"                                               # level/logger
    r"Slot\.indication\s*:\s*RU/UL-slot=([0-9.]+)"      # slot number / slot point
)

TIMESTAMP_FMT = "%Y-%m-%dT%H:%M:%S.%f"


def parse_timestamp(ts_str: str) -> datetime:
    # Truncate sub-microsecond digits if present (strptime only handles 6)
    dot = ts_str.rfind(".")
    if dot != -1 and len(ts_str) - dot - 1 > 6:
        ts_str = ts_str[: dot + 7]
    return datetime.strptime(ts_str, TIMESTAMP_FMT)


def parse_slot_value(slot_str: str) -> tuple[int | None, tuple[int, ...]]:
    parts = tuple(int(part) for part in slot_str.split("."))
    if len(parts) == 1:
        return parts[0], parts
    return None, parts


def flatten_slot_points(slot_parts: list[tuple[int, ...]]) -> list[int]:
    if not slot_parts:
        return []
    if any(len(parts) != len(slot_parts[0]) for parts in slot_parts):
        raise ValueError("Mixed slot formats in log")
    if len(slot_parts[0]) == 1:
        return [parts[0] for parts in slot_parts]
    if len(slot_parts[0]) != 3:
        raise ValueError(f"Unsupported slot point format with {len(slot_parts[0])} fields")

    _, frame_indices, slot_indices = zip(*slot_parts)
    slots_per_frame = max(slot_indices) + 1
    # Lower-PHY slot logs currently expose a bare slot counter plus SFN/slot formatting.
    # The leading field is not reliable across rollovers, so unwrap continuity from SFN/slot only.
    out: list[int] = []
    epoch = 0
    prev = None
    for _, sfn, slot in slot_parts:
        current = sfn * slots_per_frame + slot
        if prev is not None and current < prev:
            epoch += 1024 * slots_per_frame
        out.append(epoch + current)
        prev = current
    return out


def percentile(sorted_data: list[float], p: float) -> float:
    if not sorted_data:
        return float("nan")
    idx = (len(sorted_data) - 1) * p / 100.0
    lo, hi = int(idx), min(int(idx) + 1, len(sorted_data) - 1)
    return sorted_data[lo] + (sorted_data[hi] - sorted_data[lo]) * (idx - lo)


def outlier_fence(sorted_data: list[float], k: float = 3.0) -> float:
    """Return Q3 + k*IQR as the upper outlier fence."""
    q1 = percentile(sorted_data, 25)
    q3 = percentile(sorted_data, 75)
    return q3 + k * (q3 - q1)


def split_outliers(sorted_data: list[float], fence: float) -> tuple[list[float], list[float]]:
    cut = next((i for i, v in enumerate(sorted_data) if v > fence), len(sorted_data))
    return sorted_data[:cut], sorted_data[cut:]


def print_histogram(deltas_us: list[float], bins: int = 20) -> None:
    lo, hi = min(deltas_us), max(deltas_us)
    if lo == hi:
        print(f"  All values identical: {lo:.1f} µs")
        return
    width = (hi - lo) / bins
    counts: list[int] = [0] * bins
    for v in deltas_us:
        b = min(int((v - lo) / width), bins - 1)
        counts[b] += 1
    max_count = max(counts)
    bar_width = 40
    for i, c in enumerate(counts):
        left = lo + i * width
        right = left + width
        bar = "#" * int(bar_width * c / max_count) if max_count else ""
        print(f"  [{left:8.1f}, {right:8.1f}) µs | {bar:<{bar_width}} {c}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logfile", nargs="?", default="-",
                        help="Log file path, or '-' for stdin (default: stdin)")
    parser.add_argument("--bins", type=int, default=20,
                        help="Number of histogram bins (default: 20)")
    parser.add_argument("--no-histogram", action="store_true",
                        help="Skip the ASCII histogram")
    parser.add_argument("--fence", type=float, default=3.0,
                        help="IQR multiplier k for outlier fence Q3 + k*IQR (default: 3.0)")
    args = parser.parse_args()

    src = open(args.logfile) if args.logfile != "-" else sys.stdin

    timestamps: list[datetime] = []
    slot_values: list[int | None] = []
    slot_parts: list[tuple[int, ...]] = []
    skipped = 0

    with src:
        for line in src:
            m = PATTERN.search(line)
            if not m:
                continue
            try:
                timestamps.append(parse_timestamp(m.group(1)))
                slot_value, slot_point = parse_slot_value(m.group(2))
                slot_values.append(slot_value)
                slot_parts.append(slot_point)
            except ValueError:
                skipped += 1

    if len(timestamps) < 2:
        print(f"Need at least 2 slot indications; found {len(timestamps)}.", file=sys.stderr)
        sys.exit(1)

    deltas_us = [
        (timestamps[i + 1] - timestamps[i]).total_seconds() * 1e6
        for i in range(len(timestamps) - 1)
    ]

    slot_nums = flatten_slot_points(slot_parts)

    # Slot-number deltas.
    slot_deltas = [
        slot_nums[i + 1] - slot_nums[i]
        for i in range(len(slot_nums) - 1)
    ]

    # Filter out negative time deltas (log reorder / wrap-around artefacts)
    negative = sum(1 for d in deltas_us if d < 0)
    deltas_us = [d for d in deltas_us if d >= 0]

    if not deltas_us:
        print("No valid (non-negative) inter-slot deltas found.", file=sys.stderr)
        sys.exit(1)

    deltas_us.sort()
    n = len(deltas_us)
    mean_us = sum(deltas_us) / n
    variance = sum((d - mean_us) ** 2 for d in deltas_us) / n
    stddev_us = variance ** 0.5

    # Missing slots: slot_delta > 1 means slots were skipped
    missing_events = [(i, slot_nums[i], slot_nums[i + 1], slot_deltas[i] - 1)
                      for i, d in enumerate(slot_deltas) if d > 1]
    total_missing = sum(d - 1 for d in slot_deltas if d > 1)
    slot_delta_counts: Counter[int] = Counter(slot_deltas)

    print(f"Slot indications  : {len(timestamps)}")
    print(f"Inter-slot deltas : {n}  (negative/skipped: {negative + skipped})")
    print(f"Missing slots     : {total_missing}  ({len(missing_events)} gap(s))")
    print()

    # Slot-number delta distribution
    if len(slot_delta_counts) > 1 or (slot_delta_counts and list(slot_delta_counts)[0] != 1):
        print("Slot-number delta counts:")
        for delta in sorted(slot_delta_counts):
            tag = "  <-- gap" if delta > 1 else ""
            print(f"  delta={delta:4d} : {slot_delta_counts[delta]}{tag}")
        print()

    if missing_events:
        print(f"Gap details (slot_from -> slot_to, missing count):")
        for _, s_from, s_to, n_miss in missing_events:
            print(f"  {s_from} -> {s_to}  ({n_miss} missing)")
        print()

    print("Distribution of inter-slot lengths (µs):")
    print(f"  min    : {deltas_us[0]:.1f}")
    print(f"  p1     : {percentile(deltas_us,  1):.1f}")
    print(f"  p5     : {percentile(deltas_us,  5):.1f}")
    print(f"  p25    : {percentile(deltas_us, 25):.1f}")
    print(f"  median : {percentile(deltas_us, 50):.1f}")
    print(f"  mean   : {mean_us:.1f}")
    print(f"  stddev : {stddev_us:.1f}")
    print(f"  p75    : {percentile(deltas_us, 75):.1f}")
    print(f"  p95    : {percentile(deltas_us, 95):.1f}")
    print(f"  p99    : {percentile(deltas_us, 99):.1f}")
    print(f"  max    : {deltas_us[-1]:.1f}")
    print()

    # Split inliers / outliers using Tukey fence (Q3 + k*IQR)
    fence = outlier_fence(deltas_us, args.fence)
    inliers, outliers = split_outliers(deltas_us, fence)

    if outliers:
        print(f"Outliers (> {fence:.1f} µs, i.e. Q3 + {args.fence}*IQR): {len(outliers)}")
        for v in outliers:
            print(f"  {v:.1f} µs")
        print()

    # Bucket by nearest 500 µs — restricted to inliers so gaps don't appear
    work = inliers if inliers else deltas_us
    bucket_size = 500
    bucket_counts: Counter[int] = Counter(
        int(d / bucket_size) * bucket_size for d in work
    )
    label = " (inliers only)" if outliers else ""
    print(f"Counts by {bucket_size} µs bucket{label}:")
    for b in sorted(bucket_counts):
        print(f"  [{b:6d}, {b + bucket_size:6d}) µs : {bucket_counts[b]}")
    print()

    if not args.no_histogram:
        hist_data = inliers if inliers else deltas_us
        label = f" — {len(outliers)} outlier(s) excluded" if outliers else ""
        print(f"Histogram ({args.bins} bins){label}:")
        print_histogram(hist_data, bins=args.bins)


if __name__ == "__main__":
    main()
