#!/usr/bin/env python3
"""Compute step-response metrics from a gimbal yaw step test log.

The log is written by the ``value_collector`` component, which samples every
``write_interval`` executor cycles. With ``update_rate: 1000.0`` and
``write_interval: 5`` the sample period is 5 ms.

Rows recorded before the step generator is armed carry no target angle, so the
valid region is cut by dropping rows whose control angle is empty. Inside that
region the sample period is constant and integration is a plain sum over rows.

Usage:
    python3 analyze_step_response.py data/yaw_baseline_v3.csv
    python3 analyze_step_response.py data/yaw_baseline_v3.csv --dt 0.005 --json-output result.json
"""

import argparse
import csv
import json
import math
import statistics
import sys

DEFAULT_DT = 0.005

COLUMN_ANGLE = "/gimbal/yaw/angle"
COLUMN_VELOCITY_MOTOR = "/gimbal/yaw/velocity"
COLUMN_VELOCITY_IMU = "/gimbal/yaw/velocity_imu"
COLUMN_CONTROL_ANGLE = "/gimbal/yaw/target_angle"
COLUMN_CONTROL_VELOCITY = "/gimbal/yaw/target_velocity"
COLUMN_CONTROL_TORQUE = "/gimbal/yaw/control_torque"
COLUMN_TORQUE = "/gimbal/yaw/torque"

# A transition is any jump of the target angle larger than this value.
STEP_THRESHOLD_RAD = 0.02
# Segments shorter than this are discarded as leftovers.
MIN_SEGMENT_SAMPLES = 20
# Steady state is averaged over the last second of each segment.
STEADY_WINDOW_S = 1.0
# Rise time is measured at this fraction of the commanded travel.
RISE_FRACTION = 0.9


def to_float(text):
    """Parse a CSV cell. Empty cells and ``nan`` both become ``None``."""
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(value) else value


def load_columns(path):
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: empty file")
        missing = [
            name
            for name in (
                COLUMN_ANGLE,
                COLUMN_VELOCITY_IMU,
                COLUMN_CONTROL_ANGLE,
                COLUMN_CONTROL_VELOCITY,
            )
            if name not in reader.fieldnames
        ]
        if missing:
            raise ValueError(f"{path}: missing columns {', '.join(missing)}")

        rows = list(reader)

    columns = {name: [to_float(row.get(name)) for row in rows] for name in reader.fieldnames}
    return columns


def wrap_to_pi(value):
    """Normalize an angle difference to (-pi, pi].

    The gimbal yaw angle is reported in [0, 2*pi). A step that crosses the
    zero crossing makes the reading jump by 2*pi, which corrupts every metric
    computed from a raw difference.
    """
    two_pi = 2.0 * math.pi
    value = math.fmod(value, two_pi)
    if value > math.pi:
        value -= two_pi
    elif value <= -math.pi:
        value += two_pi
    return value


def find_segments(control_angle, first, last):
    """Return the half-open index ranges between successive target changes."""
    starts = [first]
    for index in range(first + 1, last + 1):
        previous, current = control_angle[index - 1], control_angle[index]
        if previous is None or current is None:
            continue
        if abs(current - previous) > STEP_THRESHOLD_RAD:
            starts.append(index)

    segments = []
    for position, start in enumerate(starts):
        end = starts[position + 1] if position + 1 < len(starts) else last + 1
        segments.append((start, end))
    return segments


def analyze_segment(columns, start, end, dt):
    angle = columns[COLUMN_ANGLE]
    velocity_imu = columns[COLUMN_VELOCITY_IMU]
    control_angle = columns[COLUMN_CONTROL_ANGLE]
    control_velocity = columns[COLUMN_CONTROL_VELOCITY]

    target = control_angle[start]
    start_angle = angle[max(start - 1, 0)]
    travel = wrap_to_pi(target - start_angle)
    if abs(travel) < STEP_THRESHOLD_RAD:
        return None

    window = range(start, end)
    angles = [angle[i] for i in window]
    velocities = [velocity_imu[i] for i in window]
    commands = [control_velocity[i] for i in window]
    if any(value is None for value in angles + velocities + commands):
        return None

    direction = 1.0 if travel > 0 else -1.0

    # Work with differences from the segment start so that a wrap at the zero
    # crossing cannot masquerade as a large travel.
    progress = [wrap_to_pi(value - start_angle) for value in angles]
    deviation = [wrap_to_pi(value - target) for value in angles]

    rise_time = None
    for offset, value in enumerate(progress):
        if value * direction >= RISE_FRACTION * abs(travel):
            rise_time = offset * dt
            break

    overshoot = max(value * direction for value in deviation) / abs(travel) * 100.0

    # 回零段只有 1 s，窗口取满整段会把回零的过渡过程算进稳态。上限取段长的一半
    tail = min(int(STEADY_WINDOW_S / dt), len(deviation) // 2)
    steady_error = statistics.fmean(deviation[-tail:])

    angle_error_integral = sum(abs(value) for value in deviation) * dt
    velocity_error_integral = (
        sum(abs(command - actual) for command, actual in zip(commands, velocities)) * dt
    )

    def peak(name):
        values = [abs(v) for v in (columns.get(name) or [])[start:end] if v is not None]
        return max(values) if values else None

    return {
        # Quantize so that the two repeats of one level land in the same group.
        "amplitude_rad": round(round(abs(travel) / 0.05) * 0.05, 2),
        "direction": int(direction),
        "rise_time_s": rise_time,
        "overshoot_percent": overshoot,
        "steady_error_rad": steady_error,
        "angle_error_integral": angle_error_integral,
        "velocity_error_integral": velocity_error_integral,
        "peak_velocity_rad_s": max(abs(v) for v in velocities),
        "peak_control_torque_n_m": peak(COLUMN_CONTROL_TORQUE),
        "peak_torque_n_m": peak(COLUMN_TORQUE),
    }


def group_segments(segments):
    groups = {}
    for item in segments:
        groups.setdefault((item["amplitude_rad"], item["direction"]), []).append(item)
    return groups


def mean_of(items, key):
    values = [item[key] for item in items if item[key] is not None]
    return statistics.fmean(values) if values else None


def format_value(value, digits):
    return "n/a" if value is None else f"{value:.{digits}f}"


def print_table(groups):
    header = (
        "| step rad | rise s | overshoot % | steady rad | angle err int "
        "| velocity err int | peak vel | peak cmd | peak actual | n |"
    )
    print(header)
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for (amplitude, direction) in sorted(groups, key=lambda key: (key[0], -key[1])):
        items = groups[(amplitude, direction)]
        label = f"{'+' if direction > 0 else '-'}{amplitude:.2f}"
        print(
            f"| {label} | {format_value(mean_of(items, 'rise_time_s'), 3)} "
            f"| {format_value(mean_of(items, 'overshoot_percent'), 1)} "
            f"| {format_value(mean_of(items, 'steady_error_rad'), 5)} "
            f"| {format_value(mean_of(items, 'angle_error_integral'), 4)} "
            f"| {format_value(mean_of(items, 'velocity_error_integral'), 3)} "
            f"| {format_value(mean_of(items, 'peak_velocity_rad_s'), 3)} "
            f"| {format_value(mean_of(items, 'peak_control_torque_n_m'), 1)} "
            f"| {format_value(mean_of(items, 'peak_torque_n_m'), 2)} | {len(items)} |"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", help="step test log written by value_collector")
    parser.add_argument(
        "--dt", type=float, default=DEFAULT_DT, help="sample period in seconds"
    )
    parser.add_argument("--json-output", help="write the metrics to this JSON file")
    args = parser.parse_args()

    columns = load_columns(args.csv_path)
    control_angle = columns[COLUMN_CONTROL_ANGLE]
    valid = [index for index, value in enumerate(control_angle) if value is not None]
    if not valid:
        print("error: no rows carry a target angle", file=sys.stderr)
        return 2
    first, last = valid[0], valid[-1]

    segments = []
    for start, end in find_segments(control_angle, first, last):
        if end - start < MIN_SEGMENT_SAMPLES:
            continue
        result = analyze_segment(columns, start, end, args.dt)
        if result is not None:
            segments.append(result)

    if not segments:
        print("error: no usable step segments", file=sys.stderr)
        return 2

    groups = group_segments(segments)
    print(f"CSV: {args.csv_path}")
    print(f"Valid rows: {len(valid)} (index {first}..{last}), { (last - first + 1) * args.dt:.1f} s")
    print(f"Steps: {len(segments)} segments, dt = {args.dt} s\n")
    print_table(groups)

    if args.json_output:
        payload = {
            "csv_path": args.csv_path,
            "dt": args.dt,
            "segments": segments,
            "groups": {
                f"{'+' if direction > 0 else '-'}{amplitude:.2f}": items
                for (amplitude, direction), items in groups.items()
            },
        }
        with open(args.json_output, "w") as handle:
            json.dump(payload, handle, indent=2)
        print(f"\nWrote {args.json_output}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
