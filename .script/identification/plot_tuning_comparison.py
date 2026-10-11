#!/usr/bin/env python3
"""Draw a multi-panel comparison of yaw step response across tuning versions.

Each panel holds one metric. Bars are grouped by step amplitude, one bar per
version. Output is SVG, so no plotting library is needed.

Usage:
    python3 plot_tuning_comparison.py -o out.svg baseline=data/yaw_baseline.csv v1=data/yaw_pid_v1.csv
"""

import argparse
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analyze_step_response as step  # noqa: E402

# metric key, panel title, value format, whether to take the absolute value
PANELS = [
    ("rise_time_s", "上升时间 s（越小越好）", "{:.3f}", False),
    ("overshoot_percent", "欠冲幅度 %（越小越好）", "{:.2f}", True),
    ("steady_error_rad", "稳态误差绝对值 rad（越小越好）", "{:.4f}", True),
    ("angle_error_integral", "角度误差积分 rad·s（越小越好）", "{:.4f}", False),
    ("velocity_error_integral", "转速跟随误差积分 rad（越小越好）", "{:.4f}", False),
]

COLORS = ["#9e9e9e", "#90caf9", "#42a5f5", "#1565c0"]

FONT = "system-ui, 'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'Microsoft YaHei', sans-serif"

PANEL_W = 520
PANEL_H = 250
PLOT_X = 70
PLOT_Y = 48
PLOT_W = 430
PLOT_H = 150
COLS = 2


def load_groups(csv_path, dt):
    columns = step.load_columns(csv_path)
    control_angle = columns[step.COLUMN_CONTROL_ANGLE]
    valid = [i for i, v in enumerate(control_angle) if v is not None]
    if not valid:
        raise SystemExit(f"{csv_path}: no rows carry a target angle")
    first, last = valid[0], valid[-1]

    segments = []
    for start, end in step.find_segments(control_angle, first, last):
        if end - start < step.MIN_SEGMENT_SAMPLES:
            continue
        result = step.analyze_segment(columns, start, end, dt)
        if result is not None:
            segments.append(result)
    return step.group_segments(segments)


def aggregate(groups):
    """Return {amplitude: {metric: mean over both directions}}."""
    table = {}
    for (amplitude, direction), items in groups.items():
        row = table.setdefault(amplitude, {})
        for key, _, _, use_abs in PANELS:
            values = [item[key] for item in items if item[key] is not None]
            if not values:
                continue
            values = [abs(v) for v in values] if use_abs else values
            row.setdefault(key, []).extend(values)
    return {
        amplitude: {key: statistics.fmean(v) for key, v in row.items()}
        for amplitude, row in table.items()
    }


def nice_max(value):
    if value <= 0:
        return 1.0
    exponent = 10 ** math.floor(math.log10(value))
    return math.ceil(value / exponent * 1.15) * exponent


def fmt_tick(value, digits):
    text = f"{value:.{digits}f}"
    return text.rstrip("0").rstrip(".") if "." in text else text


def draw_panel(x, y, title, fmt, amplitudes, series, y_max, digits):
    out = [f'<g transform="translate({x},{y})">']
    out.append(
        f'<text x="0" y="16" font-family="{FONT}" font-size="14" fill="#212121">{title}</text>'
    )

    x0, y0 = PLOT_X, PLOT_Y
    out.append(
        f'<rect x="{x0}" y="{y0}" width="{PLOT_W}" height="{PLOT_H}" '
        'fill="#ffffff" stroke="#bdbdbd"/>'
    )

    for index in range(5):
        value = y_max * index / 4
        gy = y0 + PLOT_H - PLOT_H * index / 4
        out.append(
            f'<line x1="{x0}" y1="{gy:.1f}" x2="{x0 + PLOT_W}" y2="{gy:.1f}" '
            'stroke="#eeeeee" stroke-width="1"/>'
        )
        out.append(
            f'<text x="{x0 - 8}" y="{gy + 4:.1f}" font-family="{FONT}" font-size="11" '
            f'fill="#616161" text-anchor="end">{fmt_tick(value, digits)}</text>'
        )

    group_w = PLOT_W / max(len(amplitudes), 1)
    bar_w = (group_w - 14) / max(len(series), 1)

    for g_index, amplitude in enumerate(amplitudes):
        gx = x0 + group_w * g_index
        for s_index, (_, values, color) in enumerate(series):
            value = values.get(amplitude)
            if value is None:
                continue
            height = PLOT_H * value / y_max
            bx = gx + 8 + bar_w * s_index
            by = y0 + PLOT_H - height
            out.append(
                f'<rect x="{bx:.1f}" y="{by:.1f}" width="{bar_w:.1f}" '
                f'height="{height:.1f}" fill="{color}"/>'
            )
        label = f"{amplitude:.2f}"
        out.append(
            f'<text x="{gx + group_w / 2:.1f}" y="{y0 + PLOT_H + 18}" font-family="{FONT}" '
            f'font-size="11" fill="#424242" text-anchor="middle">{label}</text>'
        )

    out.append(
        f'<text x="{x0 + PLOT_W / 2}" y="{y0 + PLOT_H + 38}" font-family="{FONT}" '
        'font-size="11" fill="#757575" text-anchor="middle">阶跃幅值 rad</text>'
    )
    out.append("</g>")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--output", required=True, help="SVG file to write")
    parser.add_argument("--dt", type=float, default=step.DEFAULT_DT)
    parser.add_argument(
        "versions",
        metavar="LABEL=CSV",
        nargs="+",
        help="one or more label=path pairs, drawn in the given order",
    )
    args = parser.parse_args()

    tables = []
    for item in args.versions:
        label, _, path = item.partition("=")
        if not label or not path:
            raise SystemExit(f"expected LABEL=CSV, got {item}")
        tables.append((label, aggregate(load_groups(path, args.dt))))

    amplitudes = sorted(set().union(*(table.keys() for _, table in tables)))

    rows = math.ceil(len(PANELS) / COLS)
    width = 40 + COLS * PANEL_W + 40
    height = 100 + rows * PANEL_H + 30

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        f'<text x="40" y="34" font-family="{FONT}" font-size="18" fill="#212121">'
        "云台 yaw 轴阶跃响应：基线与各调优版本对比</text>",
    ]

    legend_x = 40
    for index, (label, _) in enumerate(tables):
        color = COLORS[index % len(COLORS)]
        lx = legend_x + index * 130
        out.append(f'<rect x="{lx}" y="52" width="16" height="12" fill="{color}"/>')
        out.append(
            f'<text x="{lx + 22}" y="62" font-family="{FONT}" font-size="12" '
            f'fill="#424242">{label}</text>'
        )

    for index, (key, title, fmt, _) in enumerate(PANELS):
        series = [
            (label, {a: row.get(key) for a, row in table.items()}, COLORS[i % len(COLORS)])
            for i, (label, table) in enumerate(tables)
        ]
        values = [v for _, values, _ in series for v in values.values() if v is not None]
        y_max = nice_max(max(values)) if values else 1.0
        digits = len(fmt.split(".")[1].rstrip("f"))
        col, row = index % COLS, index // COLS
        out += draw_panel(
            40 + col * PANEL_W,
            100 + row * PANEL_H,
            title,
            fmt,
            amplitudes,
            series,
            y_max,
            digits,
        )

    out.append("</svg>")

    with open(args.output, "w") as handle:
        handle.write("\n".join(out) + "\n")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
