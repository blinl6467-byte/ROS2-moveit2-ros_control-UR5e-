#!/usr/bin/env python3
import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


STRATEGY_ORDER = ["slow", "medium", "fast"]
COLORS = ["#2f6f9f", "#d85c41", "#6a994e", "#8b6fcb"]


def parse_args():
    default_dir = "src/ur5e_benchmark/results/control_strategy_comparison"
    parser = argparse.ArgumentParser(
        description="Plot UR5e position trajectory controller benchmark results."
    )
    parser.add_argument(
        "--summary-csv",
        default=f"{default_dir}/control_strategy_summary.csv",
        help="Strategy-level summary CSV.",
    )
    parser.add_argument(
        "--raw-csv",
        default=f"{default_dir}/control_strategy_benchmark.csv",
        help="Per-trial raw benchmark CSV.",
    )
    parser.add_argument(
        "--output-dir",
        default=default_dir,
        help="Directory for generated PNG files.",
    )
    return parser.parse_args()


def read_rows(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def as_float(row, key):
    value = row.get(key, "")
    return float(value) if value else 0.0


def ordered_rows(rows):
    order = {name: index for index, name in enumerate(STRATEGY_ORDER)}
    return sorted(rows, key=lambda row: order.get(row["strategy"], len(order)))


def setup_axes(ax, title, ylabel=None, xlabel=None):
    ax.set_title(title, pad=10)
    if ylabel:
        ax.set_ylabel(ylabel)
    if xlabel:
        ax.set_xlabel(xlabel)
    ax.grid(True, axis="y", alpha=0.25)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def save(fig, output_dir, filename, close=True):
    path = output_dir / filename
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    if close:
        plt.close(fig)
    print(f"Saved {path}")


def plot_duration(rows, output_dir):
    rows = ordered_rows(rows)
    strategies = [row["strategy"] for row in rows]
    planned = [as_float(row, "avg_planned_duration") for row in rows]
    measured = [as_float(row, "avg_measured_duration") for row in rows]

    fig, ax = plt.subplots(figsize=(8.5, 5.2))
    x_values = list(range(len(rows)))
    width = 0.34
    ax.bar(
        [x - width / 2.0 for x in x_values],
        planned,
        width=width,
        label="Planned",
        color=COLORS[0],
    )
    ax.bar(
        [x + width / 2.0 for x in x_values],
        measured,
        width=width,
        label="Measured",
        color=COLORS[1],
    )
    ax.set_xticks(x_values)
    ax.set_xticklabels(strategies)
    setup_axes(ax, "Planned vs Measured Duration", "Duration (s)")
    ax.legend(frameon=False)
    save(fig, output_dir, "control_duration.png")


def plot_tracking_error(rows, output_dir):
    rows = ordered_rows(rows)
    strategies = [row["strategy"] for row in rows]
    metrics = [
        ("avg_rms_error", "RMS"),
        ("avg_max_abs_error", "Max"),
        ("avg_steady_state_max_error", "Steady-state"),
    ]

    fig, ax = plt.subplots(figsize=(9.5, 5.2))
    x_values = list(range(len(rows)))
    width = 0.24
    offset_start = -width
    for index, (key, label) in enumerate(metrics):
        ax.bar(
            [x + offset_start + index * width for x in x_values],
            [as_float(row, key) for row in rows],
            width=width,
            label=label,
            color=COLORS[index],
        )
    ax.set_xticks(x_values)
    ax.set_xticklabels(strategies)
    setup_axes(ax, "Tracking Error by Speed Strategy", "Position error (rad)")
    ax.legend(frameon=False)
    save(fig, output_dir, "control_tracking_error.png", close=False)
    save(fig, output_dir, "control_rms_error.png")


def plot_response_time(rows, output_dir):
    rows = ordered_rows(rows)
    strategies = [row["strategy"] for row in rows]
    response = [as_float(row, "avg_response_time_90") for row in rows]
    settling = [as_float(row, "avg_settling_time") for row in rows]

    fig, ax = plt.subplots(figsize=(8.5, 5.2))
    x_values = list(range(len(rows)))
    width = 0.34
    ax.bar(
        [x - width / 2.0 for x in x_values],
        response,
        width=width,
        label="90% response",
        color=COLORS[0],
    )
    ax.bar(
        [x + width / 2.0 for x in x_values],
        settling,
        width=width,
        label="Settling",
        color=COLORS[2],
    )
    ax.set_xticks(x_values)
    ax.set_xticklabels(strategies)
    setup_axes(ax, "Response and Settling Time", "Time (s)")
    ax.legend(frameon=False)
    save(fig, output_dir, "control_response_time.png")


def plot_overshoot(rows, output_dir):
    rows = ordered_rows(rows)
    strategies = [row["strategy"] for row in rows]
    overshoot = [as_float(row, "avg_max_overshoot_percent") for row in rows]

    fig, ax = plt.subplots(figsize=(8.5, 5.2))
    ax.bar(strategies, overshoot, color=COLORS[3])
    setup_axes(ax, "Maximum Overshoot", "Overshoot (%)")
    save(fig, output_dir, "control_overshoot.png")


def plot_speed_error_tradeoff(raw_rows, output_dir):
    fig, ax = plt.subplots(figsize=(8.2, 5.4))
    for index, strategy in enumerate(STRATEGY_ORDER):
        rows = [row for row in raw_rows if row["strategy"] == strategy]
        if not rows:
            continue
        ax.scatter(
            [as_float(row, "velocity_scale") for row in rows],
            [as_float(row, "rms_error") for row in rows],
            s=90,
            label=strategy,
            color=COLORS[index],
            edgecolor="white",
            linewidth=0.8,
        )
    setup_axes(
        ax,
        "Velocity Scale vs RMS Tracking Error",
        "RMS position error (rad)",
        "Velocity scaling factor",
    )
    ax.legend(frameon=False)
    save(fig, output_dir, "control_speed_error_tradeoff.png")


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    summary_rows = read_rows(args.summary_csv)
    raw_rows = read_rows(args.raw_csv)

    plot_duration(summary_rows, output_dir)
    plot_tracking_error(summary_rows, output_dir)
    plot_response_time(summary_rows, output_dir)
    plot_overshoot(summary_rows, output_dir)
    plot_speed_error_tradeoff(raw_rows, output_dir)


if __name__ == "__main__":
    main()
