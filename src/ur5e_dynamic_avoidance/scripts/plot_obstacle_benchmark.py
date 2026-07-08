#!/usr/bin/env python3
import argparse
import csv
import math
import os
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


SCENARIO_ORDER = [
    "baseline",
    "table_cube_goal",
    "narrow_passage",
    "mid_arm_obstacle",
    "combined_hard",
]
DIFFICULTY_ORDER = ["easy", "medium", "hard"]
PLANNER_ORDER = [
    "RRTConnectkConfigDefault",
    "RRTkConfigDefault",
    "RRTstarkConfigDefault",
    "PRMkConfigDefault",
]
COLORS = ["#2f6f9f", "#d85c41", "#6a994e", "#8b6fcb", "#c49a2c"]


def parse_args():
    default_results_dir = (
        "src/ur5e_dynamic_avoidance/results/obstacle_benchmark"
    )
    parser = argparse.ArgumentParser(
        description="Generate presentation plots for UR5e obstacle benchmark CSV files."
    )
    parser.add_argument(
        "--planner-summary",
        default=f"{default_results_dir}/obstacle_planner_summary.csv",
        help="CSV grouped by scenario and planner.",
    )
    parser.add_argument(
        "--target-summary",
        default=f"{default_results_dir}/obstacle_target_summary.csv",
        help="CSV grouped by scenario, planner, and target.",
    )
    parser.add_argument(
        "--difficulty-summary",
        default=f"{default_results_dir}/obstacle_difficulty_summary.csv",
        help="CSV grouped by scenario, target difficulty, and planner.",
    )
    parser.add_argument(
        "--output-dir",
        default="src/ur5e_dynamic_avoidance/results/obstacle_benchmark/plots",
        help="Directory for generated PNG files.",
    )
    return parser.parse_args()


def read_rows(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def as_float(row, key):
    value = row.get(key, "")
    if value == "":
        return math.nan
    return float(value)


def planner_label(name):
    labels = {
        "RRTConnectkConfigDefault": "RRTConnect",
        "RRTkConfigDefault": "RRT",
        "RRTstarkConfigDefault": "RRT*",
        "PRMkConfigDefault": "PRM",
    }
    return labels.get(name, name)


def scenario_label(name):
    labels = {
        "baseline": "Baseline",
        "table_cube_goal": "Table+Cube",
        "narrow_passage": "Narrow",
        "mid_arm_obstacle": "Mid-arm",
        "combined_hard": "Combined",
    }
    return labels.get(name, name)


def ordered_values(values, preferred_order):
    preferred = [item for item in preferred_order if item in values]
    extras = sorted(item for item in values if item not in preferred_order)
    return preferred + extras


def setup_axes(ax, title, ylabel=None, xlabel=None, ylim=None):
    ax.set_title(title, pad=10)
    if ylabel:
        ax.set_ylabel(ylabel)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylim:
        ax.set_ylim(*ylim)
    ax.grid(True, axis="y", alpha=0.25)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def save(fig, output_dir, filename):
    path = output_dir / filename
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print(f"Saved {path}")


def plot_success_by_scene(rows, output_dir):
    scenarios = ordered_values({row["scenario"] for row in rows}, SCENARIO_ORDER)
    planners = ordered_values({row["planner"] for row in rows}, PLANNER_ORDER)
    values = {
        (row["scenario"], row["planner"]): as_float(row, "success_rate") * 100.0
        for row in rows
    }

    fig, ax = plt.subplots(figsize=(11, 5.8))
    x_positions = list(range(len(scenarios)))
    width = min(0.18, 0.72 / max(len(planners), 1))
    offset_start = -width * (len(planners) - 1) / 2.0
    for index, planner in enumerate(planners):
        heights = [values.get((scenario, planner), 0.0) for scenario in scenarios]
        bars = [
            x + offset_start + index * width
            for x in x_positions
        ]
        ax.bar(
            bars,
            heights,
            width=width,
            label=planner_label(planner),
            color=COLORS[index % len(COLORS)],
        )

    ax.set_xticks(x_positions)
    ax.set_xticklabels([scenario_label(item) for item in scenarios])
    setup_axes(ax, "Planner Success Rate by Scene", "Success rate (%)", ylim=(0, 108))
    ax.legend(ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.12))
    save(fig, output_dir, "success_rate_by_scene.png")


def plot_planning_time(rows, output_dir):
    scenarios = ordered_values({row["scenario"] for row in rows}, SCENARIO_ORDER)
    planners = ordered_values({row["planner"] for row in rows}, PLANNER_ORDER)
    values = {
        (row["scenario"], row["planner"]): as_float(row, "avg_planning_wall_time")
        for row in rows
    }

    fig, ax = plt.subplots(figsize=(11, 5.8))
    for index, planner in enumerate(planners):
        y_values = [values.get((scenario, planner), math.nan) for scenario in scenarios]
        ax.plot(
            [scenario_label(item) for item in scenarios],
            y_values,
            marker="o",
            linewidth=2.0,
            label=planner_label(planner),
            color=COLORS[index % len(COLORS)],
        )

    setup_axes(ax, "Planning Wall Time by Scene", "Time (s)")
    ax.legend(ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.12))
    save(fig, output_dir, "planning_time_by_scene.png")


def plot_path_length(rows, output_dir):
    scenarios = ordered_values({row["scenario"] for row in rows}, SCENARIO_ORDER)
    planners = ordered_values({row["planner"] for row in rows}, PLANNER_ORDER)
    values = {
        (row["scenario"], row["planner"]): as_float(row, "avg_joint_path_length")
        for row in rows
    }

    fig, ax = plt.subplots(figsize=(11, 5.8))
    for index, planner in enumerate(planners):
        y_values = [values.get((scenario, planner), math.nan) for scenario in scenarios]
        ax.plot(
            [scenario_label(item) for item in scenarios],
            y_values,
            marker="s",
            linewidth=2.0,
            label=planner_label(planner),
            color=COLORS[index % len(COLORS)],
        )

    setup_axes(ax, "Average Successful Joint Path Length", "Joint-space path length")
    ax.legend(ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.12))
    save(fig, output_dir, "joint_path_length_by_scene.png")


def plot_success_by_difficulty(rows, output_dir):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["scenario"], row["target_difficulty"])].append(
            as_float(row, "success_rate")
        )

    scenarios = ordered_values({row["scenario"] for row in rows}, SCENARIO_ORDER)
    difficulties = ordered_values(
        {row["target_difficulty"] for row in rows}, DIFFICULTY_ORDER
    )
    fig, ax = plt.subplots(figsize=(11, 5.8))
    x_positions = list(range(len(scenarios)))
    width = min(0.22, 0.72 / max(len(difficulties), 1))
    offset_start = -width * (len(difficulties) - 1) / 2.0
    for index, difficulty in enumerate(difficulties):
        heights = []
        for scenario in scenarios:
            values = grouped.get((scenario, difficulty), [])
            heights.append((sum(values) / len(values) * 100.0) if values else 0.0)
        bars = [x + offset_start + index * width for x in x_positions]
        ax.bar(
            bars,
            heights,
            width=width,
            label=difficulty.title(),
            color=COLORS[index % len(COLORS)],
        )

    ax.set_xticks(x_positions)
    ax.set_xticklabels([scenario_label(item) for item in scenarios])
    setup_axes(ax, "Success Rate by Target Difficulty", "Success rate (%)", ylim=(0, 108))
    ax.legend(ncol=3, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.12))
    save(fig, output_dir, "success_rate_by_difficulty.png")


def plot_target_heatmap(rows, output_dir):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["scenario"], row["target"])].append(as_float(row, "success_rate"))

    scenarios = ordered_values({row["scenario"] for row in rows}, SCENARIO_ORDER)
    targets = sorted({row["target"] for row in rows})
    matrix = []
    for target in targets:
        line = []
        for scenario in scenarios:
            values = grouped.get((scenario, target), [])
            line.append((sum(values) / len(values) * 100.0) if values else 0.0)
        matrix.append(line)

    fig, ax = plt.subplots(figsize=(10.5, 6.2))
    image = ax.imshow(matrix, cmap="YlGnBu", vmin=0.0, vmax=100.0, aspect="auto")
    ax.set_xticks(range(len(scenarios)))
    ax.set_xticklabels([scenario_label(item) for item in scenarios], rotation=20, ha="right")
    ax.set_yticks(range(len(targets)))
    ax.set_yticklabels(targets)
    ax.set_title("Target Success Rate Heatmap", pad=10)
    for y_index, row in enumerate(matrix):
        for x_index, value in enumerate(row):
            ax.text(
                x_index,
                y_index,
                f"{value:.0f}%",
                ha="center",
                va="center",
                color="black" if value < 70.0 else "white",
                fontsize=9,
            )
    fig.colorbar(image, ax=ax, label="Success rate (%)")
    save(fig, output_dir, "target_success_heatmap.png")


def plot_efficiency_scatter(rows, output_dir):
    fig, ax = plt.subplots(figsize=(8.5, 6))
    planners = ordered_values({row["planner"] for row in rows}, PLANNER_ORDER)
    for index, planner in enumerate(planners):
        planner_rows = [row for row in rows if row["planner"] == planner]
        x_values = [as_float(row, "avg_planning_wall_time") for row in planner_rows]
        y_values = [as_float(row, "success_rate") * 100.0 for row in planner_rows]
        ax.scatter(
            x_values,
            y_values,
            s=90,
            label=planner_label(planner),
            color=COLORS[index % len(COLORS)],
            edgecolor="white",
            linewidth=0.8,
        )

    setup_axes(
        ax,
        "Planner Efficiency Scatter",
        "Success rate (%)",
        "Planning wall time (s)",
        ylim=(-5, 108),
    )
    ax.legend(frameon=False)
    save(fig, output_dir, "planner_efficiency_scatter.png")


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    planner_rows = read_rows(args.planner_summary)
    target_rows = read_rows(args.target_summary)
    difficulty_rows = read_rows(args.difficulty_summary)

    plot_success_by_scene(planner_rows, output_dir)
    plot_planning_time(planner_rows, output_dir)
    plot_path_length(planner_rows, output_dir)
    plot_success_by_difficulty(difficulty_rows, output_dir)
    plot_target_heatmap(target_rows, output_dir)
    plot_efficiency_scatter(planner_rows, output_dir)


if __name__ == "__main__":
    main()
