#!/usr/bin/env python3
import argparse
import csv
from collections import defaultdict
from statistics import mean


def read_rows(path):
    with open(path, newline="") as csv_file:
        return list(csv.DictReader(csv_file))


def f(row, key):
    value = row.get(key, "")
    return float(value) if value != "" else 0.0


def key_value(row, group_keys):
    if isinstance(group_keys, str):
        return row[group_keys]
    return tuple(row[key] for key in group_keys)


def summarize(rows, group_keys, numeric_keys):
    groups = defaultdict(list)
    for row in rows:
        groups[key_value(row, group_keys)].append(row)

    output = []
    for name, items in sorted(groups.items()):
        success_values = [f(row, "success") for row in items]
        if isinstance(group_keys, str):
            summary = {group_keys: name}
        else:
            summary = dict(zip(group_keys, name))
        summary.update({
            "trials": len(items),
            "success_rate": mean(success_values) if success_values else 0.0,
        })
        successful_items = [row for row in items if f(row, "success") > 0.5]
        source = successful_items if successful_items else items
        for key in numeric_keys:
            values = [f(row, key) for row in source]
            summary[f"avg_{key}"] = mean(values) if values else 0.0
        output.append(summary)
    return output


def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def print_table(title, rows):
    if not rows:
        return
    print(f"\n{title}")
    headers = list(rows[0].keys())
    print(",".join(headers))
    for row in rows:
        values = []
        for header in headers:
            value = row[header]
            if isinstance(value, float):
                values.append(f"{value:.6f}")
            else:
                values.append(str(value))
        print(",".join(values))


def main():
    parser = argparse.ArgumentParser(description="Summarize UR5e benchmark CSV files.")
    parser.add_argument("--planner-csv", default="/tmp/ur5e_planner_benchmark.csv")
    parser.add_argument("--control-csv", default="/tmp/ur5e_control_strategy_benchmark.csv")
    parser.add_argument("--planner-summary", default="/tmp/ur5e_planner_summary.csv")
    parser.add_argument("--planner-target-summary", default="/tmp/ur5e_planner_target_summary.csv")
    parser.add_argument("--control-summary", default="/tmp/ur5e_control_strategy_summary.csv")
    args = parser.parse_args()

    planner_rows = read_rows(args.planner_csv)
    planner_numeric_keys = [
        "planning_wall_time", "planned_duration", "trajectory_points", "joint_path_length"
    ]
    planner_summary = summarize(
        planner_rows,
        "planner",
        planner_numeric_keys,
    )
    write_csv(args.planner_summary, planner_summary)
    print_table("Planner summary", planner_summary)

    if planner_rows and "target" in planner_rows[0]:
        planner_target_summary = summarize(
            planner_rows,
            ["planner", "target"],
            planner_numeric_keys,
        )
        write_csv(args.planner_target_summary, planner_target_summary)
        print_table("Planner target summary", planner_target_summary)

    control_rows = read_rows(args.control_csv)
    control_summary = summarize(
        control_rows,
        "strategy",
        [
            "planned_duration",
            "measured_duration",
            "rms_error",
            "max_abs_error",
            "final_abs_error",
            "steady_state_max_error",
            "steady_state_mean_error",
            "max_overshoot",
            "max_overshoot_percent",
            "response_time_90",
            "settling_time",
        ],
    )
    write_csv(args.control_summary, control_summary)
    print_table("Control strategy summary", control_summary)

    print(f"\nSaved planner summary to {args.planner_summary}")
    if planner_rows and "target" in planner_rows[0]:
        print(f"Saved planner target summary to {args.planner_target_summary}")
    print(f"Saved control summary to {args.control_summary}")


if __name__ == "__main__":
    main()
