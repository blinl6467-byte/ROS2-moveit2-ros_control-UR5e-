#!/usr/bin/env python3
import argparse
import csv
import statistics
from collections import defaultdict


def to_float(row, key):
    return float(row[key]) if row.get(key) else 0.0


def summarize(rows, group_keys):
    grouped = defaultdict(list)
    for row in rows:
        grouped[tuple(row[key] for key in group_keys)].append(row)

    summary = []
    for key, items in sorted(grouped.items()):
        successes = [int(item["success"]) for item in items]
        success_rows = [item for item in items if int(item["success"]) == 1]
        summary.append(
            {
                **dict(zip(group_keys, key)),
                "trials": len(items),
                "success_rate": sum(successes) / len(successes),
                "avg_planning_wall_time": statistics.fmean(
                    to_float(item, "planning_wall_time") for item in items
                ),
                "avg_planned_duration": statistics.fmean(
                    to_float(item, "planned_duration") for item in success_rows
                )
                if success_rows
                else 0.0,
                "avg_joint_path_length": statistics.fmean(
                    to_float(item, "joint_path_length") for item in success_rows
                )
                if success_rows
                else 0.0,
            }
        )
    return summary


def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(
        description="Summarize dynamic obstacle planner benchmark CSVs."
    )
    parser.add_argument("input_csv")
    parser.add_argument(
        "--planner-summary",
        default="/tmp/ur5e_dynamic_planner_summary.csv",
        help="Output CSV grouped by scenario and planner.",
    )
    parser.add_argument(
        "--target-summary",
        default="/tmp/ur5e_dynamic_target_summary.csv",
        help="Output CSV grouped by scenario, planner, and target.",
    )
    parser.add_argument(
        "--difficulty-summary",
        default="/tmp/ur5e_dynamic_difficulty_summary.csv",
        help="Output CSV grouped by scenario, target difficulty, and planner.",
    )
    args = parser.parse_args()

    with open(args.input_csv, newline="") as handle:
        rows = list(csv.DictReader(handle))

    write_csv(args.planner_summary, summarize(rows, ["scenario", "planner"]))
    write_csv(args.target_summary, summarize(rows, ["scenario", "planner", "target"]))
    if rows and "target_difficulty" in rows[0]:
        write_csv(
            args.difficulty_summary,
            summarize(rows, ["scenario", "target_difficulty", "planner"]),
        )
    print(f"Wrote {args.planner_summary}")
    print(f"Wrote {args.target_summary}")
    if rows and "target_difficulty" in rows[0]:
        print(f"Wrote {args.difficulty_summary}")


if __name__ == "__main__":
    main()
