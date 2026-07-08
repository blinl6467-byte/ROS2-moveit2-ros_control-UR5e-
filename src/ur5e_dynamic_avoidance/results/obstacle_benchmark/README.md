# Structured Obstacle Planner Benchmark

Generated on 2026-07-08 with:

```bash
ros2 launch ur5e_dynamic_avoidance obstacle_planner_benchmark.launch.py \
  gazebo_gui:=false trials:=1 planning_time:=1.0 \
  output_file:=/tmp/ur5e_obstacle_planner_benchmark.csv
```

This run covers:

- Scenes: `baseline`, `table_cube_goal`, `narrow_passage`, `mid_arm_obstacle`,
  `combined_hard`.
- Target difficulty groups: `easy`, `medium`, `hard`.
- Planners: `RRTConnectkConfigDefault`, `RRTkConfigDefault`, `RRTstarkConfigDefault`,
  `PRMkConfigDefault`.

Outputs:

- `obstacle_planner_benchmark.csv`: one row per scene, planner, target, and trial.
- `obstacle_planner_summary.csv`: grouped by scene and planner.
- `obstacle_target_summary.csv`: grouped by scene, planner, and target.
- `obstacle_difficulty_summary.csv`: grouped by scene, difficulty, and planner.
- `plots/success_rate_by_scene.png`: grouped bar chart for each planner in each scene.
- `plots/planning_time_by_scene.png`: line chart for planner wall-clock time.
- `plots/joint_path_length_by_scene.png`: line chart for successful joint-space path length.
- `plots/success_rate_by_difficulty.png`: grouped bar chart for easy/medium/hard targets.
- `plots/target_success_heatmap.png`: target-by-scene success-rate heatmap.
- `plots/planner_efficiency_scatter.png`: success rate versus planning time scatter plot.

Regenerate the plots with:

```bash
ros2 run ur5e_dynamic_avoidance plot_obstacle_benchmark.py \
  --planner-summary src/ur5e_dynamic_avoidance/results/obstacle_benchmark/obstacle_planner_summary.csv \
  --target-summary src/ur5e_dynamic_avoidance/results/obstacle_benchmark/obstacle_target_summary.csv \
  --difficulty-summary src/ur5e_dynamic_avoidance/results/obstacle_benchmark/obstacle_difficulty_summary.csv \
  --output-dir src/ur5e_dynamic_avoidance/results/obstacle_benchmark/plots
```

Observed result:

- `baseline` and `table_cube_goal` reached 100% success for all planners.
- `narrow_passage` reached 83.3% success per planner. The hard target
  `hard_high_reach` is deliberately difficult and failed in this scene.
- `mid_arm_obstacle` reached 50% success per planner. Easy targets collide with the
  obstacle near the upper-arm/forearm swept volume, while medium targets remained
  solvable.
- `combined_hard` reached 66.7% success per planner and is the best stress scene for
  explaining how obstacle placement and target difficulty interact.
- `RRTstarkConfigDefault` consistently consumed the 1.0 s planning budget, so it is
  useful for showing optimal/asymptotic planners versus fast feasible planners.
