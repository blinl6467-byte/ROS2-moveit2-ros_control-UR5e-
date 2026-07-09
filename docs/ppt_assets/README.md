# PPT Assets

该目录按汇报顺序整理了 PPT 可直接使用的图片素材。

## 推荐使用顺序

| 文件 | 建议页 | 用途 |
| --- | --- | --- |
| `01_readme_overview.png` | 项目总览 | 展示 GitHub README 首页和项目完成度 |
| `02_package_tree.png` | 项目结构 | 展示 ROS 2 workspace/package 组织 |
| `03_package_description.png` | 模块说明 | 说明各功能包职责 |
| `04_ros2_control_config.png` | 控制链路 | 展示 ros2_control / controller 配置 |
| `05_trajectory_recorder_code.png` | 数据记录 | 展示轨迹记录和误差分析的代码依据 |
| `06_joint_space_demo.png` | 基础规划 | 关节空间目标规划执行截图 |
| `07_pose_goal_demo.png` | 末端规划 | 末端位姿目标规划执行截图 |
| `08_cartesian_waypoints_demo.png` | 多路点规划 | 笛卡尔多路点、末端轨迹和执行成功日志 |
| `08b_cartesian_waypoints_rviz_clean.png` | 多路点规划 | 裁剪后的 RViz 干净版，适合直接放 PPT |
| `09_obstacle_avoidance_demo.png` | 避障规划 | 障碍物、目标点、路径标记和执行成功日志 |
| `09b_obstacle_avoidance_rviz_clean.png` | 避障规划 | 裁剪后的 RViz 干净版，适合直接放 PPT |
| `10_tracking_error_focused.png` | 轨迹跟踪 | 六轴关节规划/实际误差曲线 |
| `11_tracking_rms_by_joint.png` | 轨迹跟踪 | 各关节 RMS 误差柱状图 |
| `12_planner_planning_time.png` | Planner 对比 | 不同 OMPL planner 平均规划时间 |
| `13_planner_path_length.png` | Planner 对比 | 不同 OMPL planner 路径长度 |
| `14_control_response_time.png` | 控制策略对比 | 不同速度/加速度缩放策略响应时间 |
| `15_control_tracking_error.png` | 控制策略对比 | 不同缩放策略跟踪误差 |
| `16_control_speed_error_tradeoff.png` | 控制策略对比 | 速度和误差权衡关系 |
| `17_obstacle_success_rate_by_scene.png` | 避障结果 | 不同障碍场景规划成功率 |
| `18_obstacle_target_success_heatmap.png` | 避障结果 | planner/目标组合成功率热力图 |
| `19_obstacle_planning_time_by_scene.png` | 避障结果 | 不同场景规划耗时 |
| `20_live_cartesian_desktop.png` | 多路点规划 | 现场运行截图，包含 RViz 主界面 |
| `20c_live_cartesian_scene_only.png` | 多路点规划 | 现场运行截图裁剪版，只保留 RViz 主视图区 |
| `21_live_obstacle_desktop.png` | 避障规划 | 现场运行截图，包含 RViz 主界面和避障标记 |
| `21c_live_obstacle_scene_only.png` | 避障规划 | 现场运行截图裁剪版，只保留 RViz 主视图区 |

## PPT 中最建议放的 8 张

如果页数有限，优先使用：

1. `01_readme_overview.png`
2. `02_package_tree.png`
3. `20_live_cartesian_desktop.png`
4. `21_live_obstacle_desktop.png`
5. `10_tracking_error_focused.png`
6. `12_planner_planning_time.png`
7. `14_control_response_time.png`
8. `17_obstacle_success_rate_by_scene.png`

## 还可以补充的截图

当前素材已经够做一版完整 PPT。若还想更强，可以补：

- 一张更近距离的笛卡尔多路点截图：让 P1-P5 标签全部出现在机械臂末端附近。
- 一张 Gazebo 单窗口截图：突出 UR5e 在仿真环境中的实体执行效果。
- 一张 MoveIt MotionPlanning 面板截图：展示 planning group、planner、velocity scaling 和 acceleration scaling。
- 一张项目 GitHub 提交记录或仓库首页截图：用于结尾展示项目已整理上传。

这些不是必须项；当前目录中的图已经可以覆盖“项目背景、系统结构、核心 demo、数据分析、实验结果、项目边界”。
