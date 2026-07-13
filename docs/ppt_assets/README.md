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
| `06_tracking_error_focused.png` | 轨迹跟踪 | 六轴关节规划/实际误差曲线 |
| `07_tracking_rms_by_joint.png` | 轨迹跟踪 | 各关节 RMS 误差柱状图 |
| `08_planner_planning_time.png` | Planner 对比 | 不同 OMPL planner 平均规划时间 |
| `09_planner_path_length.png` | Planner 对比 | 不同 OMPL planner 路径长度 |
| `10_control_response_time.png` | 控制策略对比 | 不同速度/加速度缩放策略响应时间 |
| `11_control_tracking_error.png` | 控制策略对比 | 不同缩放策略跟踪误差 |
| `12_control_speed_error_tradeoff.png` | 控制策略对比 | 速度和误差权衡关系 |
| `13_obstacle_success_rate_by_scene.png` | 避障结果 | 不同障碍场景规划成功率 |
| `14_obstacle_target_success_heatmap.png` | 避障结果 | planner/目标组合成功率热力图 |
| `15_obstacle_planning_time_by_scene.png` | 避障结果 | 不同场景规划耗时 |

## PPT 中最建议放的 8 张

如果页数有限，优先使用：

1. `01_readme_overview.png`
2. `02_package_tree.png`
3. `04_ros2_control_config.png`
4. `06_tracking_error_focused.png`
5. `08_planner_planning_time.png`
6. `10_control_response_time.png`
7. `13_obstacle_success_rate_by_scene.png`
8. `14_obstacle_target_success_heatmap.png`

## 还可以补充的截图

当前素材已经够做一版完整 PPT。若还想更强，可以补：

- 一张更近距离的笛卡尔多路点截图：让 P1-P5 标签全部出现在机械臂末端附近。
- 一张 Gazebo 单窗口截图：突出 UR5e 在仿真环境中的实体执行效果。
- 一张 MoveIt MotionPlanning 面板截图：展示 planning group、planner、velocity scaling 和 acceleration scaling。
- 一张项目 GitHub 提交记录或仓库首页截图：用于结尾展示项目已整理上传。

这些不是必须项；当前目录中的图已经可以覆盖“项目背景、系统结构、核心 demo、数据分析、实验结果、项目边界”。
