# Media Assets

该目录存放 README 和面试展示用的视频素材。

| 文件 | 内容 |
| --- | --- |
| `joint_space.webm` | 关节空间目标规划 demo |
| `pose_goal.webm` | 末端位姿目标规划 demo |
| `cartesian_waypoints.webm` | 笛卡尔多路点规划 demo，包含 P1-P5 标记和末端轨迹 |
| `obstacle_avoidance.webm` | 结构化避障 demo，包含障碍物、目标点、窄通道和末端轨迹 |

建议面试展示顺序：

1. 先播放 `cartesian_waypoints.webm`，说明 MoveIt 2 笛卡尔路径、多路点标记和 wrist_3_link 轨迹。
2. 再播放 `obstacle_avoidance.webm`，说明 PlanningScene 障碍物建模和结构化避障规划。
3. 如面试官继续追问基础运动，再展示 `joint_space.webm` 和 `pose_goal.webm`。

视频中的 RViz trail 配置只保留 `wrist_3_link`，避免整臂重影影响观察。
