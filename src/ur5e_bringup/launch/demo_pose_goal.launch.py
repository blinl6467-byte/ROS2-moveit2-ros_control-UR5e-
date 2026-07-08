from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 示例节点依赖完整的规划仿真环境，因此先包含 MoveIt+Gazebo 启动文件。
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    planning_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gazebo"), "/launch", "/ur_sim_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "gazebo_gui": gazebo_gui,
        }.items(),
    )

    # 适当延迟执行，给 Gazebo 控制器和 move_group 留出初始化时间。
    pose_goal_demo = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ur5e_motion_demo",
                executable="pose_goal_demo",
                output="screen",
                parameters=[robot_description_kinematics, {"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            planning_sim,
            pose_goal_demo,
        ]
    )
