from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 顶层入口拉起带 MoveIt 的 Gazebo 仿真，并向下透传图形界面开关。
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    # ur_sim_moveit.launch.py 会同时包含控制仿真和 MoveIt 配置。
    planning_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gazebo"), "/launch", "/ur_sim_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "gazebo_gui": gazebo_gui,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            planning_sim,
        ]
    )
