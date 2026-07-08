from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 透传 Gazebo 与 RViz 开关，便于从 bringup 包统一控制仿真界面。
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # 复用 ur_simulation_gazebo 中的底层仿真启动文件，并固定机器人型号为 UR5e。
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gazebo"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "gazebo_gui": gazebo_gui,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            simulation,
        ]
    )
