from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    cartesian_demo = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ur5e_motion_demo",
                executable="cartesian_waypoints_demo",
                output="screen",
                parameters=[robot_description_kinematics, {"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            planning_sim,
            cartesian_demo,
        ]
    )
