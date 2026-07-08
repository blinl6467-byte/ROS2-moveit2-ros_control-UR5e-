from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    output_file = LaunchConfiguration("output_file")
    planner = LaunchConfiguration("planner")
    planning_time = LaunchConfiguration("planning_time")
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

    demo = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ur5e_dynamic_avoidance",
                executable="obstacle_avoidance_demo",
                output="screen",
                parameters=[
                    robot_description_kinematics,
                    {
                        "use_sim_time": True,
                        "output_file": output_file,
                        "planner": planner,
                        "planning_time": ParameterValue(planning_time, value_type=float),
                    },
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument(
                "output_file", default_value="/tmp/ur5e_obstacle_avoidance_demo.csv"
            ),
            DeclareLaunchArgument("planner", default_value="RRTConnectkConfigDefault"),
            DeclareLaunchArgument("planning_time", default_value="5.0"),
            planning_sim,
            demo,
        ]
    )
