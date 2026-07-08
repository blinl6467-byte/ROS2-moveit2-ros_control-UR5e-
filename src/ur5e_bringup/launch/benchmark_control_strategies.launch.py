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
    trials = LaunchConfiguration("trials")
    strategies = LaunchConfiguration("strategies")
    settling_tolerance = LaunchConfiguration("settling_tolerance")
    response_ratio = LaunchConfiguration("response_ratio")
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

    benchmark = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ur5e_benchmark",
                executable="control_strategy_benchmark",
                output="screen",
                parameters=[
                    robot_description_kinematics,
                    {
                        "use_sim_time": True,
                        "output_file": output_file,
                        "trials": ParameterValue(trials, value_type=int),
                        "strategies": strategies,
                        "settling_tolerance": ParameterValue(
                            settling_tolerance, value_type=float
                        ),
                        "response_ratio": ParameterValue(response_ratio, value_type=float),
                    },
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument(
                "output_file", default_value="/tmp/ur5e_control_strategy_benchmark.csv"
            ),
            DeclareLaunchArgument("trials", default_value="3"),
            DeclareLaunchArgument(
                "strategies",
                default_value="slow:0.10:0.10,medium:0.25:0.25,fast:0.50:0.50",
            ),
            DeclareLaunchArgument("settling_tolerance", default_value="0.01"),
            DeclareLaunchArgument("response_ratio", default_value="0.90"),
            planning_sim,
            benchmark,
        ]
    )
