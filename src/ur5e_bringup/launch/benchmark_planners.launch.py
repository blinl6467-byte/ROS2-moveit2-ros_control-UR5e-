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
    planners = LaunchConfiguration("planners")
    targets = LaunchConfiguration("targets")
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

    benchmark = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ur5e_benchmark",
                executable="planner_benchmark",
                output="screen",
                parameters=[
                    robot_description_kinematics,
                    {
                        "use_sim_time": True,
                        "output_file": output_file,
                        "trials": ParameterValue(trials, value_type=int),
                        "planners": planners,
                        "targets": targets,
                        "planning_time": ParameterValue(planning_time, value_type=float),
                    },
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument("output_file", default_value="/tmp/ur5e_planner_benchmark.csv"),
            DeclareLaunchArgument("trials", default_value="5"),
            DeclareLaunchArgument("planning_time", default_value="3.0"),
            DeclareLaunchArgument(
                "planners",
                default_value=(
                    "RRTConnectkConfigDefault,RRTkConfigDefault,"
                    "RRTstarkConfigDefault,PRMkConfigDefault"
                ),
            ),
            DeclareLaunchArgument(
                "targets",
                default_value=(
                    "home_front:1.54,-1.62,1.40,-1.20,-1.60,-0.11;"
                    "reach_left:1.10,-1.35,1.75,-1.95,-1.57,0.35;"
                    "reach_right:2.00,-1.85,1.20,-0.95,-1.57,-0.55"
                ),
            ),
            planning_sim,
            benchmark,
        ]
    )
