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
    scenarios = LaunchConfiguration("scenarios")
    target_difficulties = LaunchConfiguration("target_difficulties")
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
                package="ur5e_dynamic_avoidance",
                executable="obstacle_planner_benchmark",
                output="screen",
                parameters=[
                    robot_description_kinematics,
                    {
                        "use_sim_time": True,
                        "output_file": output_file,
                        "trials": ParameterValue(trials, value_type=int),
                        "planners": planners,
                        "targets": targets,
                        "scenarios": scenarios,
                        "target_difficulties": target_difficulties,
                        "planning_time": ParameterValue(planning_time, value_type=float),
                    },
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument(
                "output_file", default_value="/tmp/ur5e_obstacle_planner_benchmark.csv"
            ),
            DeclareLaunchArgument("trials", default_value="3"),
            DeclareLaunchArgument("planning_time", default_value="4.0"),
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
                    "easy_front:easy:1.35,-1.45,1.45,-1.55,-1.57,0.00;"
                    "easy_left:easy:1.10,-1.42,1.70,-1.88,-1.57,0.28;"
                    "medium_table_reach:medium:1.54,-1.62,1.40,-1.20,-1.60,-0.11;"
                    "medium_cross_body:medium:1.95,-1.72,1.32,-1.05,-1.57,-0.48;"
                    "hard_high_reach:hard:0.82,-1.18,1.92,-2.22,-1.52,0.62;"
                    "hard_narrow_exit:hard:2.10,-1.95,1.18,-0.86,-1.58,-0.70"
                ),
            ),
            DeclareLaunchArgument("scenarios", default_value=""),
            DeclareLaunchArgument("target_difficulties", default_value=""),
            planning_sim,
            benchmark,
        ]
    )
