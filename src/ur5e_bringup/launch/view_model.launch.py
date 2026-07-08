from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 仅启动模型可视化，用于快速检查 URDF/Xacro 展开后的机器人外观和 TF。
    view_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_description"), "/launch", "/view_ur.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur5e",
        }.items(),
    )

    return LaunchDescription([view_model])
