#!/usr/bin/env python
# 版权所有 2024, FZI Forschungszentrum Informatik
#
# 在满足以下条件的前提下，允许以源代码或二进制形式重新分发和使用，
# 无论是否修改：
#
#    * 以源代码形式重新分发时，必须保留上述版权声明、
#      本条件列表以及以下免责声明。
#
#    * 以二进制形式重新分发时，必须在随附文档或其他材料中
#      复制上述版权声明、本条件列表以及以下免责声明。
#      
#
#    * 未经事先书面许可，不得使用 {copyright_holder} 或其
#      贡献者的名称为基于本软件的产品背书或推广。
#      
#
# 本软件由版权持有人和贡献者按“原样”提供，
# 不提供任何明示或暗示的保证，包括但不限于
# 对适销性以及特定用途适用性的暗示保证。
# 在任何情况下，版权持有人或贡献者均不对任何直接、间接、偶发、特殊、
# 惩戒性或后果性损害负责，
# 包括但不限于替代商品或服务采购、
# 使用损失、数据或利润损失、业务中断等，
# 无论损害如何产生，也无论责任依据为合同、严格责任或侵权
# （包括疏忽或其他情形），
# 即使已被告知可能发生此类损害，也不承担责任。
# 

import logging
import os
import pytest
import sys
import time
import unittest


import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
import launch_testing

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.append(os.path.dirname(__file__))
from test_common import ActionInterface, wait_for_controller  # noqa: E402


TIMEOUT_EXECUTE_TRAJECTORY = 30

ROBOT_JOINTS = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# TODO：添加 tf_prefix 参数化支持
@pytest.mark.launch_test
@launch_testing.parametrize(
    "ur_type",
    ["ur5e"],
)
def generate_test_description(ur_type):
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gazebo"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
            "start_joint_controller": "true",
            "launch_rviz": "false",
            "gazebo_gui": "false",
        }.items(),
    )
    return LaunchDescription([ReadyToTest(), gazebo])


class GazeboTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # 初始化 ROS 上下文
        rclpy.init()
        cls.node = Node("ur_gazebo_test")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # 关闭 ROS 上下文
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        wait_for_controller(self.node, "joint_trajectory_controller", 30)
        self._follow_joint_trajectory = ActionInterface(
            self.node,
            "/joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

    def test_trajectory(self, ur_type):
        """测试机器人运动。"""
        # 构造测试轨迹
        test_trajectory = [
            (Duration(sec=6, nanosec=0), [-0.1 for j in ROBOT_JOINTS]),
            (Duration(sec=9, nanosec=0), [-0.5 for j in ROBOT_JOINTS]),
            (Duration(sec=12, nanosec=0), [-1.0 for j in ROBOT_JOINTS]),
        ]

        trajectory = JointTrajectory(
            # joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
            joint_names=ROBOT_JOINTS,
            points=[
                JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
                for (test_time, test_pos) in test_trajectory
            ],
        )

        # 发送轨迹目标
        logging.info("Sending simple goal")
        goal_handle = self._follow_joint_trajectory.send_goal(trajectory=trajectory)
        self.assertTrue(goal_handle.accepted)

        # 验证执行结果
        result = self._follow_joint_trajectory.get_result(goal_handle, TIMEOUT_EXECUTE_TRAJECTORY)
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
