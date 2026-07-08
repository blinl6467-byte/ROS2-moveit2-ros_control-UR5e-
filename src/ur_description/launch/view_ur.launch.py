# 版权所有 (c) 2021 PickNik, Inc.
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
#
# 作者：Denis Stogl

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur_description"),
                                "launch",
                                "view_ur.launch.xml",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": LaunchConfiguration("ur_type"),
                }.items(),
            )
        ]
    )
