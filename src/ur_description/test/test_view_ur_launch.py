# 版权所有 (c) 2022 FZI Forschungszentrum Informatik
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
# 作者：Lukas Sackewitz

import os
import pytest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest


# 执行给定的 launch 文件，并检查所有节点是否能够启动
@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_description"), "launch/view_ur.launch.py")
        ),
        launch_arguments={"ur_type": "ur5e"}.items(),
    )

    return LaunchDescription([launch_include, ReadyToTest()])
