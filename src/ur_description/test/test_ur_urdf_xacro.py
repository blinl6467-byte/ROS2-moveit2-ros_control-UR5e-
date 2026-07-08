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
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


@pytest.mark.parametrize(
    "ur_type",
    ["ur5e"],
)
@pytest.mark.parametrize("description_file", ["ur.urdf.xacro"])
@pytest.mark.parametrize("prefix", ["", "my_ur_"])
def test_ur_urdf_xacro(ur_type, description_file, prefix):
    # 初始化启动参数
    safety_limits = "true"
    safety_pos_margin = "0.15"
    safety_k_position = "20"
    # 通用参数
    description_package = "ur_description"

    joint_limit_params = os.path.join(
        get_package_share_directory(description_package), "config", ur_type, "joint_limits.yaml"
    )
    kinematics_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "default_kinematics.yaml",
    )
    physical_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "physical_parameters.yaml",
    )
    visual_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "visual_parameters.yaml",
    )

    description_file_path = os.path.join(
        get_package_share_directory(description_package), "urdf", description_file
    )

    _, tmp_urdf_output_file = tempfile.mkstemp(suffix=".urdf")

    # 组合 `xacro` 与 `check_urdf` 命令
    xacro_command = (
        f"{shutil.which('xacro')}"
        f" {description_file_path}"
        f" joint_limit_params:={joint_limit_params}"
        f" kinematics_params:={kinematics_params}"
        f" physical_params:={physical_params}"
        f" visual_params:={visual_params}"
        f" safety_limits:={safety_limits}"
        f" safety_pos_margin:={safety_pos_margin}"
        f" safety_k_position:={safety_k_position}"
        f" name:={ur_type}"
        f" prefix:={prefix}"
        f" > {tmp_urdf_output_file}"
    )
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"

    # 尝试调用外部进程，并在结束时删除临时文件
    try:
        xacro_process = subprocess.run(
            xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert xacro_process.returncode == 0, " --- XACRO command failed ---"

        check_urdf_process = subprocess.run(
            check_urdf_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert (
            check_urdf_process.returncode == 0
        ), "\n --- URDF check failed! --- \nYour xacro does not unfold into a proper urdf robot description. Please check your xacro file."

    finally:
        os.remove(tmp_urdf_output_file)


if __name__ == "__main__":
    test_ur_urdf_xacro()
