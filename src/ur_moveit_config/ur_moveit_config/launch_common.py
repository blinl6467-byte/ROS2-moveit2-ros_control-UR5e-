#!/usr/bin/env python3

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
# 作者：Lovro Ivanov

import math
import os
import yaml

from ament_index_python.packages import get_package_share_directory


def construct_angle_radians(loader, node):
    """从 yaml 构造弧度值的辅助函数。"""
    # 支持 YAML 中的 !radians 标记，允许配置直接写弧度表达式。
    value = loader.construct_scalar(node)
    try:
        return float(value)
    except SyntaxError:
        raise Exception("invalid expression: %s" % value)


def construct_angle_degrees(loader, node):
    """从 yaml 读取角度值并转换为弧度的辅助函数。"""
    # MoveIt 内部使用弧度，读取 !degrees 时在这里统一转换。
    return math.radians(construct_angle_radians(loader, node))


def load_yaml(package_name, file_path):
    # 通过包名解析 share 目录，保证 launch 文件在安装空间中也能找到配置。
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        # 注册角度构造器，使 joint_limits/kinematics 等配置可读性更好。
        yaml.SafeLoader.add_constructor("!radians", construct_angle_radians)
        yaml.SafeLoader.add_constructor("!degrees", construct_angle_degrees)
    except Exception:
        raise Exception("yaml support not available; install python-yaml")

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # OSError 覆盖 IOError，并在可用平台上覆盖 WindowsError
        return None


def load_yaml_abs(absolute_file_path):

    try:
        # 绝对路径加载同样支持角度标记，避免两种读取路径行为不一致。
        yaml.SafeLoader.add_constructor("!radians", construct_angle_radians)
        yaml.SafeLoader.add_constructor("!degrees", construct_angle_degrees)
    except Exception:
        raise Exception("yaml support not available; install python-yaml")

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # OSError 覆盖 IOError，并在可用平台上覆盖 WindowsError
        return None
