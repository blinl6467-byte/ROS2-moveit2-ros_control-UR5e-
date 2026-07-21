#ifndef UR5E_KINEMATICS__UR5E_KINEMATICS_HPP_
#define UR5E_KINEMATICS__UR5E_KINEMATICS_HPP_

#include <array>
#include <cstddef>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ur5e_kinematics
{

/// UR5e 是六自由度串联机械臂，所有关节均为转动关节。
constexpr std::size_t kDof = 6;
/// 关节角向量 q=[q1,...,q6]^T，单位为 rad。
using JointVector = Eigen::Matrix<double, kDof, 1>;
/// 空间几何雅可比：[末端线速度; 末端角速度] = J(q) * q_dot。
using Jacobian = Eigen::Matrix<double, 6, kDof>;
/// 六维位姿误差：[位置误差(m); SO(3)旋转向量误差(rad)]。
using PoseVector = Eigen::Matrix<double, 6, 1>;

/// 阻尼最小二乘逆运动学的可调参数。
struct IkOptions
{
  /// 最大迭代次数；超过后即使残差未收敛也返回。
  std::size_t max_iterations{200};
  /// 平移误差收敛阈值，单位 m。
  double position_tolerance{1e-5};
  /// 旋转向量误差收敛阈值，单位 rad。
  double orientation_tolerance{1e-4};
  /// 远离奇异点时仍保留的基础阻尼 lambda_0。
  double damping{1e-3};
  /// 最小奇异值低于此阈值时自动增大阻尼。
  double singular_value_threshold{5e-2};
  /// 姿态误差相对位置误差的权重，用于处理 m 与 rad 的量纲和任务优先级。
  double orientation_weight{0.35};
  /// 单次关节更新 ||delta_q|| 的上限，避免大步长导致发散。
  double max_step_norm{0.20};
  /// 求得 delta_q 后的整体步长系数，相当于梯度迭代中的学习率。
  double step_scale{1.0};
  /// 是否在每次迭代后将关节角截断到模型关节限位内。
  bool enforce_joint_limits{true};
};

/// 逆运动学求解结果及诊断量。
struct IkResult
{
  /// 求解结束时的关节角；即使失败也返回最后一次迭代结果。
  JointVector joints{JointVector::Zero()};
  /// 位置和姿态误差是否同时满足阈值。
  bool converged{false};
  /// 实际迭代次数。
  std::size_t iterations{0};
  /// 最终位置误差范数，单位 m。
  double position_error{0.0};
  /// 最终姿态旋转向量范数，单位 rad。
  double orientation_error{0.0};
  /// 最后一次迭代中 J 的最小奇异值，用于判断接近奇异位形的程度。
  double minimum_singular_value{0.0};
};

/// 与本工作区 URDF 的 base_link -> tool0 坐标链一致的运动学模型。
///
/// 核心求解不调用 MoveIt、KDL、IKFast 或 Pinocchio。KDL 仅在测试目标中
/// 作为外部参考，用于检查自研 FK 和 Jacobian 是否与 URDF 一致。
class Ur5eKinematics
{
public:
  Ur5eKinematics();

  /// 正运动学：输入六个关节角，返回 ^base_link T_tool0。
  Eigen::Isometry3d forward(const JointVector & joints) const;

  /// 计算在 base_link 中表达、作用于 tool0 原点的 6x6 空间几何雅可比。
  Jacobian geometricJacobian(const JointVector & joints) const;

  /// 使用加权阻尼最小二乘迭代求解目标位姿对应的关节角。
  ///
  /// 数值 IK 是局部方法，结果依赖 seed，不保证对所有可达位姿全局收敛。
  IkResult inverse(
    const Eigen::Isometry3d & target,
    const JointVector & seed,
    const IkOptions & options = IkOptions{}) const;

  /// SO(3) 对数映射：将旋转矩阵转成 axis * angle 旋转向量。
  static Eigen::Vector3d rotationLog(const Eigen::Matrix3d & rotation);

  /// 构造从 current 指向 target、且在基坐标系表达的六维位姿误差。
  static PoseVector poseError(
    const Eigen::Isometry3d & current,
    const Eigen::Isometry3d & target);

  /// UR5e各关节下限，单位rad。
  const JointVector & lowerLimits() const noexcept;
  /// UR5e各关节上限，单位rad。
  const JointVector & upperLimits() const noexcept;

private:
  /// 根据URDF的xyz和固定轴RPY构造齐次变换；旋转顺序为Rz(yaw)Ry(pitch)Rx(roll)。
  static Eigen::Isometry3d makeTransform(
    const Eigen::Vector3d & translation,
    const Eigen::Vector3d & roll_pitch_yaw);
  /// 构造关节局部Z轴旋转Rz(q)。
  static Eigen::Isometry3d rotationAboutZ(double angle);
  /// 按元素将关节向量截断至[lower_limits_, upper_limits_]。
  JointVector clampToLimits(const JointVector & joints) const;

  /// 六个parent-link到joint-frame的q=0固定变换，直接来自URDF运动学参数。
  std::array<Eigen::Isometry3d, kDof> joint_origins_;
  /// base_link -> base_link_inertia固定变换。
  Eigen::Isometry3d base_to_internal_;
  /// wrist_3_link -> flange -> tool0合成固定变换。
  Eigen::Isometry3d wrist_to_tool0_;
  JointVector lower_limits_;
  JointVector upper_limits_;
};

}  // namespace ur5e_kinematics

#endif  // UR5E_KINEMATICS__UR5E_KINEMATICS_HPP_
