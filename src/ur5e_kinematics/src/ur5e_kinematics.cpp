#include "ur5e_kinematics/ur5e_kinematics.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/SVD>

namespace ur5e_kinematics
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSmallAngle = 1e-9;
}  // namespace

Ur5eKinematics::Ur5eKinematics()
{
  // 这里没有直接套用网上常见的UR5e标准DH表，而是使用当前工程Xacro实际加载的
  // default_kinematics.yaml。原因是：同一台机器人可以选择不同的base/tool坐标系，
  // 标准DH结果往往位于厂家Base/Flange坐标系，而本项目需要base_link -> tool0。
  //
  // joint_origins_[i]表示第i个关节在q_i=0时从父连杆到关节坐标系的固定变换：
  //   ^parent T_joint_origin
  // 每个关节在自己的局部+Z轴旋转，所以完整关节变换为：
  //   ^parent T_child(q_i) = joint_origin_i * Rz(q_i)
  joint_origins_[0] = makeTransform({0.0, 0.0, 0.1625}, {0.0, 0.0, 0.0});
  joint_origins_[1] = makeTransform({0.0, 0.0, 0.0}, {kPi / 2.0, 0.0, 0.0});
  joint_origins_[2] = makeTransform({-0.425, 0.0, 0.0}, {0.0, 0.0, 0.0});
  joint_origins_[3] = makeTransform({-0.3922, 0.0, 0.1333}, {0.0, 0.0, 0.0});
  joint_origins_[4] = makeTransform({0.0, -0.0997, 0.0}, {kPi / 2.0, 0.0, 0.0});
  joint_origins_[5] = makeTransform(
    {0.0, 0.0996, 0.0}, {kPi / 2.0, kPi, kPi});

  // URDF为了同时满足REP-103和UR控制器坐标系约定，在base_link与真正的串联链之间
  // 插入了绕Z轴pi的固定变换。末端也存在wrist_3_link -> flange -> tool0两段固定
  // 旋转。忽略这些固定变换会导致位置可能相近、姿态和轴方向却与TF不一致。
  base_to_internal_ = makeTransform({0.0, 0.0, 0.0}, {0.0, 0.0, kPi});
  wrist_to_tool0_ =
    makeTransform({0.0, 0.0, 0.0}, {0.0, -kPi / 2.0, -kPi / 2.0}) *
    makeTransform({0.0, 0.0, 0.0}, {kPi / 2.0, 0.0, kPi / 2.0});

  lower_limits_ << -2.0 * kPi, -2.0 * kPi, -kPi, -2.0 * kPi, -2.0 * kPi,
    -2.0 * kPi;
  upper_limits_ << 2.0 * kPi, 2.0 * kPi, kPi, 2.0 * kPi, 2.0 * kPi,
    2.0 * kPi;
}

Eigen::Isometry3d Ur5eKinematics::forward(const JointVector & joints) const
{
  // 齐次变换从左向右累乘：当前transform始终表示base_link到当前坐标系的变换。
  //
  // ^0 T_6(q) = ^0 T_internal * Π_i(origin_i * Rz(q_i)) * ^wrist T_tool0
  //
  // Eigen::Isometry3d内部存储R和p，可表示：p_base = R * p_local + translation。
  Eigen::Isometry3d transform = base_to_internal_;
  for (std::size_t i = 0; i < kDof; ++i) {
    transform = transform * joint_origins_[i] * rotationAboutZ(joints[i]);
  }
  return transform * wrist_to_tool0_;
}

Jacobian Ur5eKinematics::geometricJacobian(const JointVector & joints) const
{
  // 对空间中任意转动关节i，如果其轴方向为z_i、轴上点为p_i，末端原点为p_e：
  //   v_e = z_i x (p_e - p_i) * q_dot_i
  //   w_e = z_i * q_dot_i
  // 因此雅可比第i列为 [z_i x (p_e-p_i); z_i]。
  //
  // 关键是z_i和p_i必须统一表达在base_link中。故先沿链记录每个关节在施加
  // Rz(q_i)之前的原点和Z轴，再得到最终tool0原点。
  std::array<Eigen::Vector3d, kDof> joint_positions;
  std::array<Eigen::Vector3d, kDof> joint_axes;

  Eigen::Isometry3d transform = base_to_internal_;
  for (std::size_t i = 0; i < kDof; ++i) {
    transform = transform * joint_origins_[i];
    joint_positions[i] = transform.translation();
    // 局部关节轴为[0,0,1]^T，左乘当前旋转矩阵后得到基坐标系表达。
    joint_axes[i] = transform.linear() * Eigen::Vector3d::UnitZ();
    transform = transform * rotationAboutZ(joints[i]);
  }
  const Eigen::Vector3d tool_position = (transform * wrist_to_tool0_).translation();

  Jacobian jacobian;
  for (std::size_t i = 0; i < kDof; ++i) {
    jacobian.template block<3, 1>(0, i) =
      joint_axes[i].cross(tool_position - joint_positions[i]);
    jacobian.template block<3, 1>(3, i) = joint_axes[i];
  }
  return jacobian;
}

IkResult Ur5eKinematics::inverse(
  const Eigen::Isometry3d & target,
  const JointVector & seed,
  const IkOptions & options) const
{
  IkResult result;
  // IK是局部迭代算法。seed越接近目标解，越容易收敛到期望构型；不同seed可能得到
  // 同一末端位姿的不同关节解。启用限位时，先保证初值本身合法。
  result.joints = options.enforce_joint_limits ? clampToLimits(seed) : seed;

  for (std::size_t iteration = 0; iteration <= options.max_iterations; ++iteration) {
    const Eigen::Isometry3d current = forward(result.joints);
    // e=[p_target-p_current; Log(R_target*R_current^T)]，均在base_link表达。
    PoseVector error = poseError(current, target);
    result.position_error = error.head<3>().norm();
    result.orientation_error = error.tail<3>().norm();
    result.iterations = iteration;

    const Jacobian jacobian = geometricJacobian(result.joints);
    // 奇异值分解J=UΣV^T。最小奇异值接近0意味着某个笛卡尔方向几乎无法通过
    // 有限关节速度产生，普通伪逆会出现非常大的关节增量。
    const Eigen::JacobiSVD<Jacobian> svd(jacobian);
    result.minimum_singular_value = svd.singularValues().minCoeff();

    if (result.position_error <= options.position_tolerance &&
      result.orientation_error <= options.orientation_tolerance)
    {
      result.converged = true;
      return result;
    }
    if (iteration == options.max_iterations) {
      break;
    }

    // 位置误差单位是m，姿态误差单位是rad，不能直接认为两者具有相同的重要性。
    // 对误差和J的后三行同时乘权重W，等价于求解min ||W(J*dq-e)||^2。
    Jacobian weighted_jacobian = jacobian;
    weighted_jacobian.bottomRows<3>() *= options.orientation_weight;
    error.tail<3>() *= options.orientation_weight;

    // 自适应阻尼：远离奇异点时lambda约等于基础阻尼；sigma_min低于阈值后，
    // 按差值平方增加lambda。这样牺牲少量末端跟踪精度，换取有限、平滑的dq。
    const double singularity_gap = std::max(
      0.0, options.singular_value_threshold - result.minimum_singular_value);
    const double damping = options.damping + singularity_gap * singularity_gap;

    // 阻尼最小二乘解：
    //   dq = J^T (J J^T + lambda^2 I)^-1 e
    // 它等价于最小化 ||J*dq-e||^2 + lambda^2||dq||^2。
    // 使用LDLT解线性方程，不显式求逆，数值稳定性和计算效率更好。
    const Eigen::Matrix<double, 6, 6> regularized =
      weighted_jacobian * weighted_jacobian.transpose() +
      damping * damping * Eigen::Matrix<double, 6, 6>::Identity();
    JointVector delta = weighted_jacobian.transpose() * regularized.ldlt().solve(error);

    // 即使DLS已经抑制奇异点放大，远初值或强非线性区域仍可能产生过大更新。
    // 对整体2范数限幅保留方向，只缩短步长，降低跨越关节限位或迭代振荡风险。
    const double delta_norm = delta.norm();
    if (delta_norm > options.max_step_norm && delta_norm > kSmallAngle) {
      delta *= options.max_step_norm / delta_norm;
    }
    result.joints += options.step_scale * delta;
    if (options.enforce_joint_limits) {
      result.joints = clampToLimits(result.joints);
    }
  }

  return result;
}

Eigen::Vector3d Ur5eKinematics::rotationLog(const Eigen::Matrix3d & rotation)
{
  // SO(3)对数映射把旋转矩阵R转换为旋转向量phi=axis*angle。
  // 使用单位四元数实现，比直接用acos((trace(R)-1)/2)在0和pi附近更稳健。
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  // q和-q表示同一个旋转。统一选择w>=0的半球，使返回角落在[0,pi]，避免
  // 迭代时因四元数符号跳变突然走长旋转路径。
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }

  const Eigen::Vector3d vector_part = quaternion.vec();
  const double vector_norm = vector_part.norm();
  if (vector_norm < kSmallAngle) {
    // 小角度时sin(theta/2)≈theta/2，因此axis*theta≈2*q_xyz。
    return 2.0 * vector_part;
  }

  // q=[cos(theta/2), axis*sin(theta/2)]。
  const double angle = 2.0 * std::atan2(vector_norm, quaternion.w());
  return angle * vector_part / vector_norm;
}

PoseVector Ur5eKinematics::poseError(
  const Eigen::Isometry3d & current,
  const Eigen::Isometry3d & target)
{
  PoseVector error;
  error.head<3>() = target.translation() - current.translation();
  // R_error=R_target*R_current^T是在空间/基坐标系表达的左乘旋转误差，和本类
  // 返回的空间几何雅可比角速度部分保持同一表达坐标系。
  error.tail<3>() = rotationLog(target.linear() * current.linear().transpose());
  return error;
}

const JointVector & Ur5eKinematics::lowerLimits() const noexcept
{
  return lower_limits_;
}

const JointVector & Ur5eKinematics::upperLimits() const noexcept
{
  return upper_limits_;
}

Eigen::Isometry3d Ur5eKinematics::makeTransform(
  const Eigen::Vector3d & translation,
  const Eigen::Vector3d & roll_pitch_yaw)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  // URDF rpy="roll pitch yaw"采用固定轴RPY约定，其旋转矩阵为Rz(yaw)Ry(pitch)Rx(roll)。
  transform.linear() =
    (Eigen::AngleAxisd(roll_pitch_yaw.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(roll_pitch_yaw.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll_pitch_yaw.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  transform.translation() = translation;
  return transform;
}

Eigen::Isometry3d Ur5eKinematics::rotationAboutZ(double angle)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return transform;
}

JointVector Ur5eKinematics::clampToLimits(const JointVector & joints) const
{
  // 先与下限逐元素取max，再与上限逐元素取min。
  return joints.cwiseMax(lower_limits_).cwiseMin(upper_limits_);
}

}  // namespace ur5e_kinematics
