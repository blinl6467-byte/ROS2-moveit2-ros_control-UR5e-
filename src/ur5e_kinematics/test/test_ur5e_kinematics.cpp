#include <cmath>
#include <cstddef>
#include <random>

#include <gtest/gtest.h>

#include "ur5e_kinematics/ur5e_kinematics.hpp"

namespace
{
using ur5e_kinematics::JointVector;
using ur5e_kinematics::Ur5eKinematics;

constexpr double kPi = 3.14159265358979323846;

JointVector randomConfiguration(Ur5eKinematics & model, std::mt19937 & generator)
{
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  JointVector joints;
  for (std::size_t i = 0; i < ur5e_kinematics::kDof; ++i) {
    const double lower = 0.7 * model.lowerLimits()[i];
    const double upper = 0.7 * model.upperLimits()[i];
    joints[i] = lower + unit(generator) * (upper - lower);
  }
  return joints;
}

TEST(Ur5eKinematics, ForwardProducesRigidTransforms)
{
  Ur5eKinematics model;
  std::mt19937 generator(42U);
  for (std::size_t sample = 0; sample < 100; ++sample) {
    const Eigen::Isometry3d pose = model.forward(randomConfiguration(model, generator));
    EXPECT_NEAR(pose.linear().determinant(), 1.0, 1e-12);
    EXPECT_LT(
      (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm(),
      1e-12);
  }
}

TEST(Ur5eKinematics, ZeroConfigurationMatchesUrdfChainReference)
{
  Ur5eKinematics model;
  const Eigen::Isometry3d pose = model.forward(JointVector::Zero());

  // Independently calculated from the fixed transforms in the UR5e Xacro.
  const Eigen::Vector3d expected_position(0.8172, 0.2329, 0.0628);
  EXPECT_LT((pose.translation() - expected_position).norm(), 1e-10);
  EXPECT_NEAR(pose.linear().determinant(), 1.0, 1e-12);
}

TEST(Ur5eKinematics, RotationLogHandlesSmallAndLargeAngles)
{
  EXPECT_LT(Ur5eKinematics::rotationLog(Eigen::Matrix3d::Identity()).norm(), 1e-12);

  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
  for (const double angle : {1e-7, 0.4, kPi - 1e-7}) {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    const Eigen::Vector3d recovered = Ur5eKinematics::rotationLog(rotation);
    EXPECT_NEAR(recovered.norm(), angle, 1e-8);
    EXPECT_GT(recovered.normalized().dot(axis), 1.0 - 1e-8);
  }
}

TEST(Ur5eKinematics, GeometricJacobianMatchesFiniteDifference)
{
  Ur5eKinematics model;
  std::mt19937 generator(84U);
  constexpr double epsilon = 1e-7;

  for (std::size_t sample = 0; sample < 30; ++sample) {
    const JointVector joints = randomConfiguration(model, generator);
    const auto analytical = model.geometricJacobian(joints);

    for (std::size_t column = 0; column < ur5e_kinematics::kDof; ++column) {
      JointVector plus = joints;
      JointVector minus = joints;
      plus[column] += epsilon;
      minus[column] -= epsilon;
      const Eigen::Isometry3d pose_plus = model.forward(plus);
      const Eigen::Isometry3d pose_minus = model.forward(minus);

      const Eigen::Vector3d numerical_linear =
        (pose_plus.translation() - pose_minus.translation()) / (2.0 * epsilon);
      const Eigen::Vector3d numerical_angular = Ur5eKinematics::rotationLog(
        pose_plus.linear() * pose_minus.linear().transpose()) / (2.0 * epsilon);

      EXPECT_LT((analytical.block<3, 1>(0, column) - numerical_linear).norm(), 2e-7);
      EXPECT_LT((analytical.block<3, 1>(3, column) - numerical_angular).norm(), 2e-7);
    }
  }
}

TEST(Ur5eKinematics, InverseRecoversNearbyReachablePoses)
{
  Ur5eKinematics model;
  std::mt19937 generator(126U);
  std::normal_distribution<double> noise(0.0, 0.15);
  std::size_t converged = 0;

  for (std::size_t sample = 0; sample < 50; ++sample) {
    const JointVector target_joints = randomConfiguration(model, generator);
    JointVector seed = target_joints;
    for (std::size_t i = 0; i < ur5e_kinematics::kDof; ++i) {
      seed[i] += noise(generator);
    }

    const auto result = model.inverse(model.forward(target_joints), seed);
    if (result.converged) {
      ++converged;
      EXPECT_LT(result.position_error, 1e-5);
      EXPECT_LT(result.orientation_error, 1e-4);
      const auto final_error = Ur5eKinematics::poseError(
        model.forward(result.joints), model.forward(target_joints));
      EXPECT_LT(final_error.head<3>().norm(), 1e-5);
      EXPECT_LT(final_error.tail<3>().norm(), 1e-4);
    }
  }

  EXPECT_GE(converged, 48U);
}

TEST(Ur5eKinematics, InverseRespectsJointLimits)
{
  Ur5eKinematics model;
  JointVector outside;
  outside.setConstant(100.0);
  const auto target = model.forward(JointVector::Zero());
  const auto result = model.inverse(target, outside);
  EXPECT_TRUE((result.joints.array() <= model.upperLimits().array()).all());
  EXPECT_TRUE((result.joints.array() >= model.lowerLimits().array()).all());
}

}  // namespace
