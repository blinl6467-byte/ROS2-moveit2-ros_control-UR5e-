#include <cmath>
#include <cstddef>
#include <random>

#include <gtest/gtest.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "ur5e_kinematics/ur5e_kinematics.hpp"

#ifndef TEST_URDF_PATH
#error "TEST_URDF_PATH must point to the generated UR5e URDF"
#endif

namespace
{
using ur5e_kinematics::JointVector;
using ur5e_kinematics::Ur5eKinematics;

Eigen::Isometry3d toEigen(const KDL::Frame & frame)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      transform.linear()(row, column) = frame.M(row, column);
    }
  }
  transform.translation() << frame.p.x(), frame.p.y(), frame.p.z();
  return transform;
}

class KdlReferenceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(kdl_parser::treeFromFile(TEST_URDF_PATH, tree_));
    ASSERT_TRUE(tree_.getChain("base_link", "tool0", chain_));
    ASSERT_EQ(chain_.getNrOfJoints(), ur5e_kinematics::kDof);
  }

  KDL::Tree tree_;
  KDL::Chain chain_;
};

TEST_F(KdlReferenceTest, ForwardKinematicsMatchesGeneratedUrdf)
{
  Ur5eKinematics model;
  KDL::ChainFkSolverPos_recursive kdl_solver(chain_);
  KDL::JntArray kdl_joints(ur5e_kinematics::kDof);
  std::mt19937 generator(2026U);
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  for (std::size_t sample = 0; sample < 200; ++sample) {
    JointVector joints;
    for (std::size_t i = 0; i < ur5e_kinematics::kDof; ++i) {
      joints[i] = 0.65 * model.lowerLimits()[i] + unit(generator) *
        0.65 * (model.upperLimits()[i] - model.lowerLimits()[i]);
      kdl_joints(i) = joints[i];
    }

    KDL::Frame kdl_pose;
    ASSERT_GE(kdl_solver.JntToCart(kdl_joints, kdl_pose), 0);
    const Eigen::Isometry3d reference = toEigen(kdl_pose);
    const Eigen::Isometry3d actual = model.forward(joints);

    EXPECT_LT((actual.translation() - reference.translation()).norm(), 1e-9);
    EXPECT_LT(
      Ur5eKinematics::rotationLog(actual.linear() * reference.linear().transpose()).norm(),
      1e-9);
  }
}

TEST_F(KdlReferenceTest, GeometricJacobianMatchesGeneratedUrdf)
{
  Ur5eKinematics model;
  KDL::ChainJntToJacSolver kdl_solver(chain_);
  KDL::JntArray kdl_joints(ur5e_kinematics::kDof);
  std::mt19937 generator(2027U);
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  for (std::size_t sample = 0; sample < 100; ++sample) {
    JointVector joints;
    for (std::size_t i = 0; i < ur5e_kinematics::kDof; ++i) {
      joints[i] = 0.65 * model.lowerLimits()[i] + unit(generator) *
        0.65 * (model.upperLimits()[i] - model.lowerLimits()[i]);
      kdl_joints(i) = joints[i];
    }

    KDL::Jacobian kdl_jacobian(ur5e_kinematics::kDof);
    ASSERT_GE(kdl_solver.JntToJac(kdl_joints, kdl_jacobian), 0);
    const auto actual = model.geometricJacobian(joints);
    for (std::size_t row = 0; row < 6; ++row) {
      for (std::size_t column = 0; column < ur5e_kinematics::kDof; ++column) {
        EXPECT_NEAR(actual(row, column), kdl_jacobian(row, column), 1e-9);
      }
    }
  }
}

}  // namespace
