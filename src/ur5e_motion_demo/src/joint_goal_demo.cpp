#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur5e_joint_goal_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("ur5e_joint_goal_demo");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  move_group_interface.setPlanningTime(5.0);
  move_group_interface.setMaxVelocityScalingFactor(0.25);
  move_group_interface.setMaxAccelerationScalingFactor(0.25);
  move_group_interface.setStartStateToCurrentState();

  const std::vector<double> target_joints = {
    -1.20, -1.35, -1.85, -1.45, 1.57, 0.0
  };
  move_group_interface.setJointValueTarget(target_joints);

  RCLCPP_INFO(logger, "Planning joint-space goal for UR5e.");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = static_cast<bool>(move_group_interface.plan(plan));

  if (planned) {
    RCLCPP_INFO(logger, "Planning succeeded. Executing trajectory.");
    const bool executed = static_cast<bool>(move_group_interface.execute(plan));
    if (!executed) {
      RCLCPP_ERROR(logger, "Trajectory execution failed.");
    }
  } else {
    RCLCPP_ERROR(logger, "Joint-space planning failed.");
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
