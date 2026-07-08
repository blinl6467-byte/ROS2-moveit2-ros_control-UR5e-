#include <memory>
#include <chrono>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur5e_cartesian_waypoints_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("ur5e_cartesian_waypoints_demo");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  move_group_interface.setPlanningTime(8.0);
  move_group_interface.setMaxVelocityScalingFactor(0.20);
  move_group_interface.setMaxAccelerationScalingFactor(0.20);
  move_group_interface.setStartStateToCurrentState();

  move_group_interface.setNamedTarget("test_configuration");
  moveit::planning_interface::MoveGroupInterface::Plan preplan;
  const bool preplanned = static_cast<bool>(move_group_interface.plan(preplan));
  if (!preplanned) {
    RCLCPP_ERROR(logger, "Failed to plan to Cartesian demo start configuration.");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }
  const bool premoved = static_cast<bool>(move_group_interface.execute(preplan));
  if (!premoved) {
    RCLCPP_ERROR(logger, "Failed to move to Cartesian demo start configuration.");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  move_group_interface.setStartStateToCurrentState();

  geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;
  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto waypoint = start_pose;
  waypoint.position.z += 0.03;
  waypoints.push_back(waypoint);

  waypoint.position.y -= 0.03;
  waypoints.push_back(waypoint);

  waypoint.position.x += 0.03;
  waypoints.push_back(waypoint);

  waypoint.position.z -= 0.02;
  waypoints.push_back(waypoint);

  waypoint.position.y += 0.03;
  waypoint.position.x -= 0.02;
  waypoints.push_back(waypoint);

  moveit_msgs::msg::RobotTrajectory trajectory;
  constexpr double eef_step = 0.005;
  constexpr double jump_threshold = 0.0;
  RCLCPP_INFO(logger, "Planning Cartesian path through %zu waypoints.", waypoints.size());
  const double fraction = move_group_interface.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "Cartesian path planning fraction: %.2f%%", fraction * 100.0);
  if (fraction < 0.90) {
    RCLCPP_ERROR(logger, "Cartesian path coverage is too low. Aborting execution.");
  } else {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const bool executed = static_cast<bool>(move_group_interface.execute(plan));
    if (!executed) {
      RCLCPP_ERROR(logger, "Cartesian trajectory execution failed.");
    }
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
