#include <chrono>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

namespace
{
builtin_interfaces::msg::Duration make_duration_msg(const int64_t nanoseconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return duration;
}

void scale_trajectory_time(
  moveit_msgs::msg::RobotTrajectory & trajectory,
  const double scale)
{
  for (auto & point : trajectory.joint_trajectory.points) {
    const auto scaled_nanoseconds = static_cast<int64_t>(
      static_cast<double>(rclcpp::Duration(point.time_from_start).nanoseconds()) * scale);
    point.time_from_start = make_duration_msg(scaled_nanoseconds);
    for (auto & velocity : point.velocities) {
      velocity /= scale;
    }
    for (auto & acceleration : point.accelerations) {
      acceleration /= scale * scale;
    }
  }
}

visualization_msgs::msg::Marker make_base_marker(
  const std::string & frame_id,
  const std::string & ns,
  const int id,
  const int type)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = make_duration_msg(0);
  return marker;
}

visualization_msgs::msg::MarkerArray make_waypoint_markers(
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  visualization_msgs::msg::MarkerArray markers;

  auto line = make_base_marker(
    frame_id, "cartesian_waypoint_path", 0, visualization_msgs::msg::Marker::LINE_STRIP);
  line.scale.x = 0.01;
  line.color.r = 0.1;
  line.color.g = 0.65;
  line.color.b = 1.0;
  line.color.a = 1.0;

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = waypoints[i].position.x;
    point.y = waypoints[i].position.y;
    point.z = waypoints[i].position.z;
    line.points.push_back(point);

    auto sphere = make_base_marker(
      frame_id, "cartesian_waypoints", static_cast<int>(i),
      visualization_msgs::msg::Marker::SPHERE);
    sphere.pose.position = waypoints[i].position;
    sphere.scale.x = 0.035;
    sphere.scale.y = 0.035;
    sphere.scale.z = 0.035;
    sphere.color.r = 1.0;
    sphere.color.g = 0.55;
    sphere.color.b = 0.05;
    sphere.color.a = 1.0;
    markers.markers.push_back(sphere);

    auto label = make_base_marker(
      frame_id, "cartesian_waypoint_labels", static_cast<int>(i),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    label.pose.position = waypoints[i].position;
    label.pose.position.z += 0.055;
    label.scale.z = 0.04;
    label.color.r = 1.0;
    label.color.g = 1.0;
    label.color.b = 1.0;
    label.color.a = 1.0;
    std::ostringstream text;
    text << "P" << (i + 1);
    label.text = text.str();
    markers.markers.push_back(label);
  }

  markers.markers.push_back(line);
  return markers;
}
}  // namespace

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
  auto marker_publisher =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "cartesian_waypoint_markers", rclcpp::QoS(1).transient_local());

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  move_group_interface.setPlanningTime(8.0);
  move_group_interface.setMaxVelocityScalingFactor(0.08);
  move_group_interface.setMaxAccelerationScalingFactor(0.08);
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

  const auto frame_id = move_group_interface.getPlanningFrame();
  marker_publisher->publish(make_waypoint_markers(frame_id, waypoints));
  rclcpp::sleep_for(std::chrono::milliseconds(500));

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
    scale_trajectory_time(trajectory, 3.5);
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
