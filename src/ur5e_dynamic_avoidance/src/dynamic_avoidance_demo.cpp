#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
builtin_interfaces::msg::Duration make_duration_msg(const int64_t nanoseconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return duration;
}

struct LabelMarker
{
  std::string text;
  double x;
  double y;
  double z;
};

moveit_msgs::msg::CollisionObject make_box(
  const std::string & id,
  const std::string & frame_id,
  const double x,
  const double y,
  const double z,
  const std::array<double, 3> & size)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame_id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size[0], size[1], size[2]};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  return object;
}

double trajectory_duration(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }
  const auto & duration = trajectory.points.back().time_from_start;
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

double joint_path_length(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  double length = 0.0;
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & previous = trajectory.points[i - 1].positions;
    const auto & current = trajectory.points[i].positions;
    const auto count = std::min(previous.size(), current.size());
    double segment = 0.0;
    for (std::size_t j = 0; j < count; ++j) {
      const double delta = current[j] - previous[j];
      segment += delta * delta;
    }
    length += std::sqrt(segment);
  }
  return length;
}

visualization_msgs::msg::Marker make_text_marker(
  const std::string & frame_id,
  const int id,
  const LabelMarker & label)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = "obstacle_scene_labels";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = label.x;
  marker.pose.position.y = label.y;
  marker.pose.position.z = label.z;
  marker.scale.z = 0.055;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.text = label.text;
  marker.lifetime = make_duration_msg(0);
  return marker;
}

visualization_msgs::msg::Marker make_target_marker(
  const std::string & frame_id,
  const int id,
  const double x,
  const double y,
  const double z)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = "obstacle_demo_goal_markers";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 1.0;
  marker.color.g = 0.7;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = make_duration_msg(0);
  return marker;
}

visualization_msgs::msg::Marker make_line_marker(
  const std::string & frame_id,
  const int id,
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & target_pose)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = "unobstructed_reference_path";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.012;
  marker.color.r = 1.0;
  marker.color.g = 0.25;
  marker.color.b = 0.15;
  marker.color.a = 1.0;
  marker.lifetime = make_duration_msg(0);

  geometry_msgs::msg::Point start_point;
  start_point.x = start_pose.position.x;
  start_point.y = start_pose.position.y;
  start_point.z = start_pose.position.z;
  marker.points.push_back(start_point);

  geometry_msgs::msg::Point target_point;
  target_point.x = target_pose.position.x;
  target_point.y = target_pose.position.y;
  target_point.z = target_pose.position.z;
  marker.points.push_back(target_point);
  return marker;
}

visualization_msgs::msg::Marker make_route_marker(
  const std::string & frame_id,
  const int id,
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = "avoidance_route";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.018;
  marker.color.r = 0.0;
  marker.color.g = 0.9;
  marker.color.b = 0.35;
  marker.color.a = 1.0;
  marker.lifetime = make_duration_msg(0);

  for (const auto & pose : poses) {
    geometry_msgs::msg::Point point;
    point.x = pose.position.x;
    point.y = pose.position.y;
    point.z = pose.position.z;
    marker.points.push_back(point);
  }
  return marker;
}

geometry_msgs::msg::Pose offset_pose(
  const geometry_msgs::msg::Pose & pose,
  const double x,
  const double y,
  const double z)
{
  auto offset = pose;
  offset.position.x += x;
  offset.position.y += y;
  offset.position.z += z;
  return offset;
}

visualization_msgs::msg::MarkerArray make_scene_markers(
  const std::string & frame_id,
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & target_pose,
  const geometry_msgs::msg::Pose & blocker_pose,
  const std::vector<geometry_msgs::msg::Pose> & route_poses)
{
  visualization_msgs::msg::MarkerArray markers;
  const std::vector<LabelMarker> labels = {
    {"table", 0.45, 0.00, 0.13},
    {"front obstacle", blocker_pose.position.x, blocker_pose.position.y,
      blocker_pose.position.z + 0.36},
    {"start pose", start_pose.position.x, start_pose.position.y, start_pose.position.z + 0.08},
    {"target pose", target_pose.position.x, target_pose.position.y, target_pose.position.z + 0.08}};

  for (std::size_t i = 0; i < labels.size(); ++i) {
    markers.markers.push_back(make_text_marker(frame_id, static_cast<int>(i), labels[i]));
  }

  auto start_marker = make_target_marker(
    frame_id, 100, start_pose.position.x, start_pose.position.y, start_pose.position.z);
  start_marker.ns = "obstacle_demo_start_markers";
  start_marker.color.r = 0.1;
  start_marker.color.g = 0.75;
  start_marker.color.b = 1.0;
  markers.markers.push_back(start_marker);
  markers.markers.push_back(
    make_target_marker(
      frame_id, 101, target_pose.position.x, target_pose.position.y, target_pose.position.z));
  markers.markers.push_back(make_line_marker(frame_id, 200, start_pose, target_pose));
  markers.markers.push_back(make_route_marker(frame_id, 201, route_poses));
  return markers;
}

template<typename T>
T get_or_declare_parameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur5e_dynamic_avoidance_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("ur5e_dynamic_avoidance_demo");

  const std::string output_file = get_or_declare_parameter<std::string>(
    node, "output_file", "/tmp/ur5e_dynamic_avoidance_demo.csv");
  const std::string planner = get_or_declare_parameter<std::string>(
    node, "planner", "RRTConnectkConfigDefault");
  const double planning_time = get_or_declare_parameter<double>(node, "planning_time", 5.0);
  const double velocity_scale = get_or_declare_parameter<double>(node, "velocity_scale", 0.06);
  const double acceleration_scale = get_or_declare_parameter<double>(
    node, "acceleration_scale",
    0.06);
  const double max_planned_duration = get_or_declare_parameter<double>(
    node, "max_planned_duration", 12.0);

  std::ofstream csv(output_file, std::ios::out | std::ios::trunc);
  if (!csv.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open output file: %s", output_file.c_str());
    rclcpp::shutdown();
    return 1;
  }
  csv <<
    "scene,step,target_difficulty,planner,success,executed,planning_wall_time,planned_duration,"
    "trajectory_points,joint_path_length\n";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});
  auto marker_publisher =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "obstacle_demo_markers", rclcpp::QoS(1).transient_local());

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");
  move_group.setPlannerId(planner);
  move_group.setPlanningTime(planning_time);
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(acceleration_scale);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.15);
  move_group.setNumPlanningAttempts(10);

  const std::string frame_id = move_group.getPlanningFrame();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const std::vector<std::string> object_ids = {
    "demo_table",
    "demo_straight_line_blocker"};
  planning_scene_interface.removeCollisionObjects(object_ids);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  move_group.setNamedTarget("test_configuration");
  moveit::planning_interface::MoveGroupInterface::Plan start_plan;
  const bool start_planned = static_cast<bool>(move_group.plan(start_plan));
  if (!start_planned || !static_cast<bool>(move_group.execute(start_plan))) {
    RCLCPP_ERROR(logger, "Failed to move to obstacle avoidance start configuration.");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  move_group.setStartStateToCurrentState();
  const auto start_pose = move_group.getCurrentPose().pose;
  auto target_pose = offset_pose(start_pose, 0.18, -0.20, 0.0);
  target_pose.orientation = start_pose.orientation;

  geometry_msgs::msg::Pose blocker_pose;
  blocker_pose.orientation.w = 1.0;
  blocker_pose.position.x = start_pose.position.x + 0.13;
  blocker_pose.position.y = start_pose.position.y - 0.13;
  blocker_pose.position.z = 0.19;

  const double bypass_y = blocker_pose.position.y + 0.28;
  auto bypass_entry_pose = start_pose;
  bypass_entry_pose.position.x = blocker_pose.position.x - 0.08;
  bypass_entry_pose.position.y = bypass_y;
  bypass_entry_pose.position.z += 0.03;
  bypass_entry_pose.orientation = start_pose.orientation;

  auto bypass_exit_pose = target_pose;
  bypass_exit_pose.position.x = blocker_pose.position.x + 0.10;
  bypass_exit_pose.position.y = bypass_y;
  bypass_exit_pose.position.z += 0.03;
  bypass_exit_pose.orientation = start_pose.orientation;

  const std::vector<geometry_msgs::msg::Pose> route_poses = {
    start_pose,
    bypass_entry_pose,
    bypass_exit_pose,
    target_pose};

  planning_scene_interface.applyCollisionObjects(
  {
    make_box("demo_table", frame_id, 0.45, 0.00, 0.03, {0.90, 0.80, 0.06}),
    make_box(
      "demo_straight_line_blocker", frame_id, blocker_pose.position.x, blocker_pose.position.y,
      blocker_pose.position.z, {0.14, 0.14, 0.26})});
  marker_publisher->publish(make_scene_markers(
      frame_id, start_pose, target_pose, blocker_pose, route_poses));
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(logger, "Writing obstacle avoidance demo results to %s", output_file.c_str());

  bool all_segments_succeeded = true;
  bool all_segments_executed = true;
  double total_wall_time = 0.0;
  double total_planned_duration = 0.0;
  std::size_t total_trajectory_points = 0;
  double total_joint_path_length = 0.0;

  const auto plan_and_execute_pose =
    [&](const std::string & step_name, const geometry_msgs::msg::Pose & pose) {
      move_group.clearPoseTargets();
      move_group.setStartStateToCurrentState();
      if (!move_group.setJointValueTarget(pose)) {
        RCLCPP_WARN(logger, "%s: failed to find an IK solution for the pose target.", step_name.c_str());
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto started = std::chrono::steady_clock::now();
      const bool success = static_cast<bool>(move_group.plan(plan));
      const auto finished = std::chrono::steady_clock::now();
      const double wall_time = std::chrono::duration<double>(finished - started).count();
      const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
      const double planned_duration = success ? trajectory_duration(joint_trajectory) : 0.0;
      const double path_length = success ? joint_path_length(joint_trajectory) : 0.0;
      bool executed = false;

      if (success && planned_duration <= max_planned_duration) {
        executed = static_cast<bool>(move_group.execute(plan));
      } else if (success) {
        RCLCPP_WARN(
          logger, "%s: planned duration %.3fs exceeds %.3fs, skipping execution.",
          step_name.c_str(), planned_duration, max_planned_duration);
      }

      csv << "blocked_pose_demo," << step_name << ",hard," << planner << ","
          << (success ? 1 : 0) << "," << (executed ? 1 : 0) << "," << wall_time << ","
          << planned_duration << ","
          << (success ? joint_trajectory.points.size() : 0) << ","
          << path_length << "\n";
      csv.flush();

      RCLCPP_INFO(
        logger, "%s: plan %s, execute %s, duration %.3fs, wall time %.3fs",
        step_name.c_str(), success ? "success" : "failed", executed ? "success" : "failed",
        planned_duration, wall_time);

      all_segments_succeeded = all_segments_succeeded && success;
      all_segments_executed = all_segments_executed && executed;
      total_wall_time += wall_time;
      total_planned_duration += planned_duration;
      total_trajectory_points += success ? joint_trajectory.points.size() : 0;
      total_joint_path_length += path_length;

      rclcpp::sleep_for(std::chrono::milliseconds(300));
      return success && executed;
    };

  plan_and_execute_pose("avoidance_pose_entry", bypass_entry_pose);
  plan_and_execute_pose("avoidance_pose_exit", bypass_exit_pose);
  plan_and_execute_pose("final_pose_behind_obstacle", target_pose);

  csv << "blocked_pose_demo,obvious_pose_avoidance_total,hard," << planner << ","
      << (all_segments_succeeded ? 1 : 0) << "," << (all_segments_executed ? 1 : 0) << ","
      << total_wall_time << "," << total_planned_duration << ","
      << total_trajectory_points << "," << total_joint_path_length << "\n";
  csv.flush();

  RCLCPP_INFO(
    logger, "obvious_pose_avoidance_total: plan %s, execute %s, duration %.3fs",
    all_segments_succeeded ? "success" : "failed",
    all_segments_executed ? "success" : "failed",
    total_planned_duration);

  planning_scene_interface.removeCollisionObjects(object_ids);
  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
