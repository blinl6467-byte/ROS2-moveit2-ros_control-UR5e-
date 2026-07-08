#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
struct AvoidanceStep
{
  std::string name;
  std::string difficulty;
  std::array<double, 6> joints;
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
  const double velocity_scale = get_or_declare_parameter<double>(node, "velocity_scale", 0.20);
  const double acceleration_scale = get_or_declare_parameter<double>(
    node, "acceleration_scale",
    0.20);

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

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");
  move_group.setPlannerId(planner);
  move_group.setPlanningTime(planning_time);
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(acceleration_scale);

  const std::string frame_id = move_group.getPlanningFrame();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const std::vector<std::string> object_ids = {
    "demo_table",
    "demo_cube_obstacle",
    "demo_target_object",
    "demo_left_wall",
    "demo_right_wall",
    "demo_mid_arm_cube"};
  planning_scene_interface.removeCollisionObjects(object_ids);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  planning_scene_interface.applyCollisionObjects(
  {
    make_box("demo_table", frame_id, 0.45, 0.00, 0.03, {0.90, 0.80, 0.06}),
    make_box("demo_cube_obstacle", frame_id, 0.38, 0.02, 0.32, {0.14, 0.14, 0.30}),
    make_box("demo_target_object", frame_id, 0.54, -0.22, 0.12, {0.08, 0.08, 0.12}),
    make_box("demo_left_wall", frame_id, 0.40, 0.30, 0.35, {0.14, 0.15, 0.48}),
    make_box("demo_right_wall", frame_id, 0.40, -0.30, 0.35, {0.14, 0.15, 0.48}),
    make_box("demo_mid_arm_cube", frame_id, 0.18, 0.18, 0.42, {0.16, 0.16, 0.26})});
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  const std::vector<AvoidanceStep> steps = {
    {"table_reach", "medium", {1.54, -1.62, 1.40, -1.20, -1.60, -0.11}},
    {"narrow_exit", "hard", {2.10, -1.95, 1.18, -0.86, -1.58, -0.70}},
    {"return_table_reach", "medium", {1.54, -1.62, 1.40, -1.20, -1.60, -0.11}}};

  RCLCPP_INFO(logger, "Writing obstacle avoidance demo results to %s", output_file.c_str());
  for (const auto & step : steps) {
    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(std::vector<double>(step.joints.begin(), step.joints.end()));

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto started = std::chrono::steady_clock::now();
    const bool success = static_cast<bool>(move_group.plan(plan));
    const auto finished = std::chrono::steady_clock::now();
    const double wall_time = std::chrono::duration<double>(finished - started).count();
    bool executed = false;
    if (success) {
      executed = static_cast<bool>(move_group.execute(plan));
    }

    const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
    csv << "combined_hard_demo," << step.name << "," << step.difficulty << "," << planner << ","
        << (success ? 1 : 0) << "," << (executed ? 1 : 0) << "," << wall_time << ","
        << (success ? trajectory_duration(joint_trajectory) : 0.0) << ","
        << (success ? joint_trajectory.points.size() : 0) << ","
        << (success ? joint_path_length(joint_trajectory) : 0.0) << "\n";
    csv.flush();

    RCLCPP_INFO(
      logger, "%s: plan %s, execute %s, wall time %.3fs", step.name.c_str(),
      success ? "success" : "failed", executed ? "success" : "failed", wall_time);
  }

  planning_scene_interface.removeCollisionObjects(object_ids);
  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
