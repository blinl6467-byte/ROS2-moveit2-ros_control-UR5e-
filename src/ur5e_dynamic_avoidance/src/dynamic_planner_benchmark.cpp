#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
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
struct BenchmarkTarget
{
  std::string name;
  std::string difficulty;
  std::array<double, 6> joints;
};

struct ObstacleScenario
{
  std::string name;
  std::string type;
  std::vector<moveit_msgs::msg::CollisionObject> objects;
};

std::vector<std::string> split(const std::string & text, const char delimiter)
{
  std::vector<std::string> values;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, delimiter)) {
    const auto begin = item.find_first_not_of(" \t");
    const auto end = item.find_last_not_of(" \t");
    if (begin != std::string::npos && end != std::string::npos) {
      values.push_back(item.substr(begin, end - begin + 1));
    }
  }
  return values;
}

std::vector<std::string> split_csv(const std::string & text)
{
  return split(text, ',');
}

std::set<std::string> parse_filter(const std::string & text)
{
  const auto values = split_csv(text);
  return std::set<std::string>(values.begin(), values.end());
}

bool passes_filter(const std::string & value, const std::set<std::string> & filter)
{
  return filter.empty() || filter.count(value) > 0;
}

std::vector<BenchmarkTarget> parse_targets(const std::string & text)
{
  std::vector<BenchmarkTarget> targets;
  const auto target_specs = split(text, ';');
  for (std::size_t i = 0; i < target_specs.size(); ++i) {
    std::string name = "target_" + std::to_string(i + 1);
    std::string difficulty = "custom";
    std::string joints_text = target_specs[i];

    const auto fields = split(target_specs[i], ':');
    if (fields.size() == 2) {
      name = fields[0];
      joints_text = fields[1];
    } else if (fields.size() >= 3) {
      name = fields[0];
      difficulty = fields[1];
      joints_text = fields[2];
    }

    const auto joint_values = split_csv(joints_text);
    if (joint_values.size() != 6) {
      throw std::runtime_error(
              "Each target must contain exactly 6 joint values: " + target_specs[i]);
    }

    BenchmarkTarget target;
    target.name = name;
    target.difficulty = difficulty;
    for (std::size_t joint_index = 0; joint_index < target.joints.size(); ++joint_index) {
      target.joints[joint_index] = std::stod(joint_values[joint_index]);
    }
    targets.push_back(target);
  }
  return targets;
}

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

std::vector<ObstacleScenario> make_scenarios(const std::string & frame_id)
{
  return {
    {"baseline", "baseline", {}},
    {"table_cube_goal", "table_object", {
        make_box("table", frame_id, 0.45, 0.00, 0.03, {0.90, 0.80, 0.06}),
        make_box("cube_obstacle", frame_id, 0.38, 0.02, 0.32, {0.14, 0.14, 0.30}),
        make_box("target_object", frame_id, 0.54, -0.22, 0.12, {0.08, 0.08, 0.12})}},
    {"narrow_passage", "narrow_passage", {
        make_box("passage_left_wall", frame_id, 0.40, 0.28, 0.35, {0.14, 0.16, 0.48}),
        make_box("passage_right_wall", frame_id, 0.40, -0.28, 0.35, {0.14, 0.16, 0.48}),
        make_box("passage_backstop", frame_id, 0.64, 0.00, 0.36, {0.08, 0.82, 0.55})}},
    {"mid_arm_obstacle", "mid_arm_proximity", {
        make_box("mid_arm_cube", frame_id, 0.18, 0.18, 0.42, {0.18, 0.18, 0.28}),
        make_box("low_table_reference", frame_id, 0.45, 0.00, 0.03, {0.90, 0.80, 0.06})}},
    {"combined_hard", "combined", {
        make_box("combined_table", frame_id, 0.45, 0.00, 0.03, {0.90, 0.80, 0.06}),
        make_box("combined_left_wall", frame_id, 0.40, 0.30, 0.35, {0.14, 0.15, 0.48}),
        make_box("combined_right_wall", frame_id, 0.40, -0.30, 0.35, {0.14, 0.15, 0.48}),
        make_box("combined_mid_cube", frame_id, 0.18, 0.18, 0.42, {0.16, 0.16, 0.26})}}};
}

std::vector<std::string> scenario_object_ids()
{
  return {
    "table",
    "cube_obstacle",
    "target_object",
    "passage_left_wall",
    "passage_right_wall",
    "passage_backstop",
    "mid_arm_cube",
    "low_table_reference",
    "combined_table",
    "combined_left_wall",
    "combined_right_wall",
    "combined_mid_cube"};
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
    "ur5e_obstacle_planner_benchmark",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("ur5e_obstacle_planner_benchmark");

  const auto planners = split_csv(
    get_or_declare_parameter<std::string>(
      node,
      "planners",
      "RRTConnectkConfigDefault,RRTkConfigDefault,RRTstarkConfigDefault,PRMkConfigDefault"));
  const auto targets = parse_targets(
    get_or_declare_parameter<std::string>(
      node,
      "targets",
      "easy_front:easy:1.35,-1.45,1.45,-1.55,-1.57,0.00;"
      "easy_left:easy:1.10,-1.42,1.70,-1.88,-1.57,0.28;"
      "medium_table_reach:medium:1.54,-1.62,1.40,-1.20,-1.60,-0.11;"
      "medium_cross_body:medium:1.95,-1.72,1.32,-1.05,-1.57,-0.48;"
      "hard_high_reach:hard:0.82,-1.18,1.92,-2.22,-1.52,0.62;"
      "hard_narrow_exit:hard:2.10,-1.95,1.18,-0.86,-1.58,-0.70"));
  const int trials = get_or_declare_parameter<int>(node, "trials", 3);
  const double planning_time = get_or_declare_parameter<double>(node, "planning_time", 4.0);
  const std::string output_file = get_or_declare_parameter<std::string>(
    node, "output_file", "/tmp/ur5e_obstacle_planner_benchmark.csv");
  const auto scenario_filter = parse_filter(
    get_or_declare_parameter<std::string>(node, "scenarios", ""));
  const auto difficulty_filter = parse_filter(
    get_or_declare_parameter<std::string>(node, "target_difficulties", ""));

  std::ofstream csv(output_file, std::ios::out | std::ios::trunc);
  if (!csv.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open output file: %s", output_file.c_str());
    rclcpp::shutdown();
    return 1;
  }
  csv << "scenario,scene_type,target_difficulty,planner,target,trial,success,"
    "planning_wall_time,planned_duration,trajectory_points,joint_path_length\n";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");
  move_group.setPlanningTime(planning_time);
  move_group.setMaxVelocityScalingFactor(0.25);
  move_group.setMaxAccelerationScalingFactor(0.25);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const auto scenarios = make_scenarios(move_group.getPlanningFrame());
  const auto object_ids = scenario_object_ids();

  RCLCPP_INFO(logger, "Writing obstacle planner benchmark results to %s", output_file.c_str());
  for (const auto & scenario : scenarios) {
    if (!passes_filter(scenario.name, scenario_filter)) {
      continue;
    }

    planning_scene_interface.removeCollisionObjects(object_ids);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    if (!scenario.objects.empty()) {
      planning_scene_interface.applyCollisionObjects(scenario.objects);
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    for (const auto & planner : planners) {
      move_group.setPlannerId(planner);
      for (const auto & target : targets) {
        if (!passes_filter(target.difficulty, difficulty_filter)) {
          continue;
        }

        for (int trial = 1; trial <= trials; ++trial) {
          move_group.clearPoseTargets();
          move_group.setStartStateToCurrentState();
          move_group.setJointValueTarget(
            std::vector<double>(
              target.joints.begin(),
              target.joints.end()));

          moveit::planning_interface::MoveGroupInterface::Plan plan;
          const auto started = std::chrono::steady_clock::now();
          const bool success = static_cast<bool>(move_group.plan(plan));
          const auto finished = std::chrono::steady_clock::now();
          const double wall_time = std::chrono::duration<double>(finished - started).count();

          const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
          csv << scenario.name << "," << scenario.type << "," << target.difficulty << ","
              << planner << "," << target.name << "," << trial << ","
              << (success ? 1 : 0) << "," << wall_time << ","
              << (success ? trajectory_duration(joint_trajectory) : 0.0) << ","
              << (success ? joint_trajectory.points.size() : 0) << ","
              << (success ? joint_path_length(joint_trajectory) : 0.0) << "\n";
          csv.flush();

          RCLCPP_INFO(
            logger, "%s/%s %s %s trial %d/%d: %s, wall time %.3fs",
            scenario.name.c_str(), target.difficulty.c_str(), planner.c_str(),
            target.name.c_str(), trial, trials, success ? "success" : "failed", wall_time);
        }
      }
    }
  }

  planning_scene_interface.removeCollisionObjects(object_ids);
  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
