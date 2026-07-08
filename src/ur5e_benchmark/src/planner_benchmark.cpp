#include <chrono>
#include <cmath>
#include <fstream>
#include <array>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

namespace
{
struct BenchmarkTarget
{
  std::string name;
  std::array<double, 6> joints;
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

std::vector<BenchmarkTarget> parse_targets(const std::string & text)
{
  std::vector<BenchmarkTarget> targets;
  const auto target_specs = split(text, ';');
  for (std::size_t i = 0; i < target_specs.size(); ++i) {
    std::string name = "target_" + std::to_string(i + 1);
    std::string joints_text = target_specs[i];

    const auto separator = target_specs[i].find(':');
    if (separator != std::string::npos) {
      name = target_specs[i].substr(0, separator);
      joints_text = target_specs[i].substr(separator + 1);
    }

    const auto joint_values = split_csv(joints_text);
    if (joint_values.size() != 6) {
      throw std::runtime_error(
              "Each target must contain exactly 6 joint values: " + target_specs[i]);
    }

    BenchmarkTarget target;
    target.name = name;
    for (std::size_t joint_index = 0; joint_index < target.joints.size(); ++joint_index) {
      target.joints[joint_index] = std::stod(joint_values[joint_index]);
    }
    targets.push_back(target);
  }
  return targets;
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
    "ur5e_planner_benchmark",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("ur5e_planner_benchmark");

  const auto planners = split_csv(
    get_or_declare_parameter<std::string>(
      node,
      "planners",
      "RRTConnectkConfigDefault,RRTkConfigDefault,RRTstarkConfigDefault,PRMkConfigDefault"
  ));
  const auto targets = parse_targets(
    get_or_declare_parameter<std::string>(
      node,
      "targets",
      "home_front:1.54,-1.62,1.40,-1.20,-1.60,-0.11;"
      "reach_left:1.10,-1.35,1.75,-1.95,-1.57,0.35;"
      "reach_right:2.00,-1.85,1.20,-0.95,-1.57,-0.55"
  ));
  const int trials = get_or_declare_parameter<int>(node, "trials", 5);
  const double planning_time = get_or_declare_parameter<double>(node, "planning_time", 3.0);
  const std::string output_file = get_or_declare_parameter<std::string>(
    node,
    "output_file", "/tmp/ur5e_planner_benchmark.csv");

  std::ofstream csv(output_file, std::ios::out | std::ios::trunc);
  if (!csv.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open output file: %s", output_file.c_str());
    rclcpp::shutdown();
    return 1;
  }
  csv << "planner,target,trial,success,planning_wall_time,planned_duration,"
    "trajectory_points,joint_path_length\n";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");
  move_group.setPlanningTime(planning_time);
  move_group.setMaxVelocityScalingFactor(0.25);
  move_group.setMaxAccelerationScalingFactor(0.25);

  RCLCPP_INFO(logger, "Writing planner benchmark results to %s", output_file.c_str());
  for (const auto & planner : planners) {
    move_group.setPlannerId(planner);
    for (const auto & target : targets) {
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
        const double wall_time =
          std::chrono::duration<double>(finished - started).count();

        const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
        csv << planner << "," << target.name << "," << trial << "," << (success ? 1 : 0) << ","
            << wall_time << ","
            << (success ? trajectory_duration(joint_trajectory) : 0.0) << ","
            << (success ? joint_trajectory.points.size() : 0) << ","
            << (success ? joint_path_length(joint_trajectory) : 0.0) << "\n";
        csv.flush();

        RCLCPP_INFO(
          logger,
          "%s %s trial %d/%d: %s, plan wall time %.3fs",
          planner.c_str(), target.name.c_str(), trial, trials,
          success ? "success" : "failed", wall_time);
      }
    }
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
