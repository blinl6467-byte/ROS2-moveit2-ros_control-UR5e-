#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

struct Strategy
{
  std::string name;
  double velocity_scale;
  double acceleration_scale;
};

struct TrackingStats
{
  std::size_t samples{0};
  double sum_squared_error{0.0};
  double max_abs_error{0.0};
  double final_abs_error{0.0};
  double steady_state_max_error{0.0};
  double steady_state_mean_error{0.0};
  double max_overshoot{0.0};
  double max_overshoot_percent{0.0};
  double response_time_90{0.0};
  double settling_time{0.0};
  double first_stamp{-1.0};
  double last_stamp{-1.0};
};

struct ControllerSample
{
  double stamp{0.0};
  std::vector<double> desired;
  std::vector<double> actual;
};

namespace
{
double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::vector<Strategy> parse_strategies(const std::string & text)
{
  std::vector<Strategy> strategies;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    std::stringstream item_stream(item);
    std::string name;
    std::string velocity;
    std::string acceleration;
    if (std::getline(item_stream, name, ':') &&
      std::getline(item_stream, velocity, ':') &&
      std::getline(item_stream, acceleration, ':'))
    {
      strategies.push_back(
        Strategy{name, std::stod(velocity), std::stod(acceleration)});
    }
  }
  return strategies;
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

double trajectory_duration(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }
  const auto & duration = trajectory.points.back().time_from_start;
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}
}  // namespace

class ControlStrategyBenchmark : public rclcpp::Node
{
public:
  ControlStrategyBenchmark()
  : Node(
      "ur5e_control_strategy_benchmark",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    settling_tolerance_ = parameter_or_default("settling_tolerance", 0.01);
    response_ratio_ = parameter_or_default("response_ratio", 0.90);

    controller_state_sub_ =
      create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/joint_trajectory_controller/controller_state", 20,
      [this](control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
        record_controller_state(*msg);
      });
  }

  void start_trial(const std::string & strategy_name)
  {
    active_strategy_ = strategy_name;
    active_ = true;
    stats_ = TrackingStats{};
    samples_.clear();
  }

  TrackingStats stop_trial()
  {
    active_ = false;
    compute_response_metrics();
    return stats_;
  }

private:
  double parameter_or_default(const std::string & name, const double default_value)
  {
    if (has_parameter(name)) {
      return get_parameter(name).as_double();
    }
    return declare_parameter<double>(name, default_value);
  }

  void record_controller_state(
    const control_msgs::msg::JointTrajectoryControllerState & msg)
  {
    if (!active_) {
      return;
    }

    const auto joint_count = std::min(
      msg.error.positions.size(), msg.joint_names.size());
    const auto desired_count = msg.desired.positions.size();
    const auto actual_count = msg.actual.positions.size();
    if (joint_count == 0 || desired_count == 0 || actual_count == 0) {
      return;
    }
    const auto sample_joint_count = std::min(
      joint_count, std::min(desired_count, actual_count));

    double max_sample_error = 0.0;
    double sample_squared_error = 0.0;
    for (std::size_t i = 0; i < sample_joint_count; ++i) {
      const double abs_error = std::abs(msg.error.positions[i]);
      max_sample_error = std::max(max_sample_error, abs_error);
      sample_squared_error += abs_error * abs_error;
    }

    const double stamp = stamp_to_seconds(msg.header.stamp);
    if (stats_.first_stamp < 0.0) {
      stats_.first_stamp = stamp;
    }
    stats_.last_stamp = stamp;
    stats_.samples += sample_joint_count;
    stats_.sum_squared_error += sample_squared_error;
    stats_.max_abs_error = std::max(stats_.max_abs_error, max_sample_error);
    stats_.final_abs_error = max_sample_error;

    ControllerSample sample;
    sample.stamp = stamp;
    sample.desired.assign(
      msg.desired.positions.begin(),
      msg.desired.positions.begin() + static_cast<long>(sample_joint_count));
    sample.actual.assign(
      msg.actual.positions.begin(),
      msg.actual.positions.begin() + static_cast<long>(sample_joint_count));
    samples_.push_back(sample);
  }

  void compute_response_metrics()
  {
    if (samples_.empty()) {
      return;
    }

    const auto joint_count = std::min(
      samples_.front().actual.size(), samples_.back().desired.size());
    if (joint_count == 0) {
      return;
    }

    const auto & first = samples_.front();
    const auto & last = samples_.back();
    std::vector<double> delta(joint_count, 0.0);
    std::vector<std::size_t> moving_joints;
    for (std::size_t i = 0; i < joint_count; ++i) {
      delta[i] = last.desired[i] - first.actual[i];
      if (std::abs(delta[i]) > 1e-4) {
        moving_joints.push_back(i);
      }
    }

    double steady_error_sum = 0.0;
    for (std::size_t i = 0; i < joint_count; ++i) {
      const double abs_error = std::abs(last.actual[i] - last.desired[i]);
      stats_.steady_state_max_error = std::max(stats_.steady_state_max_error, abs_error);
      steady_error_sum += abs_error;
    }
    stats_.steady_state_mean_error = steady_error_sum / static_cast<double>(joint_count);

    double max_overshoot_ratio = 0.0;
    double response_stamp = -1.0;
    for (const auto & sample : samples_) {
      bool all_reached = !moving_joints.empty();
      for (const auto joint_index : moving_joints) {
        const double progress =
          (sample.actual[joint_index] - first.actual[joint_index]) / delta[joint_index];
        max_overshoot_ratio = std::max(max_overshoot_ratio, progress - 1.0);
        if (progress < response_ratio_) {
          all_reached = false;
        }
      }
      if (response_stamp < 0.0 && all_reached) {
        response_stamp = sample.stamp;
      }
    }
    stats_.response_time_90 =
      response_stamp >= 0.0 ? response_stamp - first.stamp : 0.0;
    stats_.max_overshoot_percent = std::max(0.0, max_overshoot_ratio) * 100.0;

    double largest_motion = 0.0;
    for (const auto value : delta) {
      largest_motion = std::max(largest_motion, std::abs(value));
    }
    stats_.max_overshoot =
      largest_motion * std::max(0.0, max_overshoot_ratio);

    double settling_stamp = last.stamp;
    for (auto sample_it = samples_.rbegin(); sample_it != samples_.rend(); ++sample_it) {
      bool inside_band = true;
      for (std::size_t i = 0; i < joint_count; ++i) {
        if (std::abs(sample_it->actual[i] - last.desired[i]) > settling_tolerance_) {
          inside_band = false;
          break;
        }
      }
      if (!inside_band) {
        break;
      }
      settling_stamp = sample_it->stamp;
    }
    stats_.settling_time = settling_stamp - first.stamp;
  }

  bool active_{false};
  std::string active_strategy_;
  TrackingStats stats_;
  std::vector<ControllerSample> samples_;
  double settling_tolerance_{0.01};
  double response_ratio_{0.90};
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_state_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<ControlStrategyBenchmark>();
  auto const logger = rclcpp::get_logger("ur5e_control_strategy_benchmark");

  const auto strategies = parse_strategies(
    get_or_declare_parameter<std::string>(
      node,
      "strategies", "slow:0.10:0.10,medium:0.25:0.25,fast:0.50:0.50"));
  const int trials = get_or_declare_parameter<int>(node, "trials", 3);
  const std::string output_file = get_or_declare_parameter<std::string>(
    node,
    "output_file", "/tmp/ur5e_control_strategy_benchmark.csv");

  std::ofstream csv(output_file, std::ios::out | std::ios::trunc);
  if (!csv.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open output file: %s", output_file.c_str());
    rclcpp::shutdown();
    return 1;
  }
  csv << "strategy,trial,velocity_scale,acceleration_scale,success,"
    "planned_duration,measured_duration,samples,rms_error,max_abs_error,final_abs_error,"
    "steady_state_max_error,steady_state_mean_error,max_overshoot,"
    "max_overshoot_percent,response_time_90,settling_time\n";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");
  move_group.setPlanningTime(5.0);

  for (const auto & strategy : strategies) {
    for (int trial = 1; trial <= trials; ++trial) {
      move_group.setMaxVelocityScalingFactor(0.30);
      move_group.setMaxAccelerationScalingFactor(0.30);
      move_group.setNamedTarget("home");
      moveit::planning_interface::MoveGroupInterface::Plan reset_plan;
      if (static_cast<bool>(move_group.plan(reset_plan))) {
        static_cast<void>(move_group.execute(reset_plan));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      move_group.setStartStateToCurrentState();
      move_group.setMaxVelocityScalingFactor(strategy.velocity_scale);
      move_group.setMaxAccelerationScalingFactor(strategy.acceleration_scale);
      move_group.setNamedTarget("test_configuration");

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned = static_cast<bool>(move_group.plan(plan));
      bool executed = false;
      node->start_trial(strategy.name);
      if (planned) {
        executed = static_cast<bool>(move_group.execute(plan));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      const auto stats = node->stop_trial();

      const double measured_duration =
        (stats.first_stamp >= 0.0 && stats.last_stamp >= stats.first_stamp) ?
        stats.last_stamp - stats.first_stamp : 0.0;
      const double rms_error =
        stats.samples > 0 ?
        std::sqrt(stats.sum_squared_error / static_cast<double>(stats.samples)) : 0.0;
      const auto & joint_trajectory = plan.trajectory_.joint_trajectory;

      csv << strategy.name << "," << trial << ","
          << strategy.velocity_scale << "," << strategy.acceleration_scale << ","
          << (planned && executed ? 1 : 0) << ","
          << (planned ? trajectory_duration(joint_trajectory) : 0.0) << ","
          << measured_duration << ","
          << stats.samples << ","
          << rms_error << ","
          << stats.max_abs_error << ","
          << stats.final_abs_error << ","
          << stats.steady_state_max_error << ","
          << stats.steady_state_mean_error << ","
          << stats.max_overshoot << ","
          << stats.max_overshoot_percent << ","
          << stats.response_time_90 << ","
          << stats.settling_time << "\n";
      csv.flush();

      RCLCPP_INFO(
        logger,
        "%s trial %d/%d: %s, RMS error %.6f rad, max error %.6f rad",
        strategy.name.c_str(), trial, trials,
        planned && executed ? "success" : "failed",
        rms_error, stats.max_abs_error);
    }
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
