#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
double value_or_nan(const std::vector<double> & values, const std::size_t index)
{
  if (index >= values.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return values[index];
}

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
}  // namespace

class TrajectoryRecorder : public rclcpp::Node
{
public:
  TrajectoryRecorder()
  : Node("ur5e_trajectory_recorder")
  {
    output_file_ = declare_parameter<std::string>("output_file", "/tmp/ur5e_trajectory_log.csv");
    std::filesystem::create_directories(std::filesystem::path(output_file_).parent_path());
    csv_.open(output_file_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error("Failed to open trajectory log file: " + output_file_);
    }

    csv_ << "source,stamp_sec,joint,desired_position,actual_position,error_position,"
      "joint_state_position,joint_state_velocity\n";

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {record_joint_state(*msg);});

    controller_state_sub_ =
      create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/joint_trajectory_controller/controller_state", 10,
      [this](control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
        record_controller_state(*msg);
      });

    RCLCPP_INFO(get_logger(), "Recording trajectory data to %s", output_file_.c_str());
  }

  ~TrajectoryRecorder() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

private:
  void record_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    const double stamp = stamp_to_seconds(msg.header.stamp);
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      csv_ << "joint_states," << stamp << "," << msg.name[i] << ",,,,"
           << value_or_nan(msg.position, i) << ","
           << value_or_nan(msg.velocity, i) << "\n";
    }
  }

  void record_controller_state(
    const control_msgs::msg::JointTrajectoryControllerState & msg)
  {
    const double stamp = stamp_to_seconds(msg.header.stamp);
    for (std::size_t i = 0; i < msg.joint_names.size(); ++i) {
      csv_ << "controller_state," << stamp << "," << msg.joint_names[i] << ","
           << value_or_nan(msg.reference.positions, i) << ","
           << value_or_nan(msg.feedback.positions, i) << ","
           << value_or_nan(msg.error.positions, i) << ",,\n";
    }
  }

  std::string output_file_;
  std::ofstream csv_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_state_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryRecorder>());
  rclcpp::shutdown();
  return 0;
}
