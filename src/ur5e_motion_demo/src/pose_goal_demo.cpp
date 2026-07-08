#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // 自动声明 launch/参数覆盖项，便于 MoveIt 与 ROS2 参数系统协同工作。
  auto const node = std::make_shared<rclcpp::Node>(
    "ur5e_pose_goal_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("ur5e_pose_goal_demo");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // MoveGroupInterface 内部依赖 ROS 回调，后台 executor 负责处理状态和 action 通信。
  auto spinner = std::thread([&executor]() {executor.spin();});

  using moveit::planning_interface::MoveGroupInterface;
  // 规划组名称必须与 SRDF 中的 ur_manipulator 保持一致。
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  move_group_interface.setPlanningTime(5.0);
  move_group_interface.setStartStateToCurrentState();

  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // 目标位姿位于 UR5e 常见可达空间内，姿态保持单位四元数。
  auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;
      msg.position.x = 0.28;
      msg.position.y = -0.2;
      msg.position.z = 0.5;
      return msg;
    }();
  move_group_interface.setPoseTarget(target_pose);

  // 先规划再执行，便于分别报告规划失败和控制器执行失败。
  auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

  if (success) {
    auto const executed = static_cast<bool>(move_group_interface.execute(plan));
    if (!executed) {
      RCLCPP_ERROR(logger, "Trajectory execution failed!");
    }
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
