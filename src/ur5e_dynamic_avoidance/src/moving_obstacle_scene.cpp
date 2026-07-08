#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
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
  auto node = std::make_shared<rclcpp::Node>(
    "ur5e_moving_obstacle_scene",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto logger = rclcpp::get_logger("ur5e_moving_obstacle_scene");

  const std::string frame_id = get_or_declare_parameter<std::string>(node, "frame_id", "base_link");
  const std::string obstacle_id =
    get_or_declare_parameter<std::string>(node, "obstacle_id", "dynamic_sweeping_box");
  const double center_x = get_or_declare_parameter<double>(node, "center_x", 0.38);
  const double center_y = get_or_declare_parameter<double>(node, "center_y", 0.0);
  const double center_z = get_or_declare_parameter<double>(node, "center_z", 0.34);
  const double amplitude_y = get_or_declare_parameter<double>(node, "amplitude_y", 0.22);
  const double period = get_or_declare_parameter<double>(node, "period", 8.0);
  const double update_rate = get_or_declare_parameter<double>(node, "update_rate", 5.0);
  const std::array<double, 3> size = {
    get_or_declare_parameter<double>(node, "size_x", 0.16),
    get_or_declare_parameter<double>(node, "size_y", 0.16),
    get_or_declare_parameter<double>(node, "size_z", 0.32)};

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.removeCollisionObjects({obstacle_id});
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  const auto started = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(update_rate);
  RCLCPP_INFO(
    logger, "Publishing moving obstacle '%s' in frame '%s'.", obstacle_id.c_str(),
    frame_id.c_str());

  while (rclcpp::ok()) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - started).count();
    const double phase = 2.0 * M_PI * elapsed / period;
    const double y = center_y + amplitude_y * std::sin(phase);
    planning_scene_interface.applyCollisionObject(
      make_box(obstacle_id, frame_id, center_x, y, center_z, size));
    rclcpp::spin_some(node);
    rate.sleep();
  }

  planning_scene_interface.removeCollisionObjects({obstacle_id});
  rclcpp::shutdown();
  return 0;
}
