cmake_minimum_required(VERSION 3.8)
project(ur5e_motion_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pose_goal_demo src/pose_goal_demo.cpp)
target_compile_features(pose_goal_demo PUBLIC c_std_99 cxx_std_17)  # 要求 C99 和 C++17
ament_target_dependencies(
  pose_goal_demo
  "geometry_msgs"
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(joint_goal_demo src/joint_goal_demo.cpp)
target_compile_features(joint_goal_demo PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  joint_goal_demo
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(cartesian_waypoints_demo src/cartesian_waypoints_demo.cpp)
target_compile_features(cartesian_waypoints_demo PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  cartesian_waypoints_demo
  "geometry_msgs"
  "moveit_msgs"
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(trajectory_recorder src/trajectory_recorder.cpp)
target_compile_features(trajectory_recorder PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  trajectory_recorder
  "control_msgs"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS
  pose_goal_demo
  joint_goal_demo
  cartesian_waypoints_demo
  trajectory_recorder
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/plot_trajectory_log.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 下一行会跳过版权检查 linter
  # 当所有源文件都补充版权和许可证后，可注释掉该行
  set(ament_cmake_copyright_FOUND TRUE)
  # 下一行会跳过 cpplint（仅在 git 仓库中有效）
  # 当该包位于 git 仓库且满足下述条件时，可注释掉该行
  # 已为所有源文件添加版权和许可证
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
