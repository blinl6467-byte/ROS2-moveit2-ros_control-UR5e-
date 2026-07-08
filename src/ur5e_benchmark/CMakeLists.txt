cmake_minimum_required(VERSION 3.8)
project(ur5e_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(planner_benchmark src/planner_benchmark.cpp)
target_compile_features(planner_benchmark PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  planner_benchmark
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(control_strategy_benchmark src/control_strategy_benchmark.cpp)
target_compile_features(control_strategy_benchmark PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  control_strategy_benchmark
  "control_msgs"
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS
  planner_benchmark
  control_strategy_benchmark
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/plot_control_benchmark.py
  scripts/summarize_benchmarks.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
