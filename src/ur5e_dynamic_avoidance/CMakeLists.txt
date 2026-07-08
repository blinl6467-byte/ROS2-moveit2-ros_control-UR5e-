cmake_minimum_required(VERSION 3.8)
project(ur5e_dynamic_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)

set(DEPENDENCIES
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
)

add_executable(moving_obstacle_scene src/moving_obstacle_scene.cpp)
target_compile_features(moving_obstacle_scene PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(moving_obstacle_scene ${DEPENDENCIES})

add_executable(obstacle_avoidance_demo src/dynamic_avoidance_demo.cpp)
target_compile_features(obstacle_avoidance_demo PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(obstacle_avoidance_demo ${DEPENDENCIES})

add_executable(obstacle_planner_benchmark src/dynamic_planner_benchmark.cpp)
target_compile_features(obstacle_planner_benchmark PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(obstacle_planner_benchmark ${DEPENDENCIES})

install(TARGETS
  moving_obstacle_scene
  obstacle_avoidance_demo
  obstacle_planner_benchmark
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch results
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/plot_obstacle_benchmark.py
  scripts/summarize_dynamic_benchmark.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
