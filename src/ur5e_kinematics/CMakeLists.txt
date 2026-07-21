cmake_minimum_required(VERSION 3.8)
project(ur5e_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ur5e_kinematics.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

add_executable(kinematics_benchmark src/kinematics_benchmark.cpp)
target_compile_features(kinematics_benchmark PUBLIC cxx_std_17)
target_link_libraries(kinematics_benchmark PRIVATE ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME} kinematics_benchmark
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(kdl_parser REQUIRED)
  find_package(orocos_kdl REQUIRED)
  find_package(ur_description REQUIRED)
  find_package(xacro REQUIRED)

  ament_add_gtest(test_ur5e_kinematics test/test_ur5e_kinematics.cpp)
  target_compile_features(test_ur5e_kinematics PUBLIC cxx_std_17)
  target_link_libraries(test_ur5e_kinematics ${PROJECT_NAME})

  set(TEST_URDF "${CMAKE_CURRENT_BINARY_DIR}/ur5e_kinematics_reference.urdf")
  execute_process(
    COMMAND xacro
      "${CMAKE_CURRENT_SOURCE_DIR}/../ur_description/urdf/ur.urdf.xacro"
      name:=ur
      ur_type:=ur5e
    OUTPUT_FILE "${TEST_URDF}"
    RESULT_VARIABLE XACRO_RESULT
    ERROR_VARIABLE XACRO_ERROR
  )
  if(NOT XACRO_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to generate UR5e test URDF: ${XACRO_ERROR}")
  endif()

  ament_add_gtest(test_kdl_reference test/test_kdl_reference.cpp)
  target_compile_features(test_kdl_reference PUBLIC cxx_std_17)
  target_compile_definitions(test_kdl_reference PRIVATE TEST_URDF_PATH="${TEST_URDF}")
  target_link_libraries(test_kdl_reference ${PROJECT_NAME})
  ament_target_dependencies(test_kdl_reference kdl_parser orocos_kdl)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
