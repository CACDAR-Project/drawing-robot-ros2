cmake_minimum_required(VERSION 3.8)
project(robot_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(robot_controller src/cpp/robot_controller.cpp)
ament_target_dependencies(robot_controller
  "rclcpp"
  "rclcpp_action"
  "robot_interfaces")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  robot_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
