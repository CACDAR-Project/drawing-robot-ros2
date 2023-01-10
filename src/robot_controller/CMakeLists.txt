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
find_package(rclcpp_components REQUIRED)
find_package(robot_interfaces REQUIRED)

include_directories(include)

add_library(robot_controller src/cpp/robot_controller.cpp)
ament_target_dependencies(robot_controller
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "robot_interfaces")
add_executable(dummy_controller src/cpp/dummy_controller.cpp)
ament_target_dependencies(dummy_controller
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "robot_interfaces")

ament_export_targets(robot_controller HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/robot_controller
  DESTINATION include/
)
install(
  TARGETS robot_controller
  EXPORT robot_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#endif()

install(
  TARGETS dummy_controller
  DESTINATION lib/${PROJECT_NAME})



ament_package()
