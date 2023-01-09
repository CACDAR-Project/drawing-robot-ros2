cmake_minimum_required(VERSION 3.8)
project(axidraw_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_controller REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(axidraw_controller src/cpp/axidraw_controller.cpp)
ament_target_dependencies(axidraw_controller
    "rclcpp"
    "geometry_msgs"
    "robot_controller"
    "robot_interfaces"
)


install(TARGETS
  axidraw_controller
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
    src/py/axidraw_serial.py
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
#install(DIRECTORY launch rviz urdf worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
