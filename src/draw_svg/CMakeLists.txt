cmake_minimum_required(VERSION 3.8)
project(draw_svg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(ign_moveit2_examples REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Install C++
set(SRC_CPP_DIR src/cpp)
# Example 0 - Follow target
set(EXECUTABLE_0 log_position)
add_executable(${EXECUTABLE_0} ${SRC_CPP_DIR}/${EXECUTABLE_0}.cpp)
ament_target_dependencies(${EXECUTABLE_0}
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
)
install(TARGETS
    ${EXECUTABLE_0}
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python
set(SRC_PY_DIR src/py)
install(PROGRAMS
    ${SRC_PY_DIR}/draw_svg.py
    ${SRC_PY_DIR}/follow.py
    ${SRC_PY_DIR}/drawing_surface.py
    DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
    ${SRC_PY_DIR}/robots/lite6.py
    DESTINATION lib/${PROJECT_NAME}/robots/
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
install(DIRECTORY launch rviz urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()
