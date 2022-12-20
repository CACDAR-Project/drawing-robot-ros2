#include <cstdio>
#include <functional>
#include <memory>
#include <thread>

#include "robot_controller/robot_controller.hpp"
#include "robot_interfaces/action/execute_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

//#include "robot_interfaces/visibility_control.h"

//
// https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
