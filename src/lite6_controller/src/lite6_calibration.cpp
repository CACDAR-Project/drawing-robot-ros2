#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <robot_controller/robot_controller.hpp>
#include <chrono>
#include <cstdlib>
#include <queue>

#include <fstream>
#include <iostream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

//#include <moveit/planning_interface/planning_interface.h>
//#include <moveit/planning_interface/planning_response.h>
//#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>


#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <pilz_industrial_motion_planner/command_list_manager.h>

#include <xarm_msgs/srv/move_joint.hpp>
#include <xarm_msgs/srv/get_float32_list.hpp>

const std::string MOVE_GROUP = "lite6";

using namespace std::chrono_literals;

class Calibration : public rclcpp::Node
{
  public:
    //moveit::planning_interface::MoveGroupInterface move_group;

    rclcpp::TimerBase::SharedPtr status_timer;

    rclcpp::Client<xarm_msgs::srv::MoveJoint>::SharedPtr set_joint_client;
    rclcpp::Client<xarm_msgs::srv::GetFloat32List>::SharedPtr get_joint_client;

    Calibration(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
      : Node("lite6_calibration",options)
        //move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
    {
      //RCLCPP_INFO(this->get_logger(), "Starting state monitor");
      //move_group.startStateMonitor(30);
      //RCLCPP_INFO(this->get_logger(), "Started state monitor");
      //move_group.setPlanningTime(30.0);

      //status_timer = this->create_wall_timer(5s, std::bind(&Calibration::getTree, this));

      RCLCPP_ERROR(this->get_logger(), "create service client");
      set_joint_client = this->create_client<xarm_msgs::srv::MoveJoint>("/ufactory/set_servo_angle");
      get_joint_client = this->create_client<xarm_msgs::srv::GetFloat32List>("/ufactory/get_servo_angle");
      //xarm_msgs::srv::MoveJoint
      while (!set_joint_client->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "service not available, waiting again...");
      }
      while (!get_joint_client->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "service not available, waiting again...");
      }
      RCLCPP_ERROR(this->get_logger(), "Client ready");

      //RCLCPP_ERROR(this->get_logger(), "create request");
      //auto request = std::make_shared<xarm_msgs::srv::MoveJoint::Request>();
      //RCLCPP_ERROR(this->get_logger(), "send request");
      //auto result = joint_client->async_send_request(request);
      //RCLCPP_ERROR(this->get_logger(), "get request");
      //auto a = result.get()->message;
      //RCLCPP_ERROR(this->get_logger(), "A:%s", a.c_str());

      auto request = std::make_shared<xarm_msgs::srv::GetFloat32List::Request>();
      RCLCPP_ERROR(this->get_logger(), "");
      auto result = get_joint_client->async_send_request(request);
      auto msg = result.get()->message;
      RCLCPP_ERROR(this->get_logger(), "A:%s", msg.c_str());

    }

    void getTree()
    {
      //rclcpp::sleep_for(20s);
      RCLCPP_INFO(this->get_logger(), "Getting robot state");
      //moveit::core::RobotStatePtr move_group_state  = move_group.getCurrentState(30);
      //RCLCPP_INFO(this->get_logger(), move_group_state->getStateTreeString().c_str());
      //RCLCPP_ERROR(this->get_logger(), "AAAAAAAAAA");
    }

};

/**
 *
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);


  auto calibration = std::make_shared<Calibration>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(calibration);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
