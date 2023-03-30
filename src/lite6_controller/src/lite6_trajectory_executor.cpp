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

const std::string MOVE_GROUP = "lite6";

using namespace std::chrono_literals;

class TrajectoryExecutor : public rclcpp::Node
{
  public:
    //trajectory_pub = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("lite6_trajectory", 10);
    rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr subscription;
    moveit::planning_interface::MoveGroupInterface move_group;

    rclcpp::TimerBase::SharedPtr trajectory_timer_;
    std::queue<moveit_msgs::msg::RobotTrajectory> trajectory_queue;
    bool busy = false;

    TrajectoryExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
      : Node("lite6_trajectory_executor",options),
        move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
    {
      subscription = this->create_subscription<moveit_msgs::msg::RobotTrajectory>(
        "lite6_trajectory", 100, std::bind(&TrajectoryExecutor::trajectory_callback, this, std::placeholders::_1));

      trajectory_timer_ = this->create_wall_timer(
        10ms, std::bind(&TrajectoryExecutor::executeTrajectoryFromQueue, this));
    }
    void trajectory_callback(const moveit_msgs::msg::RobotTrajectory msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received trajectory, adding to queue");
      //move_group.execute(msg);
      trajectory_queue.push(msg);
    }

    void executeTrajectoryFromQueue()
    {
      if (busy || trajectory_queue.empty())
        return;
      busy = true;
      RCLCPP_INFO(this->get_logger(), "Executing next trajectory from queue");
      move_group.execute(trajectory_queue.front());
      trajectory_queue.pop();
      busy = false;
      RCLCPP_INFO(this->get_logger(), "Finished executing trajectory");
    }
};

/**
 * Starts lite6_controller
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto trajectory_executor = std::make_shared<TrajectoryExecutor>();

  // TODO remove sleep if not necessary
  // Sleep in case move_group not loaded
  rclcpp::sleep_for(2s);

  //rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::executors::MultiThreadedExecutor executor;
  //executor.add_node(lite6);
  executor.add_node(trajectory_executor);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
