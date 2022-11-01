#include <memory>
#include <cstdio>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/moveit_commander>
//https://github.com/AndrejOrsula/ign_moveit2_examples/blob/master/examples/cpp/ex_follow_target.cpp
const std::string MOVE_GROUP = "lite6";

class PositionLogger : public rclcpp::Node
{
public:
  /// Constructor
  PositionLogger();

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;
  /// Subscriber for target pose
  //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  /// Target pose that is used to detect changes
  //geometry_msgs::msg::Pose previous_target_pose_;

private:
  /// Callback that plans and executes trajectory each time the target pose is changed
  void log_position();
};

PositionLogger::PositionLogger() : Node("position_logger"),
                                   move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
  // Use upper joint velocity and acceleration limits
  this->move_group_.setMaxAccelerationScalingFactor(1.0);
  this->move_group_.setMaxVelocityScalingFactor(1.0);

  // Subscribe to target pose
  //target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));
  auto ros_clock = rclcpp::Clock::make_shared();
  auto timer_ = rclcpp::create_timer(this, ros_clock, rclcpp::Duration::from_nanoseconds(5000000),
   [=]()
   {
       this->log_position();
   });

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void PositionLogger::log_position()
{
  auto const logger = rclcpp::get_logger("position_logger");

  auto msg = this->move_group_.getCurrentPose();
  auto p = msg.pose.position;
  char a[100];
  std::sprintf(a, "Position x: %f, y: %f, z: %f", p.x,p.y,p.z);
  std::string s(a);

  //rclcpp::RCLCPP_INFO(logger, a);
  RCLCPP_INFO(logger, "AA");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // TODO Try moveit_commander::roscpp_initialize

  auto pl = std::make_shared<PositionLogger>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pl);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
