//#include "follow.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/msg/color_rgba.hpp>

const std::string MOVE_GROUP = "lite6";

class MoveItFollowTarget : public rclcpp::Node
{
public:
  /// Constructor
  MoveItFollowTarget();

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;
  /// Subscriber for target pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  /// Target pose that is used to detect changes
  geometry_msgs::msg::Pose previous_target_pose_;

  bool moved = false;
  std::vector<geometry_msgs::msg::Pose> waypoints;


private:
  /// Callback that plans and executes trajectory each time the target pose is changed
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
};

MoveItFollowTarget::MoveItFollowTarget() : Node("follow_target"),
                                           move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
  // Use upper joint velocity and acceleration limits
  this->move_group_.setMaxAccelerationScalingFactor(1.0);
  this->move_group_.setMaxVelocityScalingFactor(1.0);

  // Subscribe to target pose
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MoveItFollowTarget::target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // Return if target pose is unchanged
  if (msg->pose == this->previous_target_pose_)
  {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Target pose has changed. Planning and executing...");

  //if (this->moved)
  if (false)
  {
    //https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/how_to_guides/using_ompl_constrained_planning/src/ompl_constrained_planning_tutorial.cpp
    //
    moveit_msgs::msg::PositionConstraint plane_constraint;
    plane_constraint.header.frame_id = this->move_group_.getPoseReferenceFrame();
    plane_constraint.link_name = this->move_group_.getEndEffectorLink();
    shape_msgs::msg::SolidPrimitive plane;
    plane.type = shape_msgs::msg::SolidPrimitive::BOX;
    //plane.dimensions = { 5.0, 0.0005, 5.0 };
    plane.dimensions = { 5.0, 1.0, 5.0 };
    plane_constraint.constraint_region.primitives.emplace_back(plane);

    geometry_msgs::msg::Pose plane_pose;
    plane_pose.position.x = 0.14;
    plane_pose.position.y = -0.3;
    plane_pose.position.z = 1.0;
    plane_pose.orientation.x = 0.0;
    plane_pose.orientation.y = 0.0;
    plane_pose.orientation.z = 0.0;
    plane_pose.orientation.w = 0.0;
    plane_constraint.constraint_region.primitive_poses.emplace_back(plane_pose);
    plane_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints plane_constraints;
    plane_constraints.position_constraints.emplace_back(plane_constraint);
    plane_constraints.name = "use_equality_constraints";
    this->move_group_.setPathConstraints(plane_constraints);

    // And again, configure and solve the planning problem
    this->move_group_.setPoseTarget(msg->pose);
    this->move_group_.setPlanningTime(10.0);
    //success = (this->move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //RCLCPP_INFO(this->get_logger(), "Plan with plane constraint %s", success ? "" : "FAILED");
    this->move_group_.move();
  }
  //else
  //{
  //  // Plan and execute motion
  //  this->move_group_.setPoseTarget(msg->pose);
  //  this->move_group_.move();
  //}

  waypoints.push_back(msg->pose);

  if (waypoints.size() >= 2)
  {
      moveit_msgs::msg::RobotTrajectory trajectory;

      // dangerous with real robot
      // https://moveit.picknik.ai/galactic/doc/examples/move_group_interface/move_group_interface_tutorial.html
      const double jump_threshold = 0.0;

      const double eef_step = 0.01;
      double fraction = this->move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

      this->move_group_.execute(trajectory);

      waypoints.clear();
  }

  // Update for next callback
  previous_target_pose_ = msg->pose;
  this->moved = true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto target_follower = std::make_shared<MoveItFollowTarget>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(target_follower);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
