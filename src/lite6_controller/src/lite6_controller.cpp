#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <robot_controller/robot_controller.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

const std::string MOVE_GROUP = "lite6";

//
class Lite6Controller : public RobotController
{
public:
  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group;

  //bool moved = false;

  Lite6Controller(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : RobotController(options),
      move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
  {
    // Use upper joint velocity and acceleration limits
    //this->move_group.setMaxAccelerationScalingFactor(1.0);
    //this->move_group.setMaxVelocityScalingFactor(1.0);

    // Subscribe to target pose
    //target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Initialization successful.");

  }

  // TODO implement time param
  // https://moveit.picknik.ai/foxy/doc/move_group_interface/move_group_interface_tutorial.html
  // https://groups.google.com/g/moveit-users/c/MOoFxy2exT4
  // https://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html

  // TODO implement feedback
  // https://answers.ros.org/question/249995/how-to-check-sate-of-plan-execution-in-moveit-during-async-execution-in-python/

  /// Callback that executes path on robot
  virtual void executePath(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotion>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(20);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<robot_interfaces::action::ExecuteMotion::Feedback>();
    auto result = std::make_shared<robot_interfaces::action::ExecuteMotion::Result>();

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(move_group.getCurrentPose().pose);
    for (auto p : goal->motion.path)
      waypoints.push_back(p.pose);

    moveit_msgs::msg::RobotTrajectory trajectory;

    // dangerous with real robot
    // https://moveit.picknik.ai/galactic/doc/examples/move_group_interface/move_group_interface_tutorial.html
    const double jump_threshold = 0.0;

    const double eef_step = 0.01;
    double fraction = this->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    this->move_group.execute(trajectory);

    //waypoints.clear();

    //for (int i = 1; (i <= 10) && rclcpp::ok(); ++i) {
    //  // Check if there is a cancel request
    //  if (goal_handle->is_canceling()) {
    //    result->result = feedback->status;
    //    goal_handle->canceled(result);
    //    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //    return;
    //  }
    //  // Update status
    //  feedback->status = std::to_string(i) + "/10 complete";
    //  // Publish feedback
    //  goal_handle->publish_feedback(feedback);
    //  RCLCPP_INFO(this->get_logger(), feedback->status.c_str());

    //  loop_rate.sleep();
    //}

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = "success";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting lite6_controller");
  auto lite6 = std::make_shared<Lite6Controller>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lite6);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
