#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <robot_controller/robot_controller.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
//#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>

#include <pilz_industrial_motion_planner/command_list_manager.h>

const std::string MOVE_GROUP = "lite6";



// MOTION PLANNING API
// https://github.com/ros-planning/moveit2_tutorials/blob/humble/doc/examples/motion_planning_api/src/motion_planning_api_tutorial.cpp
// https://moveit.picknik.ai/humble/doc/examples/motion_planning_api/motion_planning_api_tutorial.html
// https://github.com/ros-planning/moveit2/blob/main/moveit_planners/pilz_industrial_motion_planner/src/move_group_sequence_service.cpp
//
//

/**
 * Controller for xArm Lite6, implementing RobotController
 */
class Lite6Controller : public RobotController
{
public:
  /**
   * Move group interface for the robot
   */
  moveit::planning_interface::MoveGroupInterface move_group;

  //bool moved = false;
  //
  // TODO get pilz working
  //std::unique_ptr<pilz_industrial_motion_planner::CommandListManager> command_list_manager_;
  // command_list_manager_ = std::make_unique<pilz_industrial_motion_planner::CommandListManager>(this->move_group.getNodeHandle(), this->move_group.getRobotModel());

  /**
   * CommandListManager, used to plan MotionSequenceRequest
   */
  pilz_industrial_motion_planner::CommandListManager command_list_manager;
  //pilz_industrial_motion_planner::CommandListManager command_list_manager(*std::shared_ptr<rclcpp::Node>(std::move(this)), this->move_group.getRobotModel());

  /**
   * Constructor
   */
  Lite6Controller(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : RobotController(options),
      move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP),
      command_list_manager(std::shared_ptr<rclcpp::Node>(std::move(this)), this->move_group.getRobotModel())
  {

    //command_list_manager = new pilz_industrial_motion_planner::CommandListManager(this->move_group.getNodeHandle(), this->move_group.getRobotModel());
    //command_list_manager = new pilz_industrial_motion_planner::CommandListManager(std::shared_ptr<rclcpp::Node>(std::move(this)), this->move_group.getRobotModel());
    // Use upper joint velocity and acceleration limits
    //this->move_group.setMaxAccelerationScalingFactor(1.0);
    //this->move_group.setMaxVelocityScalingFactor(1.0);

    // Subscribe to target pose
    //target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

    move_group.setPlanningTime(30.0);
    RCLCPP_INFO(this->get_logger(), "Initialization successful.");

  }

  /**
   * This function takes a pose and returns a MotionPlanRequest
   */
  planning_interface::MotionPlanRequest createRequest(geometry_msgs::msg::PoseStamped pose)
  {
    planning_interface::MotionPlanRequest mpr = planning_interface::MotionPlanRequest();
    mpr.planner_id = "PTP";
    //mpr.goal_constraints.position_constraints.header.frame_id = "world";

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    mpr.group_name = MOVE_GROUP;
    moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("link_eef", pose, tolerance_pose, tolerance_angle);

    mpr.goal_constraints.push_back(pose_goal);
    return mpr;
  }

  // TODO implement time param
  // https://moveit.picknik.ai/foxy/doc/move_group_interface/move_group_interface_tutorial.html
  // https://groups.google.com/g/moveit-users/c/MOoFxy2exT4
  // https://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html

  // TODO implement feedback
  // https://answers.ros.org/question/249995/how-to-check-sate-of-plan-execution-in-moveit-during-async-execution-in-python/
  //
  // Useful
  // https://groups.google.com/g/moveit-users/c/I4sPhq_JGQk
  // https://groups.google.com/g/moveit-users/c/MOoFxy2exT4/m/0AwRHOuEwRgJ
  // https://discourse.ros.org/t/moveit-trajectory-through-waypoints/17439
  // https://answers.ros.org/question/330632/moveit-motion-planning-how-should-i-splice-several-planned-motion-trajectories/
  // https://groups.google.com/g/moveit-users/c/lZL2HTjLu-k

  /**
   * Callback that executes path on robot
   */
  virtual void executePath(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotion>> goal_handle)
  {

    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(20);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<robot_interfaces::action::ExecuteMotion::Feedback>();
    auto result = std::make_shared<robot_interfaces::action::ExecuteMotion::Result>();

    moveit_msgs::msg::MotionSequenceRequest msr;
    //waypoints.push_back(move_group.getCurrentPose().pose);
    for (auto p : goal->motion.path)
    {
      moveit_msgs::msg::MotionSequenceItem msi;
      msi.req = createRequest(p);
      msi.blend_radius = 0.001; //TODO make configurable
      msr.items.push_back(msi);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;

    //double fraction = this->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

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

/**
 * Starts lite6_controller
 */
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
