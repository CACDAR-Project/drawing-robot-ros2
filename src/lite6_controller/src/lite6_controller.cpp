#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <robot_controller/robot_controller.hpp>
#include <chrono>
//#include <queue>

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

// MOTION PLANNING API
// https://github.com/ros-planning/moveit2_tutorials/blob/humble/doc/examples/motion_planning_api/src/motion_planning_api_tutorial.cpp
// https://moveit.picknik.ai/humble/doc/examples/motion_planning_api/motion_planning_api_tutorial.html
// https://github.com/ros-planning/moveit2/blob/main/moveit_planners/pilz_industrial_motion_planner/src/move_group_sequence_service.cpp
//
//
// USE
// https://industrial-training-master.readthedocs.io/en/foxy/_source/session4/ros2/0-Motion-Planning-CPP.html

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

  /**
   * TODO Use instead of MoveGroupInterface
   * https://industrial-training-master.readthedocs.io/en/foxy/_source/session4/ros2/0-Motion-Planning-CPP.html
   */
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::Client<moveit_msgs::srv::GetMotionSequence>::SharedPtr sequence_client_;

  //std::queue<std::pair<moveit_msgs::msg::RobotTrajectory,rclcpp_action::ServerGoalHandle<ExecuteMotion>>> trajectory_queue;

  // Set limits for A4 paper
  // 297x210
  float xlim_lower = 0.10;
  float xlim_upper = 0.25;
  float ylim_lower = -0.10;
  float ylim_upper = 0.115;
  float zlim_lower = 0.157;
  float zlim_upper = 0.17;

  //bool moved = false;
  //
  // TODO get pilz working
  //std::unique_ptr<pilz_industrial_motion_planner::CommandListManager> command_list_manager_;
  // command_list_manager_ = std::make_unique<pilz_industrial_motion_planner::CommandListManager>(this->move_group.getNodeHandle(), this->move_group.getRobotModel());

  /**
   * CommandListManager, used to plan MotionSequenceRequest
   */
  //pilz_industrial_motion_planner::CommandListManager command_list_manager;
  //pilz_industrial_motion_planner::CommandListManager command_list_manager(*std::shared_ptr<rclcpp::Node>(std::move(this)), this->move_group.getRobotModel());

  /**
   * Constructor
   */
  Lite6Controller(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : RobotController(options),
      move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
      //moveit_cpp_(std::shared_ptr<rclcpp::Node>(std::move(this))),
      //planning_component_(MOVE_GROUP, moveit_cpp_),
      //command_list_manager(std::shared_ptr<rclcpp::Node>(std::move(this)), this->move_group.getRobotModel())
  {

    //command_list_manager = new pilz_industrial_motion_planner::CommandListManager(this->move_group.getNodeHandle(), this->move_group.getRobotModel());
    //command_list_manager = new pilz_industrial_motion_planner::CommandListManager(std::shared_ptr<rclcpp::Node>(std::move(this)), this->move_group.getRobotModel());
    // Use upper joint velocity and acceleration limits
    this->move_group.setMaxAccelerationScalingFactor(1.0);
    this->move_group.setMaxVelocityScalingFactor(1.0);

    // Subscribe to target pose
    //target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

  }

  void setup()
  {
    //moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(move_group.getNodeHandle());
    try
    {
      //moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());
      //moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(move_group.getNodeHandle());
      //planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(MOVE_GROUP, moveit_cpp_);
      //

      planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(this->shared_from_this(),
                                                                                               "robot_description");
      this->sequence_client_ =
        this->create_client<moveit_msgs::srv::GetMotionSequence>("plan_sequence_path");

      while (!sequence_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the 'plan_sequence_path' service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "'plan_sequence_path' service not available, waiting again...");
      }

    }
    catch (int e) {
      RCLCPP_ERROR(this->get_logger(), "Initialization Failed: %d", e);
      return;
    }


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


    // Set motion goal of end effector link
    //std::string ee_link = moveit_cpp_->getRobotModel()->getJointModelGroup(planning_component_->getPlanningGroupName())->getLinkModelNames().back();
    //RCLCPP_INFO(this->get_logger(), "Got ee_link");

    mpr.group_name = MOVE_GROUP;
    moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("link_eef", pose, tolerance_pose, tolerance_angle);
      //kinematic_constraints::constructGoalConstraints(ee_link, pose, tolerance_pose, tolerance_angle);

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
  //

  /**
   * Function that translates an input value with a given range to a value within another range.
   */
  float translate(float val, float lmin, float lmax, float rmin, float rmax)
  {
    float lspan = lmax - lmin;
    float rspan = rmax - rmin;
    float out = (val - lmin) / lspan;
    out = rmin + (val * rspan);

    // Ensure that output is within bounds
    out = std::max(rmin, out);
    out = std::min(rmax, out);
    return out;
  }

  /**
   * Translates a pose to the xArm coordinate frame
   */
  geometry_msgs::msg::PoseStamped translatePose(geometry_msgs::msg::PoseStamped pose)
  {
    // TODO support paper angle
    auto x = pose.pose.position.x;
    auto y = pose.pose.position.y;
    // X and Y are swapped in xArm space
    pose.pose.position.x = translate(y, 0, 1, xlim_lower, xlim_upper);
    pose.pose.position.y = translate(x, 0, 1, ylim_lower, ylim_upper);
    pose.pose.position.z = translate(pose.pose.position.z, 0, 1, zlim_lower, zlim_upper);

    return pose;
  }

  void logPose(geometry_msgs::msg::PoseStamped pose)
  {
    RCLCPP_INFO(this->get_logger(), "pose position.x: %f", pose.pose.position.x);
    RCLCPP_INFO(this->get_logger(), "pose position.y: %f", pose.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "pose position.z: %f", pose.pose.position.z);
  }

  /**
   * Creates a trajectory for a pose and appends it to a given trajectory
   */
  bool addPoseToTrajectory(geometry_msgs::msg::PoseStamped pose, moveit_msgs::msg::RobotTrajectory *trajectory, moveit::core::RobotStatePtr state)
  {
    pose = translatePose(pose);
    move_group.setPoseTarget(pose);
    //move_group.setApproximateJointValueTarget(pose, "link_eef");

    //moveit_msgs::msg::RobotTrajectory trajectory;
    //move_group.setPlanningPipelineId("PTP");
    move_group.setPlannerId("PTP");

    robot_trajectory::RobotTrajectory previous_trajectory(state->getRobotModel(), move_group.getName());
    previous_trajectory.setRobotTrajectoryMsg(*state, *trajectory);


    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(this->get_logger(), "Plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED, trying approximate method (if enabled)");

    if (success)
    {
      robot_trajectory::RobotTrajectory next_trajectory(state->getRobotModel(), move_group.getName());
      next_trajectory.setRobotTrajectoryMsg(*state, plan.trajectory_);

      // append trajectory, with time step of 2.0, not skipping any points
      previous_trajectory.append(next_trajectory, 0.000001, 0);
      *trajectory = moveit_msgs::msg::RobotTrajectory();
      previous_trajectory.getRobotTrajectoryMsg(*trajectory);

      // Append segment to complete trajectory
      //trajectory->joint_trajectory.points.insert(trajectory->joint_trajectory.points.end(),
      //                                    plan.trajectory_.joint_trajectory.points.begin(),
      //                                    plan.trajectory_.joint_trajectory.points.end());
      //trajectory->joint_trajectory.joint_names = plan.trajectory_.joint_trajectory.joint_names;

    }
    else
    {
      // Unsafe approximate solution
      // Likely to break pens
      //success = addPoseToTrajectoryApproximate(pose, trajectory);
    }
    move_group.clearPoseTarget();
    return success;
  }

  /**
   * Creates a trajectory for a pose and appends it to a given trajectory
   */
  bool addPoseToTrajectoryApproximate(geometry_msgs::msg::PoseStamped pose, moveit_msgs::msg::RobotTrajectory *trajectory)
  {
    pose = translatePose(pose);
    move_group.setApproximateJointValueTarget(pose, "link_eef");
    move_group.setPlannerId("PTP");

    robot_trajectory::RobotTrajectory previous_trajectory(move_group.getCurrentState()->getRobotModel(), move_group.getName());
    previous_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), *trajectory);


    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(this->get_logger(), "Plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED");

    if (success)
    {
      robot_trajectory::RobotTrajectory next_trajectory(move_group.getCurrentState()->getRobotModel(), move_group.getName());
      next_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), plan.trajectory_);

      // append trajectory, with time step of 2.0, not skipping any points
      previous_trajectory.append(next_trajectory, 0.0001, 0);
      *trajectory = moveit_msgs::msg::RobotTrajectory();
      previous_trajectory.getRobotTrajectoryMsg(*trajectory);

      // Append segment to complete trajectory
      //trajectory->joint_trajectory.points.insert(trajectory->joint_trajectory.points.end(),
      //                                    plan.trajectory_.joint_trajectory.points.begin(),
      //                                    plan.trajectory_.joint_trajectory.points.end());
      //trajectory->joint_trajectory.joint_names = plan.trajectory_.joint_trajectory.joint_names;

    }
    move_group.clearPoseTarget();
    return success;
  }

  bool sendMSR(moveit_msgs::msg::MotionSequenceRequest msr)
  {
    RCLCPP_INFO(this->get_logger(), "Creating req");
    auto req = rclcpp::Client<moveit_msgs::srv::GetMotionSequence>::SharedRequest();
    RCLCPP_INFO(this->get_logger(), "Setting msr request");
    req->request = msr;
    RCLCPP_INFO(this->get_logger(), "Sending request to sequence service");
    auto result = sequence_client_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
      //trajectory = result.get()->response->trajectory;
      for (auto t : result.get()->response.planned_trajectories)
      {
        // TODO
        //trajectory->append(t, 0.0001, 0);
        RCLCPP_INFO(this->get_logger(), "Executing trajectory of length: %ld", t.joint_trajectory.points.size());
        this->move_group.execute(t);
      }
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call motion sequence service");
      return false;
    }
  }

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

    // getting current state of robot from environment
    //if (!moveit_cpp_->getPlanningSceneMonitor()->requestPlanningSceneState())
    //{
    //  RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene");
    //  return;
    //}
    //moveit::core::RobotStatePtr start_robot_state = moveit_cpp_->getCurrentState(2.0);

    moveit_msgs::msg::MotionSequenceRequest msr = moveit_msgs::msg::MotionSequenceRequest();
    //waypoints.push_back(move_group.getCurrentPose().pose);
    std::string ee_link = move_group.getLinkNames().back();
    RCLCPP_INFO(this->get_logger(), "Got ee_link: %s", ee_link.c_str());
    for (auto p : goal->motion.path)
    {
      //RCLCPP_INFO(this->get_logger(), "Creating MSI");
      moveit_msgs::msg::MotionSequenceItem msi = moveit_msgs::msg::MotionSequenceItem();

    planning_interface::MotionPlanRequest mpr = planning_interface::MotionPlanRequest();
    mpr.planner_id = "PTP";
    mpr.allowed_planning_time = 100;
    mpr.max_cartesian_speed = 1; // m/s
    //mpr.goal_constraints.position_constraints.header.frame_id = "world";

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);


    // Set motion goal of end effector link
    //std::string ee_link = moveit_cpp_->getRobotModel()->getJointModelGroup(planning_component_->getPlanningGroupName())->getLinkModelNames().back();
    //RCLCPP_INFO(this->get_logger(), "Got ee_link");


    mpr.group_name = MOVE_GROUP;
    //moveit_msgs::msg::Constraints pose_goal =
    //  kinematic_constraints::constructGoalConstraints(ee_link, p, tolerance_pose, tolerance_angle);
      //kinematic_constraints::constructGoalConstraints(ee_link, pose, tolerance_pose, tolerance_angle);
    moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(ee_link, p, 1.0, 1.0);
      //kinematic_constraints::constructGoalConstraints(ee_link, p, 1.0, 1.0);
      //kinematic_constraints::constructGoalConstraints(ee_link, p, 1e-3, 1e-2);

    mpr.goal_constraints.push_back(pose_goal);
    mpr.max_velocity_scaling_factor = 1.0;
    mpr.max_acceleration_scaling_factor = 1.0;

      msi.req = mpr;
      msi.blend_radius = 0.001; //TODO make configurable
      msr.items.push_back(msi);
    }
    msr.items.back().blend_radius = 0.0; // Last element blend must be 0
    moveit::core::RobotStatePtr move_group_state = move_group.getCurrentState();
    moveit_msgs::msg::RobotState state_msg;
    moveit::core::robotStateToRobotStateMsg(*move_group_state, state_msg);
    msr.items.front().req.start_state = state_msg;
    RCLCPP_INFO(this->get_logger(), "Created MSR");

    //moveit::core::RobotStatePtr move_group_state = move_group.getCurrentState();
    //robot_trajectory::RobotTrajectory trajectory(move_group_state.move_group.getName());
    //moveit_msgs::msg::RobotTrajectory *trajectory_msg;
    //sendMSR(msr);
    RCLCPP_INFO(this->get_logger(), "Creating req");
    auto req = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
    RCLCPP_INFO(this->get_logger(), "Setting msr request");
    req->request = msr;
    RCLCPP_INFO(this->get_logger(), "Sending request to sequence service");
    auto res = sequence_client_->async_send_request(req);
    // Wait for the result.
    res.wait();
    try
    {
      for (auto t : res.get()->response.planned_trajectories)
      {
        // TODO
        //trajectory->append(t, 0.0001, 0);
        RCLCPP_INFO(this->get_logger(), "Executing trajectory of length: %ld", t.joint_trajectory.points.size());
        this->move_group.execute(t);
      }
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call motion sequence service");
    }
    //if (plan_success)
    //{
    //  //trajectory.setRobotTrajectoryMsg(*move_group_state, trajectory_msg);
    //  RCLCPP_INFO(this->get_logger(), "Executing trajectory with %ld points", trajectory_msg->joint_trajectory.points.size());
    //  this->move_group.execute(*trajectory_msg);
    //}
    //else
    //{
    //  RCLCPP_ERROR(this->get_logger(), "Motion sequence planning failed, not executing");
    //}


    //moveit_msgs::msg::RobotTrajectory multi_trajectory;
    ////robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), MOVE_GROUP);
    //robot_trajectory::RobotTrajectory single_trajectory(move_group.getCurrentState()->getRobotModel(), move_group.getName());

    //move_group.setStartStateToCurrentState();
    //moveit::core::RobotStatePtr move_group_state = move_group.getCurrentState();
    //for (auto p : goal->motion.path)
    //{
    //  //RCLCPP_INFO(this->get_logger(), "Planning trajectory");

    //  // Append next pose to trajectory
    //  if (!addPoseToTrajectory(p, &multi_trajectory, move_group_state)) continue;

    //  // set move_group start state to final pose of trajectory
    //  //RCLCPP_INFO(this->get_logger(), "setRobotTrajectoryMsg");
    //  single_trajectory.setRobotTrajectoryMsg(*move_group_state, multi_trajectory);
    //  //rt.setRobotTrajectoryMsg(rt.getLastWayPoint(), trajectory);
    //  //RCLCPP_INFO(this->get_logger(), "getLastWayPoint");
    //  //
    //  //moveit::core::RobotState robot_state = single_trajectory.getLastWayPoint();
    //  moveit::core::RobotState robot_state = single_trajectory.getLastWayPoint();
    //  //RCLCPP_INFO(this->get_logger(), "eef");
    //  //const Eigen::Isometry3d& eef_transform = robot_state.getGlobalLinkTransform(move_group.getEndEffectorLink());
    //  //geometry_msgs::msg::Pose pose;
    //  //pose = Eigen::toMsg(eef_transform);
    //  move_group.setStartState(robot_state);


    //  //trajectory = moveit_msgs::msg::RobotTrajectory();
    //}

    //RCLCPP_INFO(this->get_logger(), "Executing trajectory with %ld points", multi_trajectory.joint_trajectory.points.size());
    //this->move_group.execute(multi_trajectory);

    //double fraction = this->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);


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

  // TODO remove sleep if not necessary
  // Sleep in case move_group not loaded
  rclcpp::sleep_for(2s);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting up lite6_controller");
  lite6->setup();

  //rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(lite6);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
