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
  moveit::core::RobotStatePtr move_group_state;

  std::queue<moveit_msgs::msg::RobotTrajectory> trajectory_queue;
  rclcpp::TimerBase::SharedPtr trajectory_timer_;
  bool busy = false;

  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub;

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
  float xlim_upper = 0.305;
  float ylim_lower = -0.1475;
  float ylim_upper = 0.1475;
  float zlim_lower = 0.210;
  float zlim_upper = 0.218;

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

    //setting this lower seems to improve overall time and prevents robot from moving too fast
    //this->move_group.setMaxVelocityScalingFactor(1.0);
    this->move_group.setMaxVelocityScalingFactor(0.8);


    trajectory_pub = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("lite6_trajectory", 100);

    // Subscribe to target pose
    //target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

    //
    trajectory_timer_ = this->create_wall_timer(
      10ms, std::bind(&Lite6Controller::executeTrajectoryFromQueue, this));

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
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting initial robot state");
      //move_group.startStateMonitor(5);

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

  /**
   * Checks if points are within a radius of eachother, specified by the tolerance argument
   */
  bool coincidentPoints(geometry_msgs::msg::Point *p1, geometry_msgs::msg::Point *p2, double tolerance)
  {
    bool x = std::abs(p1->x - p2->x) <= tolerance;
    bool y = std::abs(p1->y - p2->y) <= tolerance;
    bool z = std::abs(p1->z - p2->z) <= tolerance;
    return x && y && z;
  }

  /**
   *
   */
  moveit_msgs::msg::MotionSequenceRequest createMotionSequenceRequest(const std::vector<geometry_msgs::msg::PoseStamped> *path)
  {
    moveit_msgs::msg::MotionSequenceRequest msr = moveit_msgs::msg::MotionSequenceRequest();
    //waypoints.push_back(move_group.getCurrentPose().pose);
    std::string ee_link = move_group.getLinkNames().back();
    RCLCPP_INFO(this->get_logger(), "Got ee_link: %s", ee_link.c_str());
    geometry_msgs::msg::Point previous_point;
    //previous_point.point.x = -1.0;
    //previous_point.point.y = -1.0;
    //previous_point.point.z = -1.0;
    for (auto p : *path)
    {
      //RCLCPP_INFO(this->get_logger(), "Creating MSI");
      moveit_msgs::msg::MotionSequenceItem msi = moveit_msgs::msg::MotionSequenceItem();

      planning_interface::MotionPlanRequest mpr = planning_interface::MotionPlanRequest();
      mpr.planner_id = "PTP";
      //mpr.planner_id = "LIN";
      mpr.group_name = move_group.getName();
      mpr.max_velocity_scaling_factor = 1.0;
      mpr.max_acceleration_scaling_factor = 0.98;
      mpr.allowed_planning_time = 10;
      mpr.max_cartesian_speed = 2; // m/s
      //mpr.goal_constraints.position_constraints.header.frame_id = "world";

      // A tolerance of 0.01 m is specified in position
      // and 0.01 radians in orientation
      //std::vector<double> tolerance_pose(3, 0.01);
      //std::vector<double> tolerance_angle(3, 0.01);


      // Set motion goal of end effector link
      //std::string ee_link = moveit_cpp_->getRobotModel()->getJointModelGroup(planning_component_->getPlanningGroupName())->getLinkModelNames().back();


      //moveit_msgs::msg::Constraints pose_goal =
      //  kinematic_constraints::constructGoalConstraints(ee_link, p, tolerance_pose, tolerance_angle);
      //kinematic_constraints::constructGoalConstraints(ee_link, pose, tolerance_pose, tolerance_angle);

      //geometry_msgs::msg::PointStamped point;
      geometry_msgs::msg::Point point;
      auto pose = translatePose(p);
      moveit_msgs::msg::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints(ee_link, pose, 1e-3, 1e-2);
      //kinematic_constraints::constructGoalConstraints(ee_link, point, 1e-5);
      //kinematic_constraints::constructGoalConstraints(ee_link, p, 1.0, 1.0);
      //kinematic_constraints::constructGoalConstraints(ee_link, p, 1e-3, 1e-2);

      mpr.goal_constraints.push_back(pose_goal);

      msi.req = mpr;
      //msi.blend_radius = 6e-7; //TODO make configurable
      //msi.blend_radius = 0.000001; //TODO make configurable
      msi.blend_radius = 0.0; //TODO make configurable
      if (coincidentPoints(&pose.pose.position, &previous_point, msi.blend_radius + 1e-5))
      {
        RCLCPP_ERROR(this->get_logger(), "Detected coincident points, setting blend radius to 0.0");
        // if points are too close, set blend radius to zero.
        msi.blend_radius = 0.0;
        // also set previous to 0
        if (msr.items.size() > 0)
          msr.items.back().blend_radius = 0.0;
      }
      previous_point = pose.pose.position;

      //RCLCPP_ERROR(this->get_logger(), "Appending point with blend_radius:%f", msi.blend_radius);
      msr.items.push_back(msi);
    }
    msr.items.back().blend_radius = 0.0; // Last element blend must be 0
    moveit_msgs::msg::RobotState state_msg;
    if (move_group_state == NULL)
    {
      RCLCPP_INFO(this->get_logger(), "Getting robot state");
      move_group_state = move_group.getCurrentState(10);
    }
    moveit::core::robotStateToRobotStateMsg(*move_group_state, state_msg);
    msr.items.front().req.start_state = state_msg;
    return msr;
  }

  /**
   *
   */
  moveit_msgs::msg::MotionSequenceRequest createMotionSequenceRequest(const std::vector<geometry_msgs::msg::PoseStamped> *path, double x_offset, double y_offset, double z_offset)
  {
    this->xlim_lower += x_offset;
    this->xlim_upper += x_offset;
    this->ylim_lower += y_offset;
    this->ylim_upper += y_offset;
    this->zlim_lower += z_offset;
    this->zlim_upper += z_offset;
    moveit_msgs::msg::MotionSequenceRequest msr = createMotionSequenceRequest(path);
    this->xlim_lower -= x_offset;
    this->xlim_upper -= x_offset;
    this->ylim_lower -= y_offset;
    this->ylim_upper -= y_offset;
    this->zlim_lower -= z_offset;
    this->zlim_upper -= z_offset;
    return msr;
  }

  /**
   *
   */
  static void appendLineToFile(std::string filepath, std::string line)
  {
    std::ofstream file;
    file.open(filepath, std::ios::out | std::ios::app);
    if (file.fail())
        throw std::ios_base::failure(std::strerror(errno));

    file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);

    file << line << std::endl;
  }

  /**
   *
   */
  std::string pointsToString(const std::vector<geometry_msgs::msg::PoseStamped> *path, double x_offset, double y_offset, double z_offset)
  {
    std::string out = "";
    this->xlim_lower += x_offset;
    this->xlim_upper += x_offset;
    this->ylim_lower += y_offset;
    this->ylim_upper += y_offset;
    this->zlim_lower += z_offset;
    this->zlim_upper += z_offset;
    for (auto p : *path)
    {
      auto position = translatePose(p).pose.position;
      out = out + std::to_string(position.x) + " " +
                  std::to_string(position.y) + " " +
                  std::to_string(position.z) + ",";
    }
    out = out.substr(0, out.size()-1); //remove trailing comma
    this->xlim_lower -= x_offset;
    this->xlim_upper -= x_offset;
    this->ylim_lower -= y_offset;
    this->ylim_upper -= y_offset;
    this->zlim_lower -= z_offset;
    this->zlim_upper -= z_offset;
    return out;
  }

  /**
   * Tests path with offsets in multiple locations and logs them to OUTPUT.csv
   */
  void planAndLogOffset(const std::vector<geometry_msgs::msg::PoseStamped> *path)
  {
    double res = 0.05;
    double bot = -0.5;
    double top = 0.5;
    for (double x = bot; x <= top; x+=res)
    {
      for (double y = bot; y <= top; y+=res)
      {
        for (double z = bot; z <= top; z+=res)
        {
          auto msr = createMotionSequenceRequest(path, x, y, z);
          auto req = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
          req->request = msr;
          auto res = sequence_client_->async_send_request(req);
          auto ts = res.get()->response.planned_trajectories;
          //RCLCPP_INFO(this->get_logger(), "Got %ld trajectories", ts.size());
          std::string status = "";
          if (ts.size() > 0)
          {
            status = "success";
          }
          else
          {
            status = "failure";
          }
          status = status + "," + pointsToString(path,x,y,z);
          appendLineToFile("OUTPUT.csv", status);
        }
      }
    }
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

    auto msr = createMotionSequenceRequest(&goal->motion.path);
    RCLCPP_INFO(this->get_logger(), "Created MSR");

    //planAndLogOffset(&goal->motion.path);

    RCLCPP_INFO(this->get_logger(), "Creating req");
    auto req = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
    RCLCPP_INFO(this->get_logger(), "Setting msr request");
    req->request = msr;
    RCLCPP_INFO(this->get_logger(), "Sending request to sequence service");
    auto res = sequence_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Waiting for result");
    auto ts = res.get()->response.planned_trajectories;
    std::string status = "";
    if (ts.size() > 0)
    {
      status = "success";
      RCLCPP_INFO(this->get_logger(), "Got %ld trajectories", ts.size());
      RCLCPP_INFO(this->get_logger(), "Adding result to motion queue");

      //trajectory_queue.push(ts[0]);
      this->trajectory_pub->publish(ts[0]);

      // Set move_group_state to the last state of planned trajectory (planning of next trajectory starts there)
      robot_trajectory::RobotTrajectory rt(move_group.getRobotModel());
      rt.setRobotTrajectoryMsg(*move_group_state, ts[0]);
      move_group_state = rt.getLastWayPointPtr();

      status = status + "," + pointsToString(&goal->motion.path,0,0,0);
      appendLineToFile("OUTPUT.csv", status);

      result->result = "success";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      return;
    }

    status = "failure";
    status = status + "," + pointsToString(&goal->motion.path,0,0,0);
    appendLineToFile("OUTPUT.csv", status);

    RCLCPP_ERROR(this->get_logger(), "Planner failed to return trajectory in time");
    result->result = "failure";
    goal_handle->succeed(result);
    // abort prevents action from being called
    //goal_handle->abort(result);

    RCLCPP_ERROR(this->get_logger(), "Goal failed");
  }
};

class TrajectoryExecutor : public rclcpp::Node
{
  public:
    //trajectory_pub = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("lite6_trajectory", 10);
    rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr subscription;
    moveit::planning_interface::MoveGroupInterface move_group;
    TrajectoryExecutor()
      : Node("lite6_trajectory_executor"),
        move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
    {
      subscription = this->create_subscription<moveit_msgs::msg::RobotTrajectory>(
        "lite6_trajectory", 100, std::bind(&TrajectoryExecutor::trajectory_callback, this, std::placeholders::_1));
    }
    void trajectory_callback(const moveit_msgs::msg::RobotTrajectory msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received trajectory, executing");
      move_group.execute(msg);
      RCLCPP_INFO(this->get_logger(), "Finished executing trajectory");
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
  //executor.add_node(trajectory_executor);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
