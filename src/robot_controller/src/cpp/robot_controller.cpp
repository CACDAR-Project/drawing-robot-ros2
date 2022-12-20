#include <cstdio>
#include <functional>
#include <memory>
#include <thread>

#include "robot_interfaces/action/execute_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

//#include "robot_interfaces/visibility_control.h"

//
// https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
class RobotController : public rclcpp::Node
{
public:
  using ExecuteMotion = robot_interfaces::action::ExecuteMotion;
  using GoalHandleExecuteMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;

  explicit RobotController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_controller",options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ExecuteMotion>(
      this,
      "execute_motion",
      std::bind(&RobotController::motion_handle_goal, this, _1, _2),
      std::bind(&RobotController::motion_handle_cancel, this, _1),
      std::bind(&RobotController::motion_handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;

  virtual rclcpp_action::GoalResponse motion_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteMotion::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with acceleration %f", goal->motion.acceleration);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  virtual rclcpp_action::CancelResponse motion_handle_cancel(
    const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  virtual void motion_handle_accepted(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotController::executePath, this, _1), goal_handle}.detach();
  }

  /// Callback that executes path on robot
  virtual void executePath(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecuteMotion::Feedback>();
    auto result = std::make_shared<ExecuteMotion::Result>();
    std::string msg = "executePath not implemented";
    result->result = msg;
    feedback->status = msg;
    RCLCPP_WARN(this->get_logger(), msg.c_str());
  }
};

// A controller for a Dummy robot. Only logs messages and serves as an example for real implementation.
class DummyController : public RobotController
{
  public:
  DummyController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : RobotController(options) {}

  /// Callback that executes path on robot
  virtual void executePath(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotion>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<robot_interfaces::action::ExecuteMotion::Feedback>();
    auto status = feedback->status;
    auto result = std::make_shared<robot_interfaces::action::ExecuteMotion::Result>();


    for (int i = 1; (i < 100) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->result = status;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update status
      status = i + "/100 complete";
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = status;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting dummy_controller");
  auto robot = std::make_shared<DummyController>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(robot);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
