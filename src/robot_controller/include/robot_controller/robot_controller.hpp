#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "robot_interfaces/action/execute_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class RobotController : public rclcpp::Node
{
public:
  using ExecuteMotion = robot_interfaces::action::ExecuteMotion;
  using GoalHandleExecuteMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;

  explicit RobotController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;

  virtual rclcpp_action::GoalResponse motion_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteMotion::Goal> goal);

  virtual rclcpp_action::CancelResponse motion_handle_cancel(
    const std::shared_ptr<GoalHandleExecuteMotion> goal_handle);

  virtual void motion_handle_accepted(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle);

  /// Callback that executes path on robot
  virtual void executePath(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle);
};

using ExecuteMotion = robot_interfaces::action::ExecuteMotion;
using GoalHandleExecuteMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;

RobotController::RobotController(const rclcpp::NodeOptions & options)
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

rclcpp_action::GoalResponse RobotController::motion_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteMotion::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with acceleration %f", goal->motion.acceleration);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotController::motion_handle_cancel(
  const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotController::motion_handle_accepted(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&RobotController::executePath, this, _1), goal_handle}.detach();
}

/// Callback that executes path on robot
void RobotController::executePath(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExecuteMotion::Feedback>();
  auto result = std::make_shared<ExecuteMotion::Result>();
  std::string msg = "executePath not implemented";
  result->result = msg;
  feedback->status = msg;
  RCLCPP_WARN(this->get_logger(), msg.c_str());
}

#endif
