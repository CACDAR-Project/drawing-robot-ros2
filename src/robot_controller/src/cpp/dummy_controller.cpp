#include <cstdio>
#include <functional>
#include <memory>
#include <thread>

#include "robot_controller/robot_controller.hpp"
#include "robot_interfaces/action/execute_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

/**
 * A controller for a Dummy robot. Only logs messages and serves as an example for real implementation.
 */
class DummyController : public RobotController
{
  public:
  DummyController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : RobotController(options) {}

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

    for (auto p : goal->motion.path)
    {
      auto pp = p.pose;
      RCLCPP_INFO(this->get_logger(), "Position x:%f y:%f z:%f",pp.position.x,pp.position.y,pp.position.z);
      RCLCPP_INFO(this->get_logger(), "Orientation w:%f x:%f y:%f z:%f", pp.orientation.w, pp.orientation.x,pp.orientation.y,pp.orientation.z);
      //  W:%f X:%f Y:%f Z:%f
    }

    for (int i = 1; (i <= 10) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->result = feedback->status;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update status
      feedback->status = std::to_string(i) + "/10 complete";
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), feedback->status.c_str());

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = "success";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

/**
 *
 */
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
