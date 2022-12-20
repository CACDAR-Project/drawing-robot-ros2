#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/execute_motion.hpp"

class RobotController : public rclcpp::Node
{
public:
  RobotController(std::string name) : Node(name)
  {
    this->service = this->create_service<robot_interfaces::srv::ExecuteMotion>("execute_path", std::bind(&RobotController::executePath, this, std::placeholders::_1, std::placeholders::_2));
  }

  /// Callback that executes path on robot
  virtual void executePath(const std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Request> request, std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Response> response)
  {
    //request->motion->path PoseStamped[]
    //request->motion->acceleration float64
    //request->motion->velocity float64
    RCLCPP_INFO(this->get_logger(), "NEW MOTION: Acceleration: ");
    //RCLCPP_INFO(this->get_logger(), "Acceleration: " + std::to_string(request.motion.acceleration));
    response->status = "executePath not implemented";
    RCLCPP_WARN(this->get_logger(), "executePath not implemented");
  }

private:
  rclcpp::Service<robot_interfaces::srv::ExecuteMotion>::SharedPtr service;
};

// A controller for a Dummy robot. Only logs messages and serves as an example for real implementation.
class DummyController : public RobotController
{
  public:

    DummyController(std::string name) : RobotController(name) {}

    virtual void executePath(const std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Request> request, std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Response> response)
    {
      //request->motion->path PoseStamped[]
      //request->motion->acceleration float64
      //request->motion->velocity float64
      RCLCPP_INFO(this->get_logger(), "NEW MOTION: Acceleration: ");
      //RCLCPP_INFO(this->get_logger(), "Acceleration: " + std::to_string(request.motion.acceleration));
      response->status = "executePath not implemented";
      RCLCPP_WARN(this->get_logger(), "AAAAAA executePath not implemented");
    }

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting dummy_controller");
  auto robot = std::make_shared<DummyController>("dummy");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(robot);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
