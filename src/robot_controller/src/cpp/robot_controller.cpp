#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/execute_motion.hpp"

class RobotController : public rclcpp::Node
{
public:
  /// Constructor
  RobotController(std::string name)
  : Node(name)
  {
  //this->service = this->create_service<robot_interfaces::srv::ExecuteMotion>(name+"_execute_path", &this->executePath);
  this->service = this->create_service<robot_interfaces::srv::ExecuteMotion>(name+"_execute_path", std::bind(&RobotController::executePath, this, std::placeholders::_1, std::placeholders::_2));

  // Subscribe to target pose
  //target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

  //RCLCPP_INFO(this->get_logger(), "Initialization successful.");


  //std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  //rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service = node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  //rclcpp::spin(node);
  //rclcpp::shutdown();
  }

  //int (RobotController::*executePath)(const std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Request> request, std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Response> response);
void executePath(const std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Request> request, std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Response> response) const
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
  /// Callback that executes path on robot

  rclcpp::Service<robot_interfaces::srv::ExecuteMotion>::SharedPtr service;
};

// A controller for a Dummy robot. Only logs messages and serves as an example for real implementation.
// CRTP pattern used here https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
//class DummyController : public RobotController<DummyController>
class DummyController : public RobotController
{
  public:

    DummyController(std::string name) : RobotController(name) {}

    void executePath(const std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Request> request, std::shared_ptr<robot_interfaces::srv::ExecuteMotion::Response> response) const
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

//RobotController::RobotController(name) : Node(name)

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
