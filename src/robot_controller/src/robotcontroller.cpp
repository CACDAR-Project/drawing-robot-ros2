#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class RobotController : public rclcpp::Node
{
public:
  /// Constructor
  RobotController();

private:
  /// Callback that executes path on robot
    virtual void add(const std::shared_ptr<robot_interfaces::srv::AddThreeInts::Request> request,
                           std::shared_ptr<robot_interfaces::srv::AddThreeInts::Response>       response);
};

RobotController::RobotController(string name) : Node(name)
{
  // Subscribe to target pose
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

int main(int argc, char ** argv)
{
  //rclcpp::init(argc, argv);

  //auto target_follower = std::make_shared<MoveItFollowTarget>();

  //rclcpp::executors::SingleThreadedExecutor executor;
  //executor.add_node(target_follower);
  //executor.spin();

  //rclcpp::shutdown();
  //return EXIT_SUCCESS;
  (void) argc;
  (void) argv;

  printf("hello world\n");
  return 0;
}
