#include <cstdio>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <chrono>

#include "robot_controller/robot_controller.hpp"
#include "robot_interfaces/action/execute_motion.hpp"
#include "robot_interfaces/msg/points.hpp"
#include "robot_interfaces/srv/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

/**
 * Controller for the axidraw robot. Calls axidraw python API over ROS2 topics
 */
class AxidrawController : public RobotController
{
  rclcpp::Client<robot_interfaces::srv::Status>::SharedPtr status_client;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr move_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr penup_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pendown_pub;
  rclcpp::Publisher<robot_interfaces::msg::Points>::SharedPtr path_pub;

  public:
  AxidrawController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : RobotController(options)
  {
    status_client = this->create_client<robot_interfaces::srv::Status>("axidraw_status");

    while (!status_client->wait_for_service(5s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for axidraw_status");
        //return 0;
      }
      RCLCPP_WARN(this->get_logger(), "Failed to connect to axidraw_status, retrying");
    }
    RCLCPP_INFO(this->get_logger(), "Successfully connected to axidraw_status, Axidraw is ready");

    // Create publishers for axidraw_serial.py topics
    move_pub = this->create_publisher<geometry_msgs::msg::Point>("axidraw_move", 10);
    penup_pub = this->create_publisher<std_msgs::msg::Empty>("axidraw_penup", 10);
    pendown_pub = this->create_publisher<std_msgs::msg::Empty>("axidraw_pendown", 10);
    path_pub = this->create_publisher<robot_interfaces::msg::Points>("axidraw_path", 10);
  }

  /**
   * Return true if axidraw is ready
   */
  bool is_ready()
  {
    auto request = std::make_shared<robot_interfaces::srv::Status::Request>();
    request->resource = "motion";
    auto result = status_client->async_send_request(request);

    result.wait();
    std::string res_string = result.get()->status;
    RCLCPP_INFO(this->get_logger(), "Called is_ready(), status: %s", res_string.c_str());
    return res_string == "ready";
  }

  // Set limits for A4 paper: 297x210mm
  float xlim = 297;
  float ylim = 210;

  /**
   * Function that translates an input value with a given range to a value within another range.
   */
  float translate(float val, float lmin, float lmax, float rmin, float rmax)
  {
    float lspan = lmax - lmin;
    float rspan = rmax - rmin;
    val = (val - lmin) / lspan;
    return rmin + (val * rspan);
  }

  /**
   * Translate a pose to axidraw coordinates (mm)
   */
  geometry_msgs::msg::Point translate_pose(geometry_msgs::msg::PoseStamped ps)
  {
    auto pose = ps.pose;
    auto point = geometry_msgs::msg::Point();

    point.x = translate(pose.position.x, 0, 1, 0, xlim);
    point.y = translate(pose.position.y, 0, 1, 0, ylim);

    // Axidraw z-axis has 2 states, pen up is 1, pen down is 0.
    point.z = pose.position.z > 0 ? 1 : 0;

    return point;
  }

  // Translate all poses in a vector
  std::vector<geometry_msgs::msg::Point> translate_poses(std::vector<geometry_msgs::msg::PoseStamped> ps)
  {
    std::vector<geometry_msgs::msg::Point> points;
    for (auto p : ps)
      points.push_back(translate_pose(p));
    return points;
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

    //for (auto p : goal->motion.path)
    //{
    //  auto pp = p.pose;
    //  RCLCPP_INFO(this->get_logger(), "Position x:%f y:%f z:%f",pp.position.x,pp.position.y,pp.position.z);
    //  RCLCPP_INFO(this->get_logger(), "Orientation w:%f x:%f y:%f z:%f", pp.orientation.w, pp.orientation.x,pp.orientation.y,pp.orientation.z);
    //  //  W:%f X:%f Y:%f Z:%f
    //}

    auto points = translate_poses(goal->motion.path);

    for (long unsigned int i = 0; (i < points.size()) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        result->result = feedback->status;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      auto p = points[i];

      // Ensure axidraw is not busy
      while (!is_ready())
      {
        feedback->status = std::to_string(i+1) + "/" + std::to_string(points.size()) + " waiting for previous motion";
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
      }

      if (p.z > 0)
        this->penup_pub->publish(std_msgs::msg::Empty());
      else
        this->pendown_pub->publish(std_msgs::msg::Empty());

      // Wait for pen movement to complete
      while (!is_ready())
      {
        feedback->status = std::to_string(i+1) + "/" + std::to_string(points.size()) + " waiting for current motion";
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
      }

      this->move_pub->publish(p);

      // Update status
      feedback->status = std::to_string(i+1) + "/" + std::to_string(points.size()) + " complete";
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), feedback->status.c_str());

    }

    // Check if goal is done
    if (rclcpp::ok())
    {
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

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting axidraw_controller");
  auto robot = std::make_shared<AxidrawController>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(robot);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
