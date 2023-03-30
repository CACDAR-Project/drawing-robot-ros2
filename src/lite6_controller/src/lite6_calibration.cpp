#include <signal.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>
#include <iostream>
#include <string>


void exit_sig_handler(int signum)
{
    fprintf(stderr, "[lite6_calibration] Ctrl-C caught, exit process...\n");
    exit(-1);
}

void wait()
{
 do
 {
   std::cout << '\n' << "Press a key to continue...";
 } while (std::cin.get() != '\n');
}

void println(std::string s) {
  std::cout << s << std::endl;
}

void print_joints(std::vector<float> jnts) {
    for (auto i : jnts)
      std::cout << i << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string hw_ns = "ufactory";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lite6_calibration");
    int ret;

    signal(SIGINT, exit_sig_handler);

    xarm_api::XArmROSClient client;
    client.init(node, hw_ns);
    client.motion_enable(true);

    println("Setting xArm lite6 to free-drive mode. Attach pen and touch the drawing surface.");
    client.set_mode(2);
    client.set_state(0);
    //rclcpp::sleep_for(std::chrono::seconds(20));
    wait();
    //client.set_mode(0);
    //client.set_state(0);
    //rclcpp::sleep_for(std::chrono::seconds(2));

    client.set_mode(0);
    client.set_state(0);
    rclcpp::sleep_for(std::chrono::seconds(2));

    std::vector<float> jnt_drawing_pose = {-0.813972, -0.0103697, 0.554617, 3.09979, -0.627334, -0.397301, 0};

    std::vector<float> jnt_pose = { 0, 0, 0, 0, 0, 0, 0 };
    client.get_servo_angle(jnt_pose);
    println("Moving to start drawing pose");
//XArmROSClient::set_servo_angle(const std::vector<fp32>& angles, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius)
    client.set_servo_angle(jnt_drawing_pose, 1, 1, 1, true, 100, -1);
    //client.set_servo_angle_j(jnt_drawing_pose, 1, 1, 100);
    //rclcpp::sleep_for(std::chrono::seconds(5));



    client.set_mode(0);
    client.set_state(0);
    return 0;

    client.set_mode(4);
    client.set_state(0);
    std::vector<float> jnt_v = { 1, 0, 0, 0, 0, 0, 0 };
    ret = client.vc_set_joint_velocity(jnt_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_joint: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    jnt_v[0] = -1;
    ret = client.vc_set_joint_velocity(jnt_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_joint: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    // stop
    jnt_v[0] = 0;
    ret = client.vc_set_joint_velocity(jnt_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_joint: %d", ret);

    std::vector<float> line_v = { 1, 0, 0, 0, 0, 0};
    client.set_mode(5);
	client.set_state(0);
    ret = client.vc_set_cartesian_velocity(line_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_line: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    line_v[0] = -1;
    ret = client.vc_set_cartesian_velocity(line_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_line: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    // stop
    line_v[0] = 0;
    ret = client.vc_set_cartesian_velocity(line_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_line: %d", ret);

    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "test_xarm_velo_move over");
    return 0;
}
