#include <signal.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>
#include <iostream>
#include <string>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


const std::string MOVE_GROUP = "lite6";

std::shared_ptr<rclcpp::Node> node;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[lite6_calibration] Ctrl-C caught, exit process...\n");
    exit(-1);
}

void wait()
{
 do
 {
   std::cout << "Press a key to continue...";
 } while (std::cin.get() != '\n');
}

void println(std::string s)
{
  std::cout << s << std::endl;
}

void print_joints(std::vector<float> jnts)
{
    std::string out = "";
    out = out + "{ ";
    for (auto i : jnts)
      out = out + std::to_string(i) + ", ";
    out = out.substr(0, out.size()-2); //remove trailing comma
    out = out + " }";
    std::cout << out << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string hw_ns = "ufactory";
    node = rclcpp::Node::make_shared("lite6_calibration");
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

    //std::vector<float> jnt_drawing_pose = { -0.975004, 0.0734182, 0.443928, 3.14102, -0.370552, -0.85577, 0 };
    std::vector<float> jnt_drawing_pose = { -0.975008, 0.00889134, 0.453255, 3.1414, -0.444295, -0.85692, 0 } ;

    std::vector<float> jnt_pose = { 0, 0, 0, 0, 0, 0, 0 };
    client.get_servo_angle(jnt_pose);

    print_joints(jnt_pose);



    //client.set_servo_angle_j(jnt_drawing_pose, 1, 1, 100);
    //rclcpp::sleep_for(std::chrono::seconds(5));


    // Compute position of pen from joint state
    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(MOVE_GROUP);
    std::string ee_link = "pen_link";

    std::vector<double> jnts;
    for (auto j : jnt_pose)
        jnts.push_back(j);

    jnts.resize(joint_model_group->getVariableNames().size());
    kinematic_state->setJointGroupPositions(joint_model_group, jnts);

    // from tutorial https://ros-planning.github.io/moveit_tutorials/doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html
    // https://github.com/ros-planning/moveit_tutorials/blob/master/doc/robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp
    //robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");

    kinematic_state->setJointGroupPositions(joint_model_group, jnts);
    // https://mycourses.aalto.fi/pluginfile.php/1433193/mod_resource/content/2/Intro_to_ROS_Eigen.pdf
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(ee_link);

    auto x = std::to_string(end_effector_state.translation().x());
    auto y = std::to_string(end_effector_state.translation().y());
    auto z = std::to_string(end_effector_state.translation().z());
    //println("x for '" + ee_link + "': " + x);
    //println("y for '" + ee_link + "': " + y);
    println("z-offset value for '" + ee_link + "' (use this value for the current pen): " + z);


    println("Moving to start drawing pose");
    wait();
    client.set_servo_angle(jnt_drawing_pose, 1, 1, 1, true, 100, -1);


    client.set_mode(0);
    client.set_state(0);

    rclcpp::shutdown();

    println("Done, ignore any errors after this");
    wait();
    return 0;
}
