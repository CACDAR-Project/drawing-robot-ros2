import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.150')
    report_type = LaunchConfiguration('report_type', default='normal')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotSystemHardware'
    controllers_name = LaunchConfiguration('controllers_name', default='controllers')
    moveit_controller_manager_key = LaunchConfiguration('moveit_controller_manager_key', default='moveit_simple_controller_manager')
    moveit_controller_manager_value = LaunchConfiguration('moveit_controller_manager_value', default='moveit_simple_controller_manager/MoveItSimpleControllerManager')

    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context))
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    log_level = LaunchConfiguration("log_level", default='warn')

    # robot_description_parameters
    # xarm_moveit_config/launch/lib/robot_moveit_config_lib.py
    moveit_config_package_name = 'custom_xarm_moveit_config'
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'robot_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        xacro_urdf_file=PathJoinSubstitution([FindPackageShare('custom_xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
        xacro_srdf_file=PathJoinSubstitution([FindPackageShare('custom_xarm_moveit_config'), 'srdf', 'xarm.srdf.xacro']),
        urdf_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns.perform(context).strip('/'),
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'ros2_control_plugin': ros2_control_plugin,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
        },
        srdf_arguments={
            'prefix': prefix,
            'dof': dof,
            'robot_type': robot_type,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_other_geometry': add_other_geometry,
        },
        arguments={
            'context': context,
            'xarm_type': xarm_type,
        }
    )
    calibration_node = Node(
            package="lite6_controller",
            executable="lite6_calibration",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description_parameters,
                {"use_sim_time": use_sim_time},
            ],
        )

    return [
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=calibration_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )),
        calibration_node
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
