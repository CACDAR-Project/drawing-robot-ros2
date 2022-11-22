#!/usr/bin/env -S ros2 launch
"""Launch Python example for following a target"""

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    log_level = LaunchConfiguration("log_level", default='warn')
    rviz_config = LaunchConfiguration('rviz_config', default=os.path.join(get_package_share_directory("draw_svg"), "rviz", "ign_moveit2_examples.rviz"))

    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gazebo_ros2_control/GazeboSystem')

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    #geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_type = LaunchConfiguration('geometry_type', default='cylinder')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.05)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.005)
    geometry_length = LaunchConfiguration('geometry_length', default=0.07)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    load_controller = LaunchConfiguration('load_controller', default=True)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)


    ros2_control_plugin = 'gazebo_ros2_control/GazeboSystem'
    controllers_name = 'fake_controllers'
    moveit_controller_manager_key = 'moveit_simple_controller_manager'
    moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'

    # robot moveit common launch
    # xarm_moveit_config/launch/_robot_moveit_common.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            # 'add_gripper': add_gripper if robot_type.perform(context) == 'xarm' else 'false',
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'no_gui_ctrl': 'false',
            'ros2_control_plugin': ros2_control_plugin,
            'controllers_name': controllers_name,
            'moveit_controller_manager_key': moveit_controller_manager_key,
            'moveit_controller_manager_value': moveit_controller_manager_value,
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
            'use_sim_time': 'true'
        }.items(),
    )

    # robot gazebo launch
    # xarm_gazebo/launch/_robot_beside_table_gazebo.launch.py
    robot_gazebo_launch = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'launch', '_robot_beside_table_gazebo.launch.py'])),
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('draw_svg'), 'launch', 'robots', 'lite6_table.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'ros2_control_plugin': ros2_control_plugin,
            'load_controller': 'true',
        }.items(),
    )

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare('draw_svg'), 'urdf', 'xarm_pen.urdf.xacro']
            ),
            #PathJoinSubstitution(
            #    [
            #        FindPackageShare('xarm_description'),
            #        "urdf",
            #        "lite6",
            #        #"lite6.urdf.xacro",
            #        "lite6_robot_macro.xacro",
            #    ]
            #),
            " ",
            #"name:=", "lite6", " ",
            "prefix:=", " ",
            "hw_ns:=", hw_ns, " ",
            "limited:=", limited, " ",
            "effort_control:=", effort_control, " ",
            "velocity_control:=", velocity_control, " ",
            "add_gripper:=", add_gripper, " ",
            "add_vacuum_gripper:=", add_vacuum_gripper, " ",
            "dof:=", dof, " ",
            "robot_type:=", robot_type, " ",
            "ros2_control_plugin:=", ros2_control_plugin, " ",
            #"ros2_control_params:=", ros2_control_params, " ",

            "add_other_geometry:=", add_other_geometry, " ",
            "geometry_type:=", geometry_type, " ",
            "geometry_mass:=", geometry_mass, " ",
            "geometry_height:=", geometry_height, " ",
            "geometry_radius:=", geometry_radius, " ",
            "geometry_length:=", geometry_length, " ",
            "geometry_width:=", geometry_width, " ",
            "geometry_mesh_filename:=", geometry_mesh_filename, " ",
            "geometry_mesh_origin_xyz:=", geometry_mesh_origin_xyz, " ",
            "geometry_mesh_origin_rpy:=", geometry_mesh_origin_rpy, " ",
            "geometry_mesh_tcp_xyz:=", geometry_mesh_tcp_xyz, " ",
            "geometry_mesh_tcp_rpy:=", geometry_mesh_tcp_rpy, " ",

            #"robot_ip:=", robot_ip, " ",
            #"report_type:=", report_type, " ",
            #"baud_checkset:=", baud_checkset, " ",
            #"default_gripper_baud:=", default_gripper_baud, " ",
        ]
    )
    # TODO fix URDF loading
    # xacro urdf/xarm_pen.urdf.xacro prefix:= hw_ns:=xarm dof:=6 limited:=false effort_control:=false velocity_control:=false add_gripper:=false add_vacuum_gripper:=false robot_type:=lite ros2_control_plugin:=gazebo_ros2_control/GazeboSystem add_other_geometry:=false geometry_type:=cylinder geometry_mass:=0.05 geometry_height:=0.1 geometry_radius:=0.005 geometry_length:=0.07 geometry_width:=0.1 geometry_mesh_filename:= geometry_mesh_origin_xyz:="0 0 0" geometry_mesh_origin_rpy:="0 0 0" geometry_mesh_tcp_xyz:="0 0 0" geometry_mesh_tcp_rpy:="0 0 0"
    _robot_description_xml = Command(['cat ', PathJoinSubstitution([FindPackageShare('draw_svg'), 'urdf', 'lite6.tmp.urdf'])])
    robot_description = {"robot_description": _robot_description_xml}
    # SRDF
    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare('xarm_moveit_config'),
                    "srdf",
                    #"_lite6_macro.srdf.xacro",
                    "xarm.srdf.xacro",
                ]
            ),
            " ",
            #"name:=", "lite6", " ",
            "prefix:=", " ",
            #"hw_ns:=", hw_ns, " ",
            #"limited:=", limited, " ",
            #"effort_control:=", effort_control, " ",
            #"velocity_control:=", velocity_control, " ",
            #"add_gripper:=", add_gripper, " ",
            #"add_vacuum_gripper:=", add_vacuum_gripper, " ",
            "dof:=", dof, " ",
            "robot_type:=", robot_type, " ",
            #"ros2_control_plugin:=", ros2_control_plugin, " ",
            #"ros2_control_params:=", ros2_control_params, " ",

            #"add_other_geometry:=", add_other_geometry, " ",
            #"geometry_type:=", geometry_type, " ",
            #"geometry_mass:=", geometry_mass, " ",
            #"geometry_height:=", geometry_height, " ",
            #"geometry_radius:=", geometry_radius, " ",
            #"geometry_length:=", geometry_length, " ",
            #"geometry_width:=", geometry_width, " ",
            #"geometry_mesh_filename:=", geometry_mesh_filename, " ",
            #"geometry_mesh_origin_xyz:=", geometry_mesh_origin_xyz, " ",
            #"geometry_mesh_origin_rpy:=", geometry_mesh_origin_rpy, " ",
            #"geometry_mesh_tcp_xyz:=", geometry_mesh_tcp_xyz, " ",
            #"geometry_mesh_tcp_rpy:=", geometry_mesh_tcp_rpy, " ",

            #"robot_ip:=", robot_ip, " ",
            #"report_type:=", report_type, " ",
            #"baud_checkset:=", baud_checkset, " ",
            #"default_gripper_baud:=", default_gripper_baud, " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": _robot_description_semantic_xml
    }



    nodes = [
        Node(
            package="draw_svg",
            executable="follow",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="draw_svg",
            executable="draw_svg.py",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="draw_svg",
            executable="drawing_surface.py",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return [
        robot_moveit_common_launch,
        robot_gazebo_launch,
    ] + nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
