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
    log_level = LaunchConfiguration("log_level", default='info')
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

    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gazebo_ros2_control/GazeboSystem')
    controllers_name = LaunchConfiguration('controllers_name', default='fake_controllers')
    moveit_controller_manager_key = LaunchConfiguration('moveit_controller_manager_key', default='moveit_simple_controller_manager')
    moveit_controller_manager_value = LaunchConfiguration('moveit_controller_manager_value', default='moveit_simple_controller_manager/MoveItSimpleControllerManager')

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
            package="lite6_controller",
            executable="lite6_controller",
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
            executable="drawing_surface.py",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

# ######################3

    #moveit_controller_manager_key = LaunchConfiguration('moveit_controller_manager_key', default='moveit_fake_controller_manager')
    #moveit_controller_manager_value = LaunchConfiguration('moveit_controller_manager_value', default='moveit_fake_controller_manager/MoveItFakeControllerManager')

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)


    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context))

    # robot_description_parameters
    # xarm_moveit_config/launch/lib/robot_moveit_config_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'robot_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        xacro_urdf_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
        xacro_srdf_file=PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'srdf', 'xarm.srdf.xacro']),
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
            'add_realsense_d435i': add_realsense_d435i,
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

    load_yaml = getattr(mod, 'load_yaml')
    controllers_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, '{}.yaml'.format(controllers_name.perform(context)))
    ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, 'ompl_planning.yaml')
    kinematics_yaml = robot_description_parameters['robot_description_kinematics']
    joint_limits_yaml = robot_description_parameters.get('robot_description_planning', None)

    # FIX acceleration limits
    for i in range(1,7):
        joint_limits_yaml['joint_limits']['joint{}'.format(i)]['has_acceleration_limits'] = True
        joint_limits_yaml['joint_limits']['joint{}'.format(i)]['max_acceleration'] = 0.5

    kinematics_yaml['kinematics_solver'] = 'kdl_kinematics_plugin/KDLKinematicsPlugin'
    #kinematics_yaml['kinematics_solver'] = 'lma_kinematics_plugin/LMAKinematicsPlugin'
    #kinematics_yaml['kinematics_solver_search_resolution'] = 0.005
    kinematics_yaml['kinematics_solver_timeout'] = 10.0
    kinematics_yaml['kinematics_solver_attempts'] = 10

    #print(joint_limits_yaml)
    #quit()

    #if add_gripper.perform(context) in ('True', 'true'):
    #    gripper_controllers_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), '{}.yaml'.format(controllers_name.perform(context)))
    #    gripper_ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), 'ompl_planning.yaml')
    #    gripper_joint_limits_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), 'joint_limits.yaml')

    #    if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
    #        for name in gripper_controllers_yaml['controller_names']:
    #            if name in gripper_controllers_yaml:
    #                if name not in controllers_yaml['controller_names']:
    #                    controllers_yaml['controller_names'].append(name)
    #                controllers_yaml[name] = gripper_controllers_yaml[name]
    #    if gripper_ompl_planning_yaml:
    #        ompl_planning_yaml.update(gripper_ompl_planning_yaml)
    #    if joint_limits_yaml and gripper_joint_limits_yaml:
    #        joint_limits_yaml['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])

    add_prefix_to_moveit_params = getattr(mod, 'add_prefix_to_moveit_params')
    add_prefix_to_moveit_params(
        controllers_yaml=controllers_yaml, ompl_planning_yaml=ompl_planning_yaml,
        kinematics_yaml=kinematics_yaml, joint_limits_yaml=joint_limits_yaml,
        prefix=prefix.perform(context))



    # Planning pipeline
    # https://github.com/AndrejOrsula/panda_moveit2_config/blob/master/launch/move_group_fake_control.launch.py
    #planning_pipeline = {
    #    "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
    #    "default_planning_pipeline": "pilz_industrial_motion_planner",
    #    "ompl": {
    #        "planning_plugin": "ompl_interface/OMPLPlanner",
    #        # TODO: Re-enable `default_planner_request_adapters/AddRuckigTrajectorySmoothing` once its issues are resolved
    #        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
    #        # TODO: Reduce start_state_max_bounds_error once spawning with specific joint configuration is enabled
    #        "start_state_max_bounds_error": 0.31416,
    #    },
    #    "pilz_industrial_motion_planner": {
    #        "planning_plugin": "pilz_industrial_motion_planner::CommandPlanner",
    #        "default_planner_config": "PTP",
    #        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService",
    #    },
    #}
    # Kinematics
    #kinematics = load_yaml('panda_moveit2_config', 'config/kinematics.yaml')





    # Planning Configuration
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    pilz_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
            # Disable AddTimeOptimalParameterization to fix motion blending https://github.com/ros-planning/moveit/issues/2905
            'request_adapters': """default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            #'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "default_planner_config": "PTP",
        }
    }

    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }



    # Moveit controllers Configuration
    moveit_controllers = {
        moveit_controller_manager_key.perform(context): controllers_yaml,
        'moveit_controller_manager': moveit_controller_manager_value.perform(context),
    }

    # Trajectory Execution Configuration
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False
    }

    plan_execution = {
        'plan_execution.record_trajectory_state_frequency': 10.0,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        # "planning_scene_monitor_options": {
        #     "name": "planning_scene_monitor",
        #     "robot_description": "robot_description",
        #     "joint_state_topic": "/joint_states",
        #     "attached_collision_object_topic": "/move_group/planning_scene_monitor",
        #     "publish_planning_scene_topic": "/move_group/publish_planning_scene",
        #     "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
        #     "wait_for_initial_state_timeout": 10.0,
        # },
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--log-level', 'debug'],
        parameters=[
            robot_description_parameters,
            #ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,
            move_group_capabilities,
            trajectory_execution,
            plan_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )

    # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'config', xarm_type, 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description_parameters,
            #ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'link_base'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [
        #RegisterEventHandler(event_handler=OnProcessExit(
        #    target_action=rviz2_node,
        #    on_exit=[EmitEvent(event=Shutdown())]
        #)),
        #rviz2_node,
        static_tf,
        move_group_node,
        robot_gazebo_launch,
    ] + nodes

# ######################3

    #return [
    #    robot_moveit_common_launch,
    #    robot_gazebo_launch,
    #] + nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
