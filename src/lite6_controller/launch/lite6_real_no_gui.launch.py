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
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=True)

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

    # # robot driver launch
    # # xarm_api/launch/_robot_driver.launch.py
    # robot_driver_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_robot_driver.launch.py'])),
    #     launch_arguments={
    #         'robot_ip': robot_ip,
    #         'report_type': report_type,
    #         'dof': dof,
    #         'hw_ns': hw_ns,
    #         'add_gripper': add_gripper,
    #         'prefix': prefix,
    #         'baud_checkset': baud_checkset,
    #         'default_gripper_baud': default_gripper_baud,
    #         'robot_type': robot_type,
    #     }.items(),
    # )

    # robot description launch
    # xarm_description/launch/_robot_description.launch.py
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('custom_xarm_description'), 'launch', '_robot_description.launch.py'])),
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
            'joint_states_remapping': PathJoinSubstitution(['/', ros_namespace, hw_ns, 'joint_states']),
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
        }.items(),
    )

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

    load_yaml = getattr(mod, 'load_yaml')
    controllers_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, '{}.yaml'.format(controllers_name.perform(context)))
    ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, 'ompl_planning.yaml')
    kinematics_yaml = robot_description_parameters['robot_description_kinematics']
    joint_limits_yaml = robot_description_parameters.get('robot_description_planning', None)

    # FIX acceleration limits
    for i in range(1,7):
        joint_limits_yaml['joint_limits']['joint{}'.format(i)]['has_acceleration_limits'] = True
        joint_limits_yaml['joint_limits']['joint{}'.format(i)]['max_acceleration'] = 1.0

    if add_gripper.perform(context) in ('True', 'true'):
        gripper_controllers_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), '{}.yaml'.format(controllers_name.perform(context)))
        gripper_ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), 'ompl_planning.yaml')
        gripper_joint_limits_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), 'joint_limits.yaml')

        if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
            for name in gripper_controllers_yaml['controller_names']:
                if name in gripper_controllers_yaml:
                    if name not in controllers_yaml['controller_names']:
                        controllers_yaml['controller_names'].append(name)
                    controllers_yaml[name] = gripper_controllers_yaml[name]
        if gripper_ompl_planning_yaml:
            ompl_planning_yaml.update(gripper_ompl_planning_yaml)
        if joint_limits_yaml and gripper_joint_limits_yaml:
            joint_limits_yaml['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])

    add_prefix_to_moveit_params = getattr(mod, 'add_prefix_to_moveit_params')
    add_prefix_to_moveit_params(
        controllers_yaml=controllers_yaml, ompl_planning_yaml=ompl_planning_yaml,
        kinematics_yaml=kinematics_yaml, joint_limits_yaml=joint_limits_yaml,
        prefix=prefix.perform(context))

    cartesian_limits = load_yaml(moveit_config_package_name, 'config', xarm_type, 'cartesian_limits.yaml')
    robot_description_parameters['robot_description_planning']['cartesian_limits'] = cartesian_limits['cartesian_limits']

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
            #'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'request_adapters': """default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
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
            ompl_planning_pipeline_config,
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
    )


    # joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['{}/joint_states'.format(hw_ns.perform(context))]}],
        remappings=[
            ('follow_joint_trajectory', '{}{}_traj_controller/follow_joint_trajectory'.format(prefix.perform(context), xarm_type)),
        ],
    )

    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'baud_checkset': baud_checkset,
            'default_gripper_baud': default_gripper_baud,
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
        }.items(),
    )

    control_node = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            '{}{}_traj_controller'.format(prefix.perform(context), xarm_type),
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    nodes = [
        Node(
            package="lite6_controller",
            executable="lite6_controller",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                #robot_description_parameters['robot_description'],
                #robot_description_parameters['robot_description_semantic'],
                #robot_description_parameters['robot_description_planning'],
                #robot_description_parameters['robot_description_kinematics'],
                robot_description_parameters,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]

    return [

        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=nodes[0],
            on_exit=[EmitEvent(event=Shutdown())]
        )),
        #rviz2_node,
        static_tf,
        move_group_node,


        robot_description_launch,
        joint_state_publisher_node,
        ros2_control_launch,
        control_node,
        # robot_driver_launch,
    ] + nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
