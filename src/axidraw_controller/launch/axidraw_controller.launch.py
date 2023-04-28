import os
import yaml
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
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    log_level = LaunchConfiguration("log_level", default='info')

    config = LaunchConfiguration("config", default=PathJoinSubstitution([FindPackageShare('axidraw_controller'), 'config', 'config.yaml']))

    robotcontroller_config = config.perform(context)
    with open(robotcontroller_config, 'r') as f:
        axidraw_config = yaml.safe_load(f)['/axidraw_serial']['ros__parameters']
        print(f'Loaded configuration: {axidraw_config}')

    nodes = [
        Node(
            package="axidraw_controller",
            executable="axidraw_controller",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
            ],
        ),
        Node(
            package="axidraw_controller",
            executable="axidraw_serial.py",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                {"serial_port": serial_port},
                axidraw_config,
            ],
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
