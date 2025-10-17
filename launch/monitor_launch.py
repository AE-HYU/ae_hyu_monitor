#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    mod_arg = DeclareLaunchArgument(
        'mod',
        default_value='',
        description='Mode: sim for simulation, empty for real'
    )
    
    # Get the launch directory
    pkg_dir = get_package_share_directory('ae_hyu_monitor')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'monitor_config.yaml')
    
    # Get mod value
    mod = LaunchConfiguration('mod')
    
    # Monitor node
    monitor_node = Node(
        package='ae_hyu_monitor',
        executable='monitor_node',
        name='monitor_node',
        parameters=[
            config_file,
            {
                'monitor/odom_topic': '/ego_racecar/odom'
            }
        ],
        condition=IfCondition(
            PythonExpression(['"', mod, '" == "sim"'])
        ),
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Monitor node for real mode
    monitor_node_real = Node(
        package='ae_hyu_monitor',
        executable='monitor_node',
        name='monitor_node',
        parameters=[
            config_file,
            {
                'monitor/odom_topic': '/odom'
            }
        ],
        condition=UnlessCondition(
            PythonExpression(['"', mod, '" == "sim"'])
        ),
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        mod_arg,
        monitor_node,
        monitor_node_real
    ])
