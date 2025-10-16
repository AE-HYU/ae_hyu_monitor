#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('ae_hyu_monitor')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'monitor_config.yaml')
    
    # Monitor node
    monitor_node = Node(
        package='ae_hyu_monitor',
        executable='monitor_node',
        name='monitor_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        monitor_node
    ])
