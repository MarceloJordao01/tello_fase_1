#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('base_localization')
    yaml_path = os.path.join(pkg_share, 'config', 'base_localization.yaml')

    return LaunchDescription([
        Node(
            package='base_localization',
            executable='base_localization',
            name='base_localization',
            output='screen',
            parameters=[yaml_path],
        )
    ])
