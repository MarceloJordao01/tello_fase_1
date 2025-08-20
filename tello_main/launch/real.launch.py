#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tello_description_share = get_package_share_directory('tello_description')
    tello_main_share = get_package_share_directory('tello_main')

    urdf_path  = os.path.join(tello_description_share, 'urdf', 'tello_1.urdf')
    rviz_cfg   = os.path.join(tello_main_share, 'config', 'rviz.rviz')
    params_yaml = os.path.join(tello_main_share, 'config', 'params.yaml')

    return LaunchDescription([
        # TF estática: odom -> map (identidade; quaternion)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_map',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'map']
        ),

        # TF via URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ),

        # Nó principal (usa o driver real /drone1/tello_action)
        Node(
            package='tello_main',
            executable='navigator',
            output='screen',
            parameters=[params_yaml]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg]
        ),
    ])
