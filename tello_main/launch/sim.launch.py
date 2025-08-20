#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ns = 'drone1'
    tello_gazebo_share = get_package_share_directory('tello_gazebo')
    tello_description_share = get_package_share_directory('tello_description')
    tello_main_share = get_package_share_directory('tello_main')

    world_path = os.path.join(tello_gazebo_share, 'worlds', 'arena_world.world')
    urdf_path  = os.path.join(tello_description_share, 'urdf', 'tello_1.urdf')
    rviz_cfg   = os.path.join(tello_main_share, 'config', 'rviz.rviz')
    params_yaml = os.path.join(tello_main_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 1) Desabilita o banco online de modelos
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),

        # 2) Gazebo Classic + gazebo_ros (SEM -u / --pause)
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_path
            ],
            output='screen'
        ),

        # 3) Spawn do Tello na simulação
        Node(
            package='tello_gazebo',
            executable='inject_entity.py',
            output='screen',
            arguments=[urdf_path, '0', '0', '1', '0']
        ),

        # 4) TF estática: odom -> map (identidade; quaternion)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_map',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'map']
        ),

        # 5) TF via URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ),

        # 6) Nó principal (lendo params do YAML)
        Node(
            package='tello_main',
            executable='navigator',
            output='screen',
            parameters=[params_yaml]
        ),

        # 7) RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg]
        ),
    ])
