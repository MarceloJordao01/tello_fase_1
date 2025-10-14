#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # caminho padrão: <pkg_share>/config/base_localization.yaml
    default_params = PathJoinSubstitution([
        FindPackageShare('base_localization'),
        'config',
        'base_localization.yaml'
    ])

    # permite sobrescrever via --ros-args --params-file
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Caminho para o arquivo YAML de parâmetros do base_localization'
    )

    node = Node(
        package='base_localization',
        executable='base_localization',
        name='base_localization',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        # remappings opcionais (se quiser forçar por launch em vez de YAML):
        # remappings=[
        #     ('/base_detection/detected_coords', '/meu_topico/de_deteccao'),
        #     ('/base_detection/num_bases', '/meu_topico/num_bases'),
        #     ('/base_localization/poses', '/minha_saida/poses'),
        # ]
    )

    return LaunchDescription([
        params_file_arg,
        node
    ])
