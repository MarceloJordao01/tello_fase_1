#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import random
from typing import List, Tuple

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ns = 'drone1'

    # Shares
    tello_gazebo_share      = get_package_share_directory('tello_gazebo')
    tello_description_share = get_package_share_directory('tello_description')
    tello_main_share        = get_package_share_directory('tello_main')
    base_loc_share          = get_package_share_directory('base_localization')

    # Paths
    world_path  = os.path.join(tello_gazebo_share, 'worlds', 'arena_world.world')
    urdf_path   = os.path.join(tello_description_share, 'urdf', 'tello_1.urdf')
    rviz_cfg    = os.path.join(tello_main_share, 'config', 'rviz.rviz')
    params_yaml = os.path.join(tello_main_share, 'config', 'params.yaml')

    # base_localization.yaml (para o calib ler cx_ref, cy_ref, D0)
    base_loc_yaml = os.path.join(base_loc_share, 'config', 'base_localization.yaml')

    # (opcional) CSV de saída do calib
    calib_csv_path = os.path.join(tello_main_share, 'calib_minimal.csv')

    # Modelo da base
    land_base_model = os.path.join(tello_gazebo_share, 'models', 'land_base', 'model.sdf')

    # ======= REGIÃO PARA SPAWN APENAS 1 BASE =======
    # Retângulo com cantos (1.37, 6.5) e (5.0, 1.5)
    X_MIN, X_MAX = 1.37, 5.0
    Y_MIN, Y_MAX = 1.5,  6.5

    # Altura aleatória entre 0.14 e 2.0
    Z_MIN, Z_MAX = 0.14, 2.0

    FIXED_YAW = 0.0  # pode manter fixo

    # Semente opcional para reprodutibilidade
    seed_env = os.environ.get('LAND_BASE_SEED')
    if seed_env:
        try:
            random.seed(int(seed_env))
        except ValueError:
            random.seed(seed_env)

    def sample_single_xy() -> Tuple[float, float]:
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        return x, y

    # Gera especificações de UMA única base
    bx, by = sample_single_xy()
    bz = random.uniform(Z_MIN, Z_MAX)
    base_specs = [{
        'name': 'land_base_single',
        'x': bx,
        'y': by,
        'z': bz,
        'yaw': FIXED_YAW
    }]

    # Sanidade (dentro do retângulo)
    for spec in base_specs:
        if not (X_MIN <= spec['x'] <= X_MAX) or not (Y_MIN <= spec['y'] <= Y_MAX):
            raise RuntimeError(
                f"Base fora do retângulo definido: {spec['name']} -> "
                f"({spec['x']:.2f}, {spec['y']:.2f})"
            )
        if not (Z_MIN <= spec['z'] <= Z_MAX):
            raise RuntimeError(
                f"Altura fora do intervalo [{Z_MIN}, {Z_MAX}]: {spec['name']} -> z={spec['z']:.3f}"
            )

    # ======= Processo Gazebo (SEM GUI) =======
    # Evita buscar modelos na internet
    disable_db = SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value='')

    # Use apenas o servidor (gzserver) para não abrir interface gráfica
    gazebo = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    )

    inject_tello = Node(
        package='tello_gazebo',
        executable='inject_entity.py',
        output='screen',
        arguments=[urdf_path, '0', '0', '1', '0']
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'map']
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[urdf_path]
    )

    navigator = Node(
        package='tello_main',
        executable='navigator',
        output='screen',
        parameters=[params_yaml]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    def make_spawn_nodes(context):
        nodes = []
        for spec in base_specs:
            nodes.append(
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name=f"spawn_{spec['name']}",
                    output='screen',
                    arguments=[
                        '-entity', spec['name'],
                        '-file', land_base_model,
                        '-x', f"{spec['x']:.3f}",
                        '-y', f"{spec['y']:.3f}",
                        '-z', f"{spec['z']:.3f}",
                        '-Y', f"{spec['yaw']:.6f}",
                    ]
                )
            )
        return nodes

    spawn_after_gazebo = TimerAction(
        period=2.5,
        actions=[OpaqueFunction(function=lambda ctx: make_spawn_nodes(ctx))]
    )

    # Lista com único nome de base
    base_names = [spec['name'] for spec in base_specs]

    # Publisher de PoseArray das bases
    land_bases_pose_pub_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='tello_main',
                executable='land_bases_pose_publisher',
                name='land_bases_pose_publisher',
                output='screen',
                parameters=[{
                    'base_names': base_names,
                    'output_topic': '/land_bases/pose_array',
                    'names_topic': '/land_bases/names',
                    'frame_id': 'map',
                    'model_states_topic': '/model_states',
                }]
            )
        ]
    )

    # ======= INCLUDES dos launches externos =======
    base_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('base_detection'),
                'launch',
                'base_detection.launch.py'
            )
        )
    )


    # ======= Nó de calibração (target_frame = base_link_1) =======
    calib_node = Node(
        package='base_localization',                  # ajuste se seu executável estiver em outro package
        executable='calib_data_logger_posearray',     # entrypoint do script de calib
        name='calib_data_logger_posearray',
        output='screen',
        parameters=[{
            'detection_topic': '/base_detection/detected_coords',
            'gt_pose_array_topic': '/land_bases/pose_array',
            'reference_frame': 'map',
            'target_frame': 'base_link_1',  
            'tf_timeout_sec': 0.3,
            'base_localization_yaml_path': base_loc_yaml,
            'csv_path': calib_csv_path,
            'min_score': 0.0,
        }]
    )

    # ======= Disparo automático da missão após ~10s =======
    start_auto_after_10s = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/tello_navigator/start_auto',
                    'std_srvs/srv/Trigger',
                    '{}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        disable_db,
        gazebo,                   
        inject_tello,
        static_tf,
        rsp,
        navigator,
        rviz,
        spawn_after_gazebo,
        land_bases_pose_pub_node,
        base_detection_launch,
        calib_node,
        start_auto_after_10s,
    ])
