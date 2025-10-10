#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import random
from typing import List, Tuple

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, OpaqueFunction
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

    land_base_model = os.path.join(tello_gazebo_share, 'models', 'land_base', 'model.sdf')

    # ======= REGRAS DE POSICIONAMENTO =======
    X_MIN, X_MAX = 0.0, 6.0
    Y_MIN, Y_MAX = 0.0, 6.0
    X_FORBID_MAX, Y_FORBID_MAX = 1.4, 1.4
    GROUND_Z = 0.14
    AIR_Z_MIN, AIR_Z_MAX = 1.0, 3.0
    MIN_DIST = 1.01
    FIXED_YAW = 0.0

    seed_env = os.environ.get('LAND_BASE_SEED')
    if seed_env:
        try:
            random.seed(int(seed_env))
        except ValueError:
            random.seed(seed_env)

    def in_bounds(x: float, y: float) -> bool:
        return (X_MIN <= x <= X_MAX) and (Y_MIN <= y <= Y_MAX)

    def in_forbidden(x: float, y: float) -> bool:
        return (X_MIN <= x <= X_FORBID_MAX) and (Y_MIN <= y <= Y_FORBID_MAX)

    def far_enough(x: float, y: float, placed: List[Tuple[float, float]]) -> bool:
        for (px, py) in placed:
            if math.hypot(x - px, y - py) <= MIN_DIST:
                return False
        return True

    def sample_xy(placed: List[Tuple[float, float]], max_tries: int = 5000) -> Tuple[float, float]:
        for _ in range(max_tries):
            x = random.uniform(X_MIN, X_MAX)
            y = random.uniform(Y_MIN, Y_MAX)
            if in_forbidden(x, y):
                continue
            if not in_bounds(x, y):
                continue
            if not far_enough(x, y, placed):
                continue
            return x, y
        raise RuntimeError("Não foi possível amostrar posições válidas sem overlap. Ajuste limites ou reduza MIN_DIST.")

    placed_xy: List[Tuple[float, float]] = []
    base_specs = []

    for i in range(1, 4):
        x, y = sample_xy(placed_xy)
        placed_xy.append((x, y))
        base_specs.append({'name': f'land_base_ground_{i}', 'x': x, 'y': y, 'z': GROUND_Z, 'yaw': FIXED_YAW})

    for i in range(1, 3):
        x, y = sample_xy(placed_xy)
        placed_xy.append((x, y))
        base_specs.append({'name': f'land_base_air_{i}', 'x': x, 'y': y, 'z': random.uniform(AIR_Z_MIN, AIR_Z_MAX), 'yaw': FIXED_YAW})

    for spec in base_specs:
        if not in_bounds(spec['x'], spec['y']):
            raise RuntimeError(f"Fora da área [0,6]x[0,6]: {spec['name']} -> ({spec['x']:.2f}, {spec['y']:.2f})")
        if in_forbidden(spec['x'], spec['y']):
            raise RuntimeError(f"Na zona proibida [0,1.4]x[0,1.4]: {spec['name']} -> ({spec['x']:.2f}, {spec['y']:.2f})")
        if spec['name'].startswith('land_base_ground') and abs(spec['z'] - GROUND_Z) > 1e-6:
            raise RuntimeError(f"Ground z deve ser {GROUND_Z}: {spec['name']} -> z={spec['z']:.3f}")

    for i in range(len(base_specs)):
        xi, yi = base_specs[i]['x'], base_specs[i]['y']
        for j in range(i + 1, len(base_specs)):
            xj, yj = base_specs[j]['x'], base_specs[j]['y']
            d = math.hypot(xi - xj, yi - yj)
            if d <= MIN_DIST:
                raise RuntimeError(f"Overlap detectado entre {base_specs[i]['name']} e {base_specs[j]['name']} (dist={d:.2f} m)")

    disable_db = SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value='')

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
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

    base_names = [spec['name'] for spec in base_specs]

    # >>> CORRIGIDO: passar '/model_states' para o nó publisher
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
    ])
