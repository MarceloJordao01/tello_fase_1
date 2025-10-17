#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import random
from typing import List, Tuple

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _workspace_root_from_share() -> str:
    """
    Toma o share do package base_localization:
      <ws>/install/base_localization/share/base_localization
    e volta 4 níveis para chegar em <ws> (raiz com 'src' e 'install').
    Fallback: cwd.
    """
    try:
        share_dir = get_package_share_directory('base_localization')
        ws_root = os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..'))
        if not os.path.isdir(ws_root):
            return os.getcwd()
        return ws_root
    except Exception:
        return os.getcwd()


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

    # ===== Args =====
    # YAML do base_localization para ler cx_ref/cy_ref/D0 no nó de calibração
    default_base_loc_yaml = PathJoinSubstitution([
        FindPackageShare('base_localization'),
        'config',
        'base_localization.yaml'
    ])
    base_loc_params_arg = DeclareLaunchArgument(
        'base_loc_params_file',
        default_value=default_base_loc_yaml,
        description='Caminho para o base_localization.yaml a ser usado pela calibração (para ler cx_ref/cy_ref/D0).'
    )

    # Flag para abrir RViz (default: true)
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Se "true", abre o RViz com o config padrão.'
    )
    use_rviz_cfg = LaunchConfiguration('use_rviz')

    # NOVO: Flag para abrir a GUI do Gazebo (gzclient). Default: false
    use_gz_gui_arg = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='false',
        description='Se "true", abre o gzclient (GUI do Gazebo).'
    )
    use_gz_gui_cfg = LaunchConfiguration('use_gazebo_gui')

    # NOVO: Flags para posição fixa da base (ou aleatória)
    base_fixed_arg = DeclareLaunchArgument(
        'base_fixed',
        default_value='false',
        description='Se "true", usa posição fixa da base (base_x, base_y, base_z, base_yaw).'
    )
    base_fixed_cfg = LaunchConfiguration('base_fixed')

    base_x_arg = DeclareLaunchArgument(
        'base_x',
        default_value='2.5',
        description='X fixo da base quando base_fixed=true.'
    )
    base_y_arg = DeclareLaunchArgument(
        'base_y',
        default_value='3.0',
        description='Y fixo da base quando base_fixed=true.'
    )
    base_z_arg = DeclareLaunchArgument(
        'base_z',
        default_value='0.5',
        description='Z fixo da base quando base_fixed=true.'
    )
    base_yaw_arg = DeclareLaunchArgument(
        'base_yaw',
        default_value='0.0',
        description='Yaw fixo da base quando base_fixed=true (radianos).'
    )

    base_x_cfg = LaunchConfiguration('base_x')
    base_y_cfg = LaunchConfiguration('base_y')
    base_z_cfg = LaunchConfiguration('base_z')
    base_yaw_cfg = LaunchConfiguration('base_yaw')

    # ===== CSV de saída na RAIZ do workspace =====
    calib_csv_path = os.path.join(_workspace_root_from_share(), 'calib_minimal.csv')

    # Modelo da base
    land_base_model = os.path.join(tello_gazebo_share, 'models', 'land_base', 'model.sdf')

    # ======= RETÂNGULO PARA POSIÇÃO ALEATÓRIA (caso base_fixed=false) =======
    # Retângulo com cantos (1.37, 6.5) e (5.0, 1.5)
    X_MIN, X_MAX = 2.00, 4.10
    Y_MIN, Y_MAX = 2.40, 5.65

    # Altura aleatória entre 0.14 e 2.0
    Z_MIN, Z_MAX = 0.14, 2.0

    # Semente opcional para reprodutibilidade
    seed_env = os.environ.get('LAND_BASE_SEED')
    if seed_env:
        try:
            random.seed(int(seed_env))
        except ValueError:
            random.seed(seed_env)

    # ======= Processo Gazebo (SEM GUI por padrão) =======
    # Evita buscar modelos na internet
    disable_db = SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value='')

    # Servidor do Gazebo (gzserver)
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    )

    # GUI do Gazebo (gzclient) - condicional
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(use_gz_gui_cfg)
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

    # RViz condicional
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz_cfg),
    )

    # Nome fixo da base
    base_name = 'land_base_single'

    # Função para gerar o nó de spawn com base em flags/aleatoriedade
    def make_spawn_nodes(context):
        def _as_bool(val: str) -> bool:
            return str(val).strip().lower() in ('1', 'true', 'yes', 'y', 't')

        is_fixed = _as_bool(context.perform_substitution(base_fixed_cfg))

        if is_fixed:
            try:
                bx = float(context.perform_substitution(base_x_cfg))
                by = float(context.perform_substitution(base_y_cfg))
                bz = float(context.perform_substitution(base_z_cfg))
                byaw = float(context.perform_substitution(base_yaw_cfg))
            except Exception:
                # Fallback para valores default caso parsing falhe
                bx, by, bz, byaw = 2.5, 3.0, 0.5, 0.0
        else:
            bx = random.uniform(X_MIN, X_MAX)
            by = random.uniform(Y_MIN, Y_MAX)
            bz = random.uniform(Z_MIN, Z_MAX)
            byaw = 0.0

        # Sanidade (se aleatório) — se estiver fora, trunca para os limites
        bx = min(max(bx, X_MIN), X_MAX)
        by = min(max(by, Y_MIN), Y_MAX)
        bz = min(max(bz, Z_MIN), Z_MAX)

        return [
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f"spawn_{base_name}",
                output='screen',
                arguments=[
                    '-entity', base_name,
                    '-file', land_base_model,
                    '-x', f"{bx:.3f}",
                    '-y', f"{by:.3f}",
                    '-z', f"{bz:.3f}",
                    '-Y', f"{byaw:.6f}",
                ]
            )
        ]

    # Spawn após o Gazebo subir
    spawn_after_gazebo = TimerAction(
        period=2.5,
        actions=[OpaqueFunction(function=lambda ctx: make_spawn_nodes(ctx))]
    )

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
                    'base_names': [base_name],
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
    # Usa o MESMO YAML do base_localization (passado via argumento base_loc_params_file)
    calib_node = Node(
        package='base_localization',
        executable='calib_data_logger_posearray',
        name='calib_data_logger_posearray',
        output='screen',
        parameters=[{
            'detection_topic': '/base_detection/detected_coords',
            'gt_pose_array_topic': '/land_bases/pose_array',
            'reference_frame': 'map',
            'target_frame': 'base_link_1',
            'tf_timeout_sec': 0.3,
            'base_localization_yaml_path': LaunchConfiguration('base_loc_params_file'),
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
        # Args
        base_loc_params_arg,
        use_rviz_arg,
        use_gz_gui_arg,
        base_fixed_arg,
        base_x_arg,
        base_y_arg,
        base_z_arg,
        base_yaw_arg,
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),
        disable_db := SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),
        gazebo_server,
        gazebo_client,
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
