#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from typing import List, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from gazebo_msgs.msg import ModelStates


def _coerce_to_str_list(val: Any) -> List[str]:
    """
    Converte o parâmetro recebido (pode vir como BYTE_ARRAY, int[], etc.) para List[str].
    """
    if val is None:
        return []
    if isinstance(val, list) and all(isinstance(x, str) for x in val):
        return val
    if isinstance(val, (bytes, bytearray)):
        try:
            return [val.decode('utf-8')]
        except Exception:
            return [str(list(val))]
    if isinstance(val, list):
        if all(isinstance(x, (bytes, bytearray)) for x in val):
            try:
                return [''.join([xb.decode('utf-8') for xb in val])]
            except Exception:
                return [str([list(xb) for xb in val])]
        if all(isinstance(x, int) for x in val):
            try:
                return [bytes(val).decode('utf-8')]
            except Exception:
                return [str(val)]
        return [str(x) for x in val]
    return [str(val)]


class LandBasesPosePublisher(Node):
    def __init__(self):
        super().__init__('land_bases_pose_publisher')

        # ===== Parâmetros (dynamic_typing para evitar bug do [] virar BYTE_ARRAY) =====
        dyn = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter('base_names', [], dyn)
        self.declare_parameter('output_topic', '/land_bases/pose_array', dyn)
        self.declare_parameter('names_topic', '/land_bases/names', dyn)
        self.declare_parameter('frame_id', 'map', dyn)
        # >>> CORRIGIDO: default certo do Gazebo Classic/Humble é "/model_states"
        self.declare_parameter('model_states_topic', '/model_states', dyn)

        # Lê e normaliza tipos
        self.base_names: List[str] = _coerce_to_str_list(self.get_parameter('base_names').value)
        self.output_topic: str = str(self.get_parameter('output_topic').value)
        self.names_topic: str = str(self.get_parameter('names_topic').value)
        self.frame_id: str = str(self.get_parameter('frame_id').value)
        self.model_states_topic: str = str(self.get_parameter('model_states_topic').value)

        if not self.base_names:
            self.get_logger().warn(
                "Parametro 'base_names' vazio ou de tipo inesperado. "
                "Tentarei descobrir os nomes por prefixo 'land_base_'."
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseArray, self.output_topic, qos)
        self.names_pub = self.create_publisher(String, self.names_topic, qos)

        # Subscriber em /model_states (corrigido)
        self.sub = self.create_subscription(ModelStates, self.model_states_topic, self._cb_states, qos)

        self._printed_first_ok = False

        self.get_logger().info(
            f"Publicando poses em '{self.output_topic}' e nomes em '{self.names_topic}'. "
            f"Frame: '{self.frame_id}'. Monitorando '{self.model_states_topic}'. "
            f"Bases esperadas: {self.base_names if self.base_names else '[auto por prefixo]'}"
        )

    def _cb_states(self, msg: ModelStates):
        name_to_pose: Dict[str, Pose] = {name: pose for name, pose in zip(msg.name, msg.pose)}

        # Se não veio base_names, descobre por prefixo
        if not self.base_names:
            self.base_names = sorted([n for n in name_to_pose.keys() if n.startswith('land_base_')])

        # Monta PoseArray na ordem de base_names
        pa = PoseArray()
        pa.header.frame_id = self.frame_id
        pa.header.stamp = self.get_clock().now().to_msg()

        valid_names: List[str] = []
        for n in self.base_names:
            if n in name_to_pose:
                pa.poses.append(name_to_pose[n])
                valid_names.append(n)

        if pa.poses:
            self.pose_pub.publish(pa)
            names_msg = String()
            names_msg.data = json.dumps(valid_names)
            self.names_pub.publish(names_msg)
            if not self._printed_first_ok:
                self.get_logger().info(
                    f"Publicado PoseArray com {len(pa.poses)} bases: {valid_names}"
                )
                self._printed_first_ok = True


def main():
    rclpy.init()
    node = LandBasesPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
