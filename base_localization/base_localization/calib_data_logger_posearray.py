#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple, Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose, Point
from std_srvs.srv import Trigger
import tf2_ros
import yaml

from ament_index_python.packages import get_package_share_directory


@dataclass
class DetBBox:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2.0

    @property
    def w(self) -> float:
        return abs(self.x2 - self.x1)

    @property
    def h(self) -> float:
        return abs(self.y2 - self.y1)


class CalibDataLoggerPoseArray(Node):
    """
    Coleta amostras sob demanda (via Trigger) para ajuste offline.

    Salva APENAS:
      stamp_sec, cx, cy, w, h, cx_ref, cy_ref, D0, transformX, transformY, transformZ

    Onde:
      - (cx,cy,w,h) vêm da detecção (px);
      - (cx_ref, cy_ref) e D0 são lidos do base_localization.yaml;
      - transformX/Y/Z é o vetor da base_link até a base (GT) EXPRESSO NO FRAME base_link:
          v_map = p_gt_map - p_bl_map
          v_bl  = R_map_bl^T * v_map
        (R_map_bl é a rotação do base_link em relação ao map via TF map<-base_link)

    Suposição: haverá apenas UMA base no PoseArray e UMA detecção relevante.
    """

    def __init__(self) -> None:
        super().__init__('calib_data_logger_posearray')

        # -------- Parâmetros --------
        self.declare_parameter('detection_topic', '/base_detection/detected_coords')
        self.declare_parameter('gt_pose_array_topic', '/land_bases/pose_array')

        self.declare_parameter('reference_frame', 'map')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 0.2)

        # Caminho do YAML de onde LEREMOS cx_ref, cy_ref e D0 (poly_size.D0)
        default_bl_yaml = os.path.join(
            get_package_share_directory('base_localization'),
            'config',
            'base_localization.yaml'
        )
        self.declare_parameter('base_localization_yaml_path', default_bl_yaml)

        # CSV de saída
        self.declare_parameter('csv_path', 'calib_minimal.csv')

        # Para aceitar/filtrar detecções
        self.declare_parameter('min_score', 0.0)

        gp = lambda n: self.get_parameter(n).get_parameter_value()

        self.detection_topic: str = gp('detection_topic').string_value
        self.gt_topic: str = gp('gt_pose_array_topic').string_value

        self.reference_frame: str = gp('reference_frame').string_value
        self.target_frame: str = gp('target_frame').string_value
        self.tf_timeout_sec: float = float(gp('tf_timeout_sec').double_value)

        self.base_loc_yaml_path: str = gp('base_localization_yaml_path').string_value
        self.csv_path: str = gp('csv_path').string_value
        self.min_score: float = float(gp('min_score').double_value)

        # Lê (cx_ref, cy_ref, D0) do YAML do base_localization
        self.cx_ref, self.cy_ref, self.D0 = self._load_refs_from_yaml(self.base_loc_yaml_path)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # Estado recente (buffers)
        self._last_bboxes: List[DetBBox] = []
        self._last_gt_poses: List[Pose] = []

        # Subscriptions (apenas para manter último estado)
        self.create_subscription(Float32MultiArray, self.detection_topic, self._det_cb, 10)
        self.create_subscription(PoseArray, self.gt_topic, self._gt_cb, 10)

        # Serviço para captura sob demanda
        self.create_service(Trigger, '~/capture_once', self._srv_capture_once)

        # CSV
        self._ensure_csv(self.csv_path)

        self.get_logger().info(
            f'[calib_data_logger_posearray] pronto. '
            f'det="{self.detection_topic}", gt="{self.gt_topic}", csv="{self.csv_path}" | '
            f'cx_ref={self.cx_ref:.1f}, cy_ref={self.cy_ref:.1f}, D0={self.D0:.3f}, '
            f'TF {self.reference_frame}<-{self.target_frame}'
        )

    # ---------- YAML loader ----------
    def _load_refs_from_yaml(self, path: str) -> Tuple[float, float, float]:
        """
        Lê cx_ref, cy_ref e poly_size.D0 do base_localization.yaml.
        Suporta formatos:
          root:
            ros__parameters: {cx_ref, cy_ref, poly_size: {D0: ...}}
        ou diretamente:
          cx_ref: ...
          cy_ref: ...
          poly_size:
            D0: ...
        """
        cx_ref = 480.0
        cy_ref = 360.0
        D0 = 300.0

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as ex:
            self.get_logger().warn(f'Falha ao ler YAML {path}: {ex}. Usando defaults.')
            return cx_ref, cy_ref, D0

        def find_key(d: Dict[str, Any], key: str) -> Optional[Any]:
            if not isinstance(d, dict):
                return None
            if key in d:
                return d[key]
            # procurar recursivamente em subdicts comuns (ex.: node_name, ros__parameters)
            for k, v in d.items():
                if isinstance(v, dict):
                    r = find_key(v, key)
                    if r is not None:
                        return r
            return None

        cxv = find_key(data, 'cx_ref')
        cyv = find_key(data, 'cy_ref')
        poly = find_key(data, 'poly_size')
        d0v = None
        if isinstance(poly, dict):
            d0v = poly.get('D0', None)

        if isinstance(cxv, (int, float)):
            cx_ref = float(cxv)
        if isinstance(cyv, (int, float)):
            cy_ref = float(cyv)
        if isinstance(d0v, (int, float)):
            D0 = float(d0v)

        return cx_ref, cy_ref, D0

    # ---------- Utilidades ----------
    def _ensure_csv(self, path: str) -> None:
        first_time = not os.path.exists(path)
        if first_time:
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    'stamp_sec',
                    'cx', 'cy', 'w', 'h',
                    'cx_ref', 'cy_ref', 'D0',
                    'transformX', 'transformY', 'transformZ'
                ])
            self.get_logger().info(f'Criado CSV: {path}')

    def _lookup_tf(self) -> Optional[Tuple[Point, Tuple[float, float, float, float]]]:
        """
        Retorna (translação do base_link no map, rotação quaternion (x,y,z,w)).
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame, self.target_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout_sec)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            origin = Point(x=float(t.x), y=float(t.y), z=float(t.z))
            quat = (float(q.x), float(q.y), float(q.z), float(q.w))
            return origin, quat
        except Exception as ex:
            self.get_logger().error(f'TF {self.reference_frame}<-{self.target_frame} indisponível: {ex}')
            return None

    @staticmethod
    def _quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> List[List[float]]:
        """
        Constrói a matriz de rotação 3x3 (R) a partir de um quaternion.
        Convenção: q = (x,y,z,w) e R mapeia vetores no frame base_link -> map (R_bl_to_map).
        """
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        R = [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
            [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
        ]
        return R

    @staticmethod
    def _matT_vec(R: List[List[float]], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        v_out = R^T * v
        """
        vx = R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2]
        vy = R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2]
        vz = R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2]
        return (vx, vy, vz)

    # ---------- Callbacks ----------
    def _gt_cb(self, msg: PoseArray) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.reference_frame:
            self.get_logger().warn(
                f'GT frame "{msg.header.frame_id}" != reference_frame "{self.reference_frame}"'
            )
        self._last_gt_poses = list(msg.poses)

    def _det_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data) if msg.data is not None else []
        self._last_bboxes = []
        if len(data) < 5:
            return
        n = len(data) // 5
        for i in range(n):
            x1, y1, x2, y2, score = map(float, data[i * 5:(i + 1) * 5])
            if score < self.min_score:
                continue
            self._last_bboxes.append(DetBBox(x1, y1, x2, y2, score))

    # ---------- Serviço: captura e grava no CSV uma única vez ----------
    def _srv_capture_once(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        # Verificações básicas
        if not self._last_bboxes:
            response.success = False
            response.message = 'Sem detecções armazenadas (aguarde /base_detection/detected_coords).'
            return response

        if not self._last_gt_poses:
            response.success = False
            response.message = 'Sem GT armazenado (aguarde /land_bases/pose_array).'
            return response

        tf_res = self._lookup_tf()
        if tf_res is None:
            response.success = False
            response.message = f'Falha no TF {self.reference_frame}<-{self.target_frame}.'
            return response
        origin_map, quat_bl = tf_res
        R_bl_to_map = self._quat_to_rot_matrix(*quat_bl)

        # Escolhe UMA detecção: pega a de maior score
        det = max(self._last_bboxes, key=lambda d: d.score)

        # Escolhe UMA base do PoseArray: usa a primeira
        gt_pose = self._last_gt_poses[0]
        gt = gt_pose.position

        # Vetor em MAP: p_gt - p_base_link
        v_map = (float(gt.x) - origin_map.x, float(gt.y) - origin_map.y, float(gt.z) - origin_map.z)
        # Vetor em BASE_LINK: v_bl = R^T * v_map (R = bl->map)
        v_bl = self._matT_vec(R_bl_to_map, v_map)

        # Timestamp: agora
        stamp = self.get_clock().now().to_msg()

        # Monta linha mínima
        row = [
            float(stamp.sec),
            det.cx, det.cy, det.w, det.h,
            float(self.cx_ref), float(self.cy_ref), float(self.D0),
            float(v_bl[0]), float(v_bl[1]), float(v_bl[2]),
        ]

        try:
            with open(self.csv_path, 'a', newline='') as f:
                wcsv = csv.writer(f)
                wcsv.writerow(row)
        except Exception as ex:
            response.success = False
            response.message = f'Falha ao escrever CSV: {ex}'
            return response

        response.success = True
        response.message = f'1 linha escrita em "{self.csv_path}".'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibDataLoggerPoseArray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
