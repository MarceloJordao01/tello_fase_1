#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple, Any, Dict, Union

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


NumberLike = Union[int, float, str]


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

    Salva:
      stamp_sec, cx, cy, w, h, cx_ref, cy_ref, D0, transformX, transformY, transformZ

    Onde:
      - (cx,cy,w,h) vêm da detecção (px);
      - (cx_ref, cy_ref, D0) são lidos do base_localization.yaml;
      - transformX/Y/Z é o vetor da base_link até a base (GT) no frame base_link:
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

        # Caminho do YAML de onde LEREMOS cx_ref, cy_ref e D0
        default_bl_yaml = os.path.join(
            get_package_share_directory('base_localization'),
            'config',
            'base_localization.yaml'
        )
        self.declare_parameter('base_localization_yaml_path', default_bl_yaml)

        # CSV de saída
        default_csv = os.path.join(self._workspace_root_from_share(), 'calib_minimal.csv')
        self.declare_parameter('csv_path', default_csv)

        # Score mínimo
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

        # Lê (cx_ref, cy_ref, D0) do YAML
        self.cx_ref, self.cy_ref, self.D0 = self._load_refs_from_yaml(self.base_loc_yaml_path)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # Estado recente
        self._last_bboxes: List[DetBBox] = []
        self._last_gt_poses: List[Pose] = []

        # Subs
        self.create_subscription(Float32MultiArray, self.detection_topic, self._det_cb, 10)
        self.create_subscription(PoseArray, self.gt_topic, self._gt_cb, 10)

        # Serviço
        self.create_service(Trigger, '~/capture_once', self._srv_capture_once)

        self.get_logger().info(
            f'[calib_data_logger_posearray] pronto. '
            f'det="{self.detection_topic}", gt="{self.gt_topic}", '
            f'csv="{self.csv_path}" | '
            f'YAML="{self.base_loc_yaml_path}" -> '
            f'cx_ref={self.cx_ref:.3f}, cy_ref={self.cy_ref:.3f}, D0={self.D0:.3f}'
        )

    # ---------- Helpers ----------
    def _workspace_root_from_share(self) -> str:
        try:
            share_dir = get_package_share_directory('base_localization')
            ws_root = os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..'))
            return ws_root if os.path.isdir(ws_root) else os.getcwd()
        except Exception:
            return os.getcwd()

    @staticmethod
    def _to_float(val: NumberLike) -> Optional[float]:
        if isinstance(val, (int, float)):
            return float(val)
        if isinstance(val, str):
            try:
                return float(val.strip())
            except Exception:
                return None
        return None

    def _load_refs_from_yaml(self, path: str) -> Tuple[float, float, float]:
        """Lê cx_ref, cy_ref e D0 do YAML."""
        cx_ref, cy_ref, D0 = 480.0, 360.0, 300.0
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as ex:
            self.get_logger().warn(f'[YAML] Falha ao ler "{path}": {ex}. Usando defaults.')
            return cx_ref, cy_ref, D0

        def find_key(d: Dict[str, Any], key: str) -> Optional[Any]:
            if not isinstance(d, dict):
                return None
            if key in d:
                return d[key]
            for _, v in d.items():
                if isinstance(v, dict):
                    r = find_key(v, key)
                    if r is not None:
                        return r
            return None

        raw_cx = find_key(data, 'cx_ref')
        raw_cy = find_key(data, 'cy_ref')
        raw_d0 = find_key(data, 'D0')

        cx_conv = self._to_float(raw_cx)
        cy_conv = self._to_float(raw_cy)
        d0_conv = self._to_float(raw_d0)

        if cx_conv is not None:
            cx_ref = cx_conv
        if cy_conv is not None:
            cy_ref = cy_conv
        if d0_conv is not None:
            D0 = d0_conv

        return cx_ref, cy_ref, D0

    @staticmethod
    def _ensure_csv_with_header(path: str) -> None:
        if os.path.exists(path):
            return
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'stamp_sec',
                'cx', 'cy', 'w', 'h',
                'cx_ref', 'cy_ref', 'D0',
                'transformX', 'transformY', 'transformZ'
            ])

    def _lookup_tf(self) -> Optional[Tuple[Point, Tuple[float, float, float, float]]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame, self.target_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout_sec)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            return Point(x=t.x, y=t.y, z=t.z), (q.x, q.y, q.z, q.w)
        except Exception as ex:
            self.get_logger().error(f'TF {self.reference_frame}<-{self.target_frame} indisponível: {ex}')
            return None

    @staticmethod
    def _quat_to_rot_matrix(qx, qy, qz, qw) -> List[List[float]]:
        xx, yy, zz = qx*qx, qy*qy, qz*qz
        xy, xz, yz = qx*qy, qx*qz, qy*qz
        wx, wy, wz = qw*qx, qw*qy, qw*qz
        return [
            [1 - 2*(yy+zz), 2*(xy - wz),     2*(xz + wy)],
            [2*(xy + wz),   1 - 2*(xx+zz),   2*(yz - wx)],
            [2*(xz - wy),   2*(yz + wx),     1 - 2*(xx+yy)],
        ]

    @staticmethod
    def _matT_vec(R, v):
        vx = R[0][0]*v[0] + R[1][0]*v[1] + R[2][0]*v[2]
        vy = R[0][1]*v[0] + R[1][1]*v[1] + R[2][1]*v[2]
        vz = R[0][2]*v[0] + R[1][2]*v[1] + R[2][2]*v[2]
        return (vx, vy, vz)

    # ---------- Callbacks ----------
    def _gt_cb(self, msg: PoseArray) -> None:
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

    # ---------- Serviço ----------
    def _srv_capture_once(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self._last_bboxes:
            response.success = False
            response.message = 'Sem detecções armazenadas.'
            return response
        if not self._last_gt_poses:
            response.success = False
            response.message = 'Sem GT armazenado.'
            return response

        tf_res = self._lookup_tf()
        if tf_res is None:
            response.success = False
            response.message = 'Falha no TF lookup.'
            return response

        origin_map, quat_bl = tf_res
        R_bl_to_map = self._quat_to_rot_matrix(*quat_bl)

        det = max(self._last_bboxes, key=lambda d: d.score)
        gt_pose = self._last_gt_poses[0]
        gt = gt_pose.position

        v_map = (gt.x - origin_map.x, gt.y - origin_map.y, gt.z - origin_map.z)
        v_bl = self._matT_vec(R_bl_to_map, v_map)

        stamp = self.get_clock().now().to_msg()

        row = [
            float(stamp.sec),
            det.cx, det.cy, det.w, det.h,
            float(self.cx_ref), float(self.cy_ref), float(self.D0),
            float(v_bl[0]), float(v_bl[1]), float(v_bl[2]),
        ]

        try:
            self._ensure_csv_with_header(self.csv_path)
            with open(self.csv_path, 'a', newline='') as f:
                csv.writer(f).writerow(row)
        except Exception as ex:
            response.success = False
            response.message = f'Erro ao escrever CSV: {ex}'
            return response

        response.success = True
        response.message = (
            f'1 linha escrita em "{self.csv_path}". '
            f'cx_ref={self.cx_ref:.2f}, cy_ref={self.cy_ref:.2f}, D0={self.D0:.2f}'
        )
        return response


def main(args=None):
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
