#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

import tf2_ros
import tf2_geometry_msgs  # registra conversões para Buffer.transform


def rotmat_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Converte matriz 3x3 em quaternion (x, y, z, w) sem libs extras."""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22

    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def rvec_tvec_to_pose(rvec: np.ndarray, tvec: np.ndarray) -> Pose:
    """Converte (rvec,tvec) do OpenCV em geometry_msgs/Pose (no frame da câmera)."""
    R, _ = cv2.Rodrigues(rvec)
    qx, qy, qz, qw = rotmat_to_quaternion(R)
    px, py, pz = tvec.flatten().tolist()

    pose = Pose()
    pose.position.x = float(px)
    pose.position.y = float(py)
    pose.position.z = float(pz)
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


class BaseLocalization(Node):
    """
    - Lê K/D da câmera via YAML.
    - Assina bboxes 2D (Float32MultiArray [x1,y1,x2,y2,score,...]).
    - Estima pose da base (quadrado 1x1m) com PnP.
    - Mantém e publica TF contínua por base como filho de 'drone_frame':
        map -> base_link_1 (seu TF) -> base_target_i (publicado aqui)
    """

    def __init__(self):
        super().__init__('base_localization')

        # ---------- Parâmetros ----------
        self.declare_parameter('detected_coords_topic', '/base_detection/detected_coords')
        self.declare_parameter('camera_frame', 'espcam_link_1')
        self.declare_parameter('drone_frame', 'base_link_1')
        self.declare_parameter('square_size_m', 1.0)
        self.declare_parameter('min_score', 0.25)
        self.declare_parameter('tf_child_prefix', 'base_target_')
        self.declare_parameter('tf_publish_rate_hz', 20.0)

        # intrínsecos
        self.declare_parameter('camera.K', [600.0, 0.0, 320.0,
                                            0.0, 600.0, 240.0,
                                            0.0,   0.0,   1.0])
        self.declare_parameter('camera.D', [0.0, 0.0, 0.0, 0.0, 0.0])

        # leitura
        self.detected_coords_topic = self.get_parameter('detected_coords_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.drone_frame = self.get_parameter('drone_frame').get_parameter_value().string_value
        self.square_size = float(self.get_parameter('square_size_m').get_parameter_value().double_value)
        self.min_score = float(self.get_parameter('min_score').get_parameter_value().double_value)
        self.tf_child_prefix = self.get_parameter('tf_child_prefix').get_parameter_value().string_value
        self.tf_publish_rate_hz = float(self.get_parameter('tf_publish_rate_hz').get_parameter_value().double_value)

        K_list = list(self.get_parameter('camera.K').get_parameter_value().double_array_value)
        if len(K_list) != 9:
            raise RuntimeError("camera.K deve ter 9 valores (fx,0,cx, 0,fy,cy, 0,0,1).")
        self.K = np.array(K_list, dtype=np.float64).reshape(3, 3)

        D_list = list(self.get_parameter('camera.D').get_parameter_value().double_array_value)
        self.D = np.array(D_list, dtype=np.float64).reshape(-1, 1) if len(D_list) > 0 else None

        # ---------- QoS ----------
        qos_default = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------- Subs ----------
        self.create_subscription(Float32MultiArray, self.detected_coords_topic, self.on_detected_coords, qos_default)

        # ---------- Estado: poses atuais das bases no frame do drone ----------
        # key: child_name (str) -> PoseStamped no frame do drone
        self._targets: Dict[str, PoseStamped] = {}

        # ---------- Timer para publicar TF continuamente ----------
        period = 1.0 / max(1.0, self.tf_publish_rate_hz)
        self._tf_timer = self.create_timer(period, self._publish_current_tfs)

        self.get_logger().info(
            f"[base_localization] frames: camera={self.camera_frame} -> drone={self.drone_frame} | "
            f"K=\n{self.K}\nD={D_list} | topic={self.detected_coords_topic} | tf_rate={self.tf_publish_rate_hz} Hz"
        )

        self._logged_tf_error_once = False

    # ---------- Callback principal ----------
    def on_detected_coords(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) % 5 != 0:
            self.get_logger().warn("Formato inválido em detected_coords (len % 5 != 0).")
            return

        # Verifica TF disponível
        if not self.tf_buffer.can_transform(self.drone_frame, self.camera_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)):
            if not self._logged_tf_error_once:
                self.get_logger().error(
                    f'TF {self.camera_frame}->{self.drone_frame} indisponível. '
                    f'Confira os frames no YAML e o /tf_static.'
                )
                self._logged_tf_error_once = True
            return

        # 3D do quadrado (1x1m) no frame do alvo
        s = float(self.square_size)
        obj_pts = np.array([
            [-s/2.0, -s/2.0, 0.0],  # TL
            [ s/2.0, -s/2.0, 0.0],  # TR
            [ s/2.0,  s/2.0, 0.0],  # BR
            [-s/2.0,  s/2.0, 0.0],  # BL
        ], dtype=np.float64)

        # vamos preencher um novo conjunto de keys válidas neste frame
        new_keys: List[str] = []
        stamp_now = self.get_clock().now().to_msg()
        idx = 0

        for i in range(0, len(data), 5):
            x1, y1, x2, y2, score = data[i:i+5]
            if float(score) < self.min_score:
                continue

            img_pts = np.array([
                [x1, y1],  # TL
                [x2, y1],  # TR
                [x2, y2],  # BR
                [x1, y2],  # BL
            ], dtype=np.float64)

            ok, rvec, tvec = self.solve_pnp(obj_pts, img_pts, self.K, self.D)
            if not ok or tvec[2] <= 0:
                continue

            # Pose no frame da CÂMERA
            pose_cam = rvec_tvec_to_pose(rvec, tvec)

            # Transforma para DRONE via Buffer.transform
            ps_cam = PoseStamped()
            ps_cam.header.frame_id = self.camera_frame
            ps_cam.header.stamp = stamp_now
            ps_cam.pose = pose_cam

            try:
                ps_drone: PoseStamped = self.tf_buffer.transform(
                    ps_cam,
                    self.drone_frame,
                    timeout=rclpy.duration.Duration(seconds=0.2)
                )
            except Exception as e:
                if not self._logged_tf_error_once:
                    self.get_logger().error(f"Transform {self.camera_frame}->{self.drone_frame} falhou: {e}")
                    self._logged_tf_error_once = True
                return

            # Nome do filho TF
            child_name = f"{self.tf_child_prefix}{idx}"
            new_keys.append(child_name)

            # Guarda/atualiza pose deste alvo
            self._targets[child_name] = ps_drone

            idx += 1

        # remove alvos antigos que não apareceram neste frame
        for k in list(self._targets.keys()):
            if k not in new_keys:
                del self._targets[k]

    # ---------- Timer: publica TFs correntes continuamente ----------
    def _publish_current_tfs(self):
        if not self._targets:
            return
        stamp_now = self.get_clock().now().to_msg()

        for child_name, ps_drone in self._targets.items():
            t = TransformStamped()
            t.header.stamp = stamp_now
            t.header.frame_id = self.drone_frame
            t.child_frame_id = child_name
            t.transform.translation.x = ps_drone.pose.position.x
            t.transform.translation.y = ps_drone.pose.position.y
            t.transform.translation.z = ps_drone.pose.position.z
            t.transform.rotation = ps_drone.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    # ---------- Utilidades ----------
    def solve_pnp(self, obj_pts: np.ndarray, img_pts: np.ndarray,
                  K: np.ndarray, D: Optional[np.ndarray]) -> Tuple[bool, np.ndarray, np.ndarray]:
        """Tenta IPPE_SQUARE → SQPNP → ITERATIVE."""
        for flag in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_SQPNP, cv2.SOLVEPNP_ITERATIVE):
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    objectPoints=obj_pts,
                    imagePoints=img_pts,
                    cameraMatrix=K,
                    distCoeffs=D if (D is not None and D.size > 0) else None,
                    flags=flag
                )
                if ok:
                    return True, rvec, tvec
            except Exception:
                pass
        return False, None, None


def main():
    rclpy.init()
    node = BaseLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
