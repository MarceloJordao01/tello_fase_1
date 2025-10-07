#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion, PointStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import TransformException


# ---------------- QoS helpers ----------------
def make_qos_sensor(reliable: bool) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def make_qos_marker() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
    )


def make_qos_status() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class BaseLocalizationNode(Node):
    """
    Fluxo:
      - Assina /base_detection/detected_coords (Float32MultiArray) e guarda o último lote.
      - No callback de detecção:
          * Lê TF (reference_frame <- target_frame). Se disponível, para cada bbox:
              cx=(x1+x2)/2, cy=(y1+y2)/2
              dx = -cx + 48
              dy = -cy + 480
              P0 = TF(target_frame) em reference_frame
              P1 = P0 + K*(dx,dy,0)   (z de P1 = 0)
          * Publica um PointStamped (frame = reference_frame) por bbox em /base_localization/points.
          * Publica /base_detection/num_bases (Int32).
      - Serviço /base_localization/update_markers (Trigger):
          * Apenas desenha/atualiza os Markers LINE_STRIP [P0,P1] por bbox (id = índice),
            e (opcional) publica PoseArray com todos P1.
      - Serviço /base_localization/clear_markers (Trigger) para remover markers.

    Importante: lifetime padrão do Marker é 0.0 (infinito) para manter visível no RViz após o serviço.
    """

    def __init__(self) -> None:
        super().__init__("base_localization")

        # ---------------- Parâmetros ----------------
        self.declare_parameter("detection_topic", "/base_detection/detected_coords")
        self.declare_parameter("num_bases_topic", "/base_detection/num_bases")

        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("target_frame", "base_link_1")
        self.declare_parameter("tf_timeout_sec", 0.2)

        # QoS/assinatura das detecções
        self.declare_parameter("detection_qos_reliable", True)

        # Ganho K (metros por pixel)
        self.declare_parameter("k_gain", 0.01)

        # Visualização (Marker)
        self.declare_parameter("marker_topic", "/base_localization/line")
        self.declare_parameter("marker_ns", "base_link_lines")
        self.declare_parameter("marker_line_width", 0.03)
        self.declare_parameter("marker_color_r", 0.1)
        self.declare_parameter("marker_color_g", 0.9)
        self.declare_parameter("marker_color_b", 0.1)
        self.declare_parameter("marker_color_a", 1.0)
        self.declare_parameter("marker_lifetime_sec", 0.0)  # 0.0 = infinito

        # Publicações auxiliares
        self.declare_parameter("publish_pose_array", True)  # pode desligar se quiser só PointStamped
        self.declare_parameter("points_topic", "/base_localization/points")
        self.declare_parameter("poses_topic", "/base_localization/poses")

        # ---------------- Lê parâmetros ----------------
        gp = lambda n: self.get_parameter(n).get_parameter_value()

        self.detection_topic: str = gp("detection_topic").string_value
        self.num_bases_topic: str = gp("num_bases_topic").string_value

        self.reference_frame: str = gp("reference_frame").string_value
        self.target_frame: str = gp("target_frame").string_value
        self.tf_timeout_sec: float = gp("tf_timeout_sec").double_value

        self.det_qos_reliable: bool = gp("detection_qos_reliable").bool_value

        self.k_gain: float = float(gp("k_gain").double_value)

        self.marker_topic: str = gp("marker_topic").string_value
        self.marker_ns: str = gp("marker_ns").string_value
        self.marker_line_width: float = float(gp("marker_line_width").double_value)
        self.marker_color = (
            float(gp("marker_color_r").double_value),
            float(gp("marker_color_g").double_value),
            float(gp("marker_color_b").double_value),
            float(gp("marker_color_a").double_value),
        )
        self.marker_lifetime_sec: float = float(gp("marker_lifetime_sec").double_value)

        self.publish_pose_array: bool = gp("publish_pose_array").bool_value
        self.points_topic: str = gp("points_topic").string_value
        self.poses_topic: str = gp("poses_topic").string_value

        # ---------------- Convenção da imagem ----------------
        self.cx_ref = 960.0/2.0
        self.cy_ref = 720.0/2.0

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # ---------------- Sub/Pub ----------------
        self.create_subscription(
            Float32MultiArray,
            self.detection_topic,
            self._detections_cb,
            make_qos_sensor(self.det_qos_reliable),
        )
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, make_qos_marker())
        self.num_bases_pub = self.create_publisher(Int32, self.num_bases_topic, make_qos_status())

        self.points_pub = self.create_publisher(PointStamped, self.points_topic, make_qos_status())
        self.poses_pub = self.create_publisher(PoseArray, self.poses_topic, make_qos_status())

        # Serviços
        self.srv_update = self.create_service(Trigger, "/base_localization/update_markers", self._srv_update_markers)
        self.srv_clear  = self.create_service(Trigger, "/base_localization/clear_markers", self._srv_clear_markers)

        # Últimas detecções: (x1, y1, x2, y2, score, cx, cy, w, h, dx, dy)
        self.last_bboxes: List[Tuple[float, float, float, float, float, float, float, float, float, float, float]] = []

        # Controle de IDs de markers
        self._last_marker_count = 0

        self.get_logger().info(
            "base_localization (modo serviço) iniciado\n"
            f"  reference_frame : {self.reference_frame}\n"
            f"  target_frame    : {self.target_frame}\n"
            f"  detection_topic : {self.detection_topic} (QoS {'RELIABLE' if self.det_qos_reliable else 'BEST_EFFORT'})\n"
            f"  num_bases_topic : {self.num_bases_topic}\n"
            f"  k_gain          : {self.k_gain:.4f} (m/pixel)\n"
            f"  marker_topic    : {self.marker_topic}\n"
            f"  marker_lifetime : {self.marker_lifetime_sec:.2f} s (0.0 = infinito)\n"
            f"  points_topic    : {self.points_topic}\n"
            f"  poses_topic     : {self.poses_topic} (publish_pose_array={self.publish_pose_array})\n"
            f"  serviços        : /base_localization/update_markers, /base_localization/clear_markers"
        )

    # ---------------- Callback de detecções ----------------
    def _detections_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data) if msg.data is not None else []
        boxes = []

        if len(data) >= 5:
            n = len(data) // 5
            for i in range(n):
                x1, y1, x2, y2, score = map(float, data[i * 5 : (i + 1) * 5])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w = abs(x2 - x1)
                h = abs(y2 - y1)
                dx = cx - self.cx_ref
                dy = cy - self.cy_ref
                boxes.append((x1, y1, x2, y2, score, cx, cy, w, h, dx, dy))

        self.last_bboxes = boxes
        self.num_bases_pub.publish(Int32(data=len(self.last_bboxes)))

        # Publica um PointStamped POR BASE, já no reference_frame (se TF disponível)
        if boxes:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    self.target_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
                t = tf.transform.translation
                now = self.get_clock().now().to_msg()

                for i, (_x1, _y1, _x2, _y2, _s, _cx, _cy, _w, _h, dx, dy) in enumerate(boxes):
                    vx = self.k_gain * dx
                    vy = self.k_gain * dy

                    p1 = Point(x=float(t.x + vx), y=float(t.y + vy), z=0.0)

                    ps = PointStamped()
                    ps.header.frame_id = self.reference_frame
                    ps.header.stamp = now
                    ps.point = p1
                    self.points_pub.publish(ps)

                # Log resumido
                self.get_logger().info(f"PointStamped publicados: {len(boxes)} base(s).")

            except TransformException as ex:
                self.get_logger().warn(f"TF indisponível {self.reference_frame} <- {self.target_frame}: {ex}")
            except Exception as ex:
                self.get_logger().warn(f"Erro ao publicar PointStamped: {ex}")
        else:
            self.get_logger().info("Detecções: 0")

    # ---------------- Serviços ----------------
    def _srv_update_markers(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        # 1) TF do target_frame no reference_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as ex:
            self._clear_all_markers()
            response.success = False
            response.message = f"TF indisponível {self.reference_frame} <- {self.target_frame}: {ex}"
            return response
        except Exception as ex:
            self._clear_all_markers()
            response.success = False
            response.message = f"Erro TF: {ex}"
            return response

        t = tf.transform.translation

        # 2) Se não houver bboxes, limpe markers e retorne
        if not self.last_bboxes:
            self._clear_all_markers()
            response.success = True
            response.message = "Sem detecções; markers limpos."
            return response

        # 3) Para cada bbox: calcula P1, publica Marker e (opcional) PoseArray
        poses = PoseArray()
        poses.header.frame_id = self.reference_frame
        poses.header.stamp = self.get_clock().now().to_msg()

        for i, (_x1, _y1, _x2, _y2, _s, _cx, _cy, _w, _h, dx, dy) in enumerate(self.last_bboxes):
            vx = self.k_gain * dx
            vy = self.k_gain * dy

            p0 = Point(x=float(t.x), y=float(t.y), z=float(t.z))
            p1 = Point(x=float(t.x + vx), y=float(t.y + vy), z=0.0)

            # Marker (linha P0->P1)
            self._publish_line(points=[p0, p1], marker_id=i)

            # (Opcional) PoseArray
            if self.publish_pose_array:
                poses.poses.append(Pose(position=p1, orientation=Quaternion(w=1.0)))

            # Log por bbox
            self.get_logger().info(
                f"[{i}] Δ(px)=({dx:.1f},{dy:.1f}) => P1=({p1.x:.3f},{p1.y:.3f},{p1.z:.3f})"
            )

        # Publica PoseArray (se habilitado)
        if self.publish_pose_array and len(poses.poses) > 0:
            self.poses_pub.publish(poses)

        # Apaga markers sobrando (se antes havia mais que agora)
        if self._last_marker_count > len(self.last_bboxes):
            for marker_id in range(len(self.last_bboxes), self._last_marker_count):
                self._delete_marker(marker_id)
        self._last_marker_count = len(self.last_bboxes)

        response.success = True
        response.message = f"Markers atualizados: {len(self.last_bboxes)} base(s)."
        return response

    def _srv_clear_markers(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._clear_all_markers()
        response.success = True
        response.message = "Markers limpos."
        return response

    # ---------------- Marker helpers ----------------
    def _new_marker(self, marker_id: int) -> Marker:
        m = Marker()
        m.header.frame_id = self.reference_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = self.marker_ns
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.scale.x = self.marker_line_width
        m.color.r, m.color.g, m.color.b, m.color.a = self.marker_color
        # lifetime: 0.0 => infinito (fica visível após o serviço)
        if self.marker_lifetime_sec > 0.0:
            m.lifetime = Duration(seconds=self.marker_lifetime_sec).to_msg()
        m.pose.orientation.w = 1.0
        return m

    def _publish_line(self, points: List[Point], marker_id: int) -> None:
        m = self._new_marker(marker_id)
        m.action = Marker.ADD
        m.points = points  # 2 pontos: P0 e P1
        self.marker_pub.publish(m)

    def _delete_marker(self, marker_id: int) -> None:
        m = self._new_marker(marker_id)
        m.action = Marker.DELETE
        self.marker_pub.publish(m)

    def _clear_all_markers(self) -> None:
        if self._last_marker_count <= 0:
            return
        for marker_id in range(self._last_marker_count):
            self._delete_marker(marker_id)
        self._last_marker_count = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
