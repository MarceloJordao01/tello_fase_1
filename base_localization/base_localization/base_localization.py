#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import List, Tuple
from dataclasses import dataclass
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger

import tf2_ros


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


# ---------------- Modelo: Polinômio por eixo com coeficientes dependentes do tamanho ----------------
@dataclass
class PolyBySize:
    """
    Δx' = Σ_k A_k(ṼD)·(ΔX_raw)^k
    Δy' = Σ_k C_k(ṼD)·(ΔY_raw)^k

    onde:
      - A_k(ṼD) = a_{k0} + a_{k1}·ṼD + a_{k2}·ṼD^2
      - C_k(ṼD) = c_{k0} + c_{k1}·ṼD + c_{k2}·ṼD^2
      - ṼD = (D - D0)/D0 e D = hypot(w, h) (diagonal do bbox em px).
    """
    D0: float = 300.0  # escala ref da diagonal (px)
    ax: List[Tuple[float, float, float]] = (
        (0.0, 0.0, 0.0),  # k=0 (bias)
        (1.0, 0.0, 0.0),  # k=1 (ganho)
        (0.0, 0.0, 0.0),  # k=2
    )
    ay: List[Tuple[float, float, float]] = (
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    )

    @staticmethod
    def _polyD(triplet: Tuple[float, float, float], Dt: float) -> float:
        a0, a1, a2 = triplet
        return a0 + a1 * Dt + a2 * (Dt ** 2)

    def _apply_axis(self, u: float, coeffs: List[Tuple[float, float, float]], Dt: float) -> float:
        # Calcula Σ_k C_k(ṼD)·u^k
        out = 0.0
        p = 1.0  # u^k
        for triplet in coeffs:  # k = 0..K
            ck = self._polyD(triplet, Dt)
            out += ck * p
            p *= u
        return out

    def apply(self, dx_raw: float, dy_raw: float, w: float, h: float) -> Tuple[float, float]:
        D = math.hypot(w, h)
        Dt = 0.0 if self.D0 <= 0.0 else (D - self.D0) / self.D0
        # Cada eixo usa apenas o seu Δ raw (sem trocas)
        dx_corr = self._apply_axis(dx_raw, self.ax, Dt)
        dy_corr = self._apply_axis(dy_raw, self.ay, Dt)
        return dx_corr, dy_corr


# ---------------- Modelo de Z: usa o MESMO D0 do poly_size ----------------
@dataclass
class ZFromSize:
    """
    z' = Σ_j Z_j(ṼD)·(ṼD)^j, com Z_j(ṼD) = z_{j0} + z_{j1}·ṼD + z_{j2}·ṼD^2.
    - Apenas da diagonal (D) -> não depende de ΔX_raw/ΔY_raw.
    - IMPORTANTE: D0 é herdado do poly_size.D0 (não existe z_from_size.D0 no YAML).
    """
    D0: float  # herdado do PolyBySize.D0
    zc: List[Tuple[float, float, float]] = (
        (0.0, 0.0, 0.0),  # j=0
        (0.0, 0.0, 0.0),  # j=1
        (0.0, 0.0, 0.0),  # j=2
    )
    z_default: float = 0.0
    z_min: float = -10.0
    z_max: float = 10.0

    @staticmethod
    def _polyD(c: Tuple[float, float, float], Dt: float) -> float:
        c0, c1, c2 = c
        return c0 + c1 * Dt + c2 * (Dt ** 2)

    def apply(self, w: float, h: float) -> float:
        try:
            D = math.hypot(w, h)
            if not math.isfinite(D) or D <= 0.0:
                return self.z_default
            Dt = 0.0 if self.D0 <= 0.0 else (D - self.D0) / self.D0
            z = 0.0
            p = 1.0  # (ṼD)^j
            for triplet in self.zc:  # j = 0..J
                cj = self._polyD(triplet, Dt)
                z += cj * p
                p *= Dt
            if not math.isfinite(z):
                return self.z_default
            return min(max(z, self.z_min), self.z_max)
        except Exception:
            return self.z_default


class BaseLocalizationNode(Node):
    """
    - Assina /base_detection/detected_coords e guarda o último lote (self.last_bboxes).
    - Assina /base_detection/num_bases e guarda o último valor (self.last_num_bases).
    - Serviço ~/update_markers:
        * Se last_num_bases == 0 => não adiciona poses novas (limpa markers).
        * Se > 0 => faz TF (map <- target), computa P1 por bbox:
            - ΔX_raw = cx_ref - cx, ΔY_raw = cy_ref - cy   (MESMO padrão do calib)
            - correção polinomial por eixo (cada eixo usa seu Δ raw)
            - z' só de D (usando D0 = poly_size.D0)
            - P1 = origin_map + [Δx', Δy', z']   (sem k_gain)
          Desenha markers P0->P1 e APPEND em PoseArray acumulado.
      No retorno do serviço, imprime o vetor `base_link -> inferência` para cada bbox.
    - Serviço ~/clear_markers:
        * Limpa apenas os markers (preserva PoseArray acumulado).
    """

    def __init__(self) -> None:
        super().__init__("base_localization")

        # ---------------- Parâmetros ----------------
        self.declare_parameter("detection_topic", "/base_detection/detected_coords")
        self.declare_parameter("num_bases_topic", "/base_detection/num_bases")

        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("target_frame", "base_link_1")
        self.declare_parameter("tf_timeout_sec", 0.2)

        self.declare_parameter("detection_qos_reliable", True)

        # Centro da imagem (px) — agora interpretado DIRETAMENTE (sem swap)
        self.declare_parameter("cx_ref", 960.0 / 2.0)
        self.declare_parameter("cy_ref", 720.0 / 2.0)

        # Visualização (Marker)
        self.declare_parameter("marker_topic", "/base_localization/line")
        self.declare_parameter("marker_ns", "base_link_lines")
        self.declare_parameter("marker_line_width", 0.03)
        self.declare_parameter("marker_color_r", 0.1)
        self.declare_parameter("marker_color_g", 0.9)
        self.declare_parameter("marker_color_b", 0.1)
        self.declare_parameter("marker_color_a", 1.0)
        self.declare_parameter("marker_lifetime_sec", 0.0)  # 0.0 = infinito

        # Publicação do PoseArray acumulado
        self.declare_parameter("poses_topic", "/base_localization/poses")

        # --------------- poly_size (por YAML: D0, ax, ay) ---------------
        self.declare_parameter("poly_size.D0", 300.0)
        # ax/ay vêm FLATTENED (lista de floats, múltiplo de 3; triplet por grau)
        self.declare_parameter("poly_size.ax", [0.0, 0.0, 0.0,   1.0, 0.0, 0.0,   0.0, 0.0, 0.0])
        self.declare_parameter("poly_size.ay", [0.0, 0.0, 0.0,   1.0, 0.0, 0.0,   0.0, 0.0, 0.0])

        # --------------- z_from_size (sem D0 no YAML) ---------------
        # Mantemos coeficientes/limites, mas **não** declaramos z_from_size.D0.
        self.declare_parameter("z_from_size.zc", [0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0])
        self.declare_parameter("z_from_size.z_default", 0.0)
        self.declare_parameter("z_from_size.z_min", -10.0)
        self.declare_parameter("z_from_size.z_max", 10.0)

        # ---------------- Lê parâmetros ----------------
        gp = lambda n: self.get_parameter(n).get_parameter_value()

        self.detection_topic: str = gp("detection_topic").string_value
        self.num_bases_topic: str = gp("num_bases_topic").string_value

        self.reference_frame: str = gp("reference_frame").string_value
        self.target_frame: str = gp("target_frame").string_value
        self.tf_timeout_sec: float = gp("tf_timeout_sec").double_value

        self.det_qos_reliable: bool = gp("detection_qos_reliable").bool_value

        # Lidos do YAML…
        self.cx_ref: float = float(gp("cx_ref").double_value)
        self.cy_ref: float = float(gp("cy_ref").double_value)

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

        self.poses_topic: str = gp("poses_topic").string_value

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
        self.create_subscription(
            Int32,
            self.num_bases_topic,
            self._num_bases_cb,
            make_qos_status(),
        )
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, make_qos_marker())

        self.poses_pub = self.create_publisher(PoseArray, self.poses_topic, make_qos_status())
        self.poses_accum = PoseArray()
        self.poses_accum.header.frame_id = self.reference_frame  # acumulado em 'map'

        # Correção polinomial e Z a partir dos parâmetros (inclui YAML)
        self.poly_size = PolyBySize(
            D0=float(gp("poly_size.D0").double_value),
            ax=self._parse_triplets(gp("poly_size.ax").double_array_value),
            ay=self._parse_triplets(gp("poly_size.ay").double_array_value),
        )
        # OBS: ZFromSize agora **usa o mesmo D0** do poly_size (sem ler do YAML).
        self.z_from_size = ZFromSize(
            D0=self.poly_size.D0,
            zc=self._parse_triplets(gp("z_from_size.zc").double_array_value),
            z_default=float(gp("z_from_size.z_default").double_value),
            z_min=float(gp("z_from_size.z_min").double_value),
            z_max=float(gp("z_from_size.z_max").double_value),
        )

        # Estado
        self.last_bboxes: List[Tuple[float, float, float, float, float, float, float, float, float]] = []
        self.last_num_bases: int = 0
        self._last_marker_count = 0

        # ---------------- Serviços ----------------
        self.update_markers_srv = self.create_service(
            Trigger, '~/update_markers', self._srv_update_markers
        )
        self.clear_markers_srv = self.create_service(
            Trigger, '~/clear_markers', self._srv_clear_markers
        )

        # Logs iniciais
        self.get_logger().info(
            "base_localization iniciado (sem k_gain)\n"
            f"  detection_topic : {self.detection_topic}\n"
            f"  num_bases_topic : {self.num_bases_topic}\n"
            f"  reference_frame : {self.reference_frame}\n"
            f"  target_frame    : {self.target_frame}\n"
            f"  cx_ref(read)    : {self.cx_ref:.1f}\n"
            f"  cy_ref(read)    : {self.cy_ref:.1f}\n"
            f"  poses_topic     : {self.poses_topic}\n"
            f"  poly_size D0    : {self.poly_size.D0}\n"
            f"  z_from_size D0  : (usa poly_size.D0)"
        )
        try:
            resolved_update = self.update_markers_srv.srv_name  # type: ignore[attr-defined]
            resolved_clear = self.clear_markers_srv.srv_name    # type: ignore[attr-defined]
            self.get_logger().info(f"Serviço disponível: {resolved_update}")
            self.get_logger().info(f"Serviço disponível: {resolved_clear}")
        except Exception:
            self.get_logger().info("Serviços disponíveis: ~/update_markers e ~/clear_markers")

    # -------- helpers ----------
    @staticmethod
    def _parse_triplets(arr: List[float]) -> List[Tuple[float, float, float]]:
        """Recebe lista FLAT [a,b,c, a,b,c, ...] e devolve [(a,b,c), ...]."""
        out: List[Tuple[float, float, float]] = []
        if arr is None:
            return out
        vec = list(arr)
        if len(vec) % 3 != 0:
            n = (len(vec) // 3) * 3
            vec = vec[:n]
        for i in range(0, len(vec), 3):
            out.append((float(vec[i]), float(vec[i+1]), float(vec[i+2])))
        return out

    # ---------------- Callbacks ----------------
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
                boxes.append((x1, y1, x2, y2, score, cx, cy, w, h))
        self.last_bboxes = boxes
        self.get_logger().debug(f"Detecções recebidas: {len(boxes)}")

    def _num_bases_cb(self, msg: Int32) -> None:
        self.last_num_bases = int(msg.data)
        self.get_logger().debug(f"num_bases atualizado: {self.last_num_bases}")

    # ---------------- Cálculo ponto alvo ----------------
    def compute_p1_from_bbox(self, bbox: Tuple[float, float, float, float, float]) -> Point:
        """
        bbox = (x1, y1, x2, y2, score) em px.
        - TF lookup (map <- target_frame) -> origin_map
        - (cx,cy,w,h), deltas DIRETOS: ΔX_raw = cx_ref - cx, ΔY_raw = cy_ref - cy
        - correção polinomial por eixo -> (Δx',Δy')
        - z' só de D (usa D0 = poly_size.D0)
        - P1 = origin_map + [Δx', Δy', z']   (sem k_gain)
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as ex:
            raise RuntimeError(f"TF indisponível {self.reference_frame} <- {self.target_frame}: {ex}")

        t = tf.transform.translation
        origin_map = Point(x=float(t.x), y=float(t.y), z=float(t.z))

        x1, y1, x2, y2, _score = bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = abs(x2 - x1)
        h = abs(y2 - y1)

        # Deltas DIRETOS (MESMO padrão do calib, sem swap)
        dx_raw = self.cx_ref - cx
        dy_raw = self.cy_ref - cy

        dx_corr, dy_corr = self.poly_size.apply(dx_raw, dy_raw, w, h)
        z_corr = self.z_from_size.apply(w, h)

        # precisa da inversao pelos eixos (do mundo e da imagem)
        vx = dy_corr
        vy = dx_corr
        return Point(x=float(origin_map.x + vx), y=float(origin_map.y + vy), z=float(origin_map.z + z_corr))

    # ---------------- Serviços ----------------
    def _srv_update_markers(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.last_num_bases <= 0:
            self._clear_all_markers()
            response.success = True
            response.message = "Sem bases novas (num_bases == 0). Markers limpos; PoseArray preservado."
            return response

        if not self.last_bboxes:
            self._clear_all_markers()
            response.success = True
            response.message = "Sem detecções no último lote. Markers limpos; PoseArray preservado."
            return response

        now = self.get_clock().now().to_msg()

        appended = 0
        vectors_info: List[str] = []

        for i, (x1, y1, x2, y2, s, _cx, _cy, _w, _h) in enumerate(self.last_bboxes):
            try:
                p1 = self.compute_p1_from_bbox((x1, y1, x2, y2, s))
            except RuntimeError as ex:
                self.get_logger().warn(str(ex))
                continue

            # P0 para desenhar a linha (pose atual do base_link no mapa)
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    self.target_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
                t = tf.transform.translation
                p0 = Point(x=float(t.x), y=float(t.y), z=float(t.z))
            except Exception as ex:
                self.get_logger().warn(f"TF para marker: {ex}")
                p0 = Point(x=0.0, y=0.0, z=0.0)

            # Publica linha e atualiza PoseArray
            self._publish_line(points=[p0, p1], marker_id=i)
            self.poses_accum.poses.append(Pose(position=p1, orientation=Quaternion(w=1.0)))
            appended += 1

            # Vetor base_link -> inferência (no frame 'map')
            vx = float(p1.x - p0.x)
            vy = float(p1.y - p0.y)
            vz = float(p1.z - p0.z)
            vectors_info.append(
                f"#{i}: v=[{vx:.3f}, {vy:.3f}, {vz:.3f}] p1=[{p1.x:.3f}, {p1.y:.3f}, {p1.z:.3f}]"
            )

        self.poses_accum.header.frame_id = self.reference_frame
        self.poses_accum.header.stamp = now
        self.poses_pub.publish(self.poses_accum)

        if self._last_marker_count > len(self.last_bboxes):
            for marker_id in range(len(self.last_bboxes), self._last_marker_count):
                self._delete_marker(marker_id)
        self._last_marker_count = len(self.last_bboxes)

        # Monta mensagem do serviço incluindo os vetores de inferência
        vectors_block = "\n  ".join(vectors_info) if vectors_info else "(nenhum)"
        response.success = True
        response.message = (
            f"Markers atualizados. {appended} pose(s) novas adicionadas (num_bases={self.last_num_bases}). "
            f"PoseArray total: {len(self.poses_accum.poses)}\n"
            f"  vetores base_link->inferencia (no frame '{self.reference_frame}'):\n  {vectors_block}"
        )
        # Também loga para facilitar depuração
        self.get_logger().info(response.message)
        return response

    def _srv_clear_markers(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._clear_all_markers()
        response.success = True
        response.message = "Markers limpos (PoseArray acumulado preservado)."
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
        if self.marker_lifetime_sec > 0.0:
            m.lifetime = Duration(seconds=self.marker_lifetime_sec).to_msg()
        m.pose.orientation.w = 1.0
        return m

    def _publish_line(self, points: List[Point], marker_id: int) -> None:
        m = self._new_marker(marker_id)
        m.action = Marker.ADD
        m.points = points
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
