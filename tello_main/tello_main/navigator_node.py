#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ast
import math
import threading
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.parameter import Parameter

from geometry_msgs.msg import TransformStamped
from tello_msgs.srv import TelloAction
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import tf2_ros


def cm_to_m(v_cm: float) -> float:
    return float(v_cm) / 100.0


def parse_waypoints_list(items: List[str]) -> List[Tuple[int, int, int, int, float]]:
    """
    Espera uma lista de strings, cada uma representando um array:
      "[dx, dy, dz, speed, delay_s]"
    Retorna lista de tuplas (dx,dy,dz,speed,delay_s).
    """
    out: List[Tuple[int, int, int, int, float]] = []
    if not items:
        return out
    for s in items:
        if not isinstance(s, str):
            continue
        try:
            arr = ast.literal_eval(s.strip())
        except Exception:
            continue
        if not isinstance(arr, (list, tuple)) or len(arr) < 4:
            continue
        dx = int(arr[0]); dy = int(arr[1]); dz = int(arr[2]); spd = int(arr[3])
        delay_s = float(arr[4]) if len(arr) >= 5 else 0.0
        out.append((dx, dy, dz, spd, delay_s))
    return out


class TelloNavigator(Node):
    """
    Publica TF dinâmica map->base_link_1 e envia comandos TelloAction.
    Fluxo:
      - espera URDF/TF do base_link subir;
      - se 'require_start_service': aguarda Trigger '~/start_auto';
      - realiza takeoff e percorre waypoints.
      - publica continuamente '~/reach_waypoint' e lê '~/move_next_waypoint'.
    """

    def __init__(self):
        super().__init__('tello_navigator')

        self.cb_group = ReentrantCallbackGroup()

        # ---------------- Parâmetros ----------------
        self.declare_parameter('ns_abs', '/drone1')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_link_frame', 'base_link_1')
        self.declare_parameter('takeoff_height_m', 0.5)
        self.declare_parameter('auto_takeoff', True)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('require_start_service', True)
        self.declare_parameter('speed_cmps', 20)
        self.declare_parameter('debug', True)
        self.declare_parameter('probe_child_frame', 'camera_link_1')
        self.declare_parameter('wait_for_base_link_timeout_s', 15.0)
        self.declare_parameter('hold_after_takeoff_s', 0.0)
        self.declare_parameter('hold_between_waypoints_s', 0.0)

        self.declare_parameter('waypoints_list')

        self.ns_abs: str = self.get_parameter('ns_abs').get_parameter_value().string_value
        self.map_frame: str = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_link_frame: str = self.get_parameter('base_link_frame').get_parameter_value().string_value
        self.takeoff_height_m: float = self.get_parameter('takeoff_height_m').get_parameter_value().double_value
        self.auto_takeoff: bool = self.get_parameter('auto_takeoff').get_parameter_value().bool_value
        self.auto_start: bool = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.require_start_service: bool = self.get_parameter('require_start_service').get_parameter_value().bool_value
        self.default_speed_cmps: int = int(self.get_parameter('speed_cmps').value)
        self.debug: bool = self.get_parameter('debug').get_parameter_value().bool_value
        self.probe_child_frame: str = self.get_parameter('probe_child_frame').get_parameter_value().string_value
        self.wait_for_base_link_timeout_s: float = float(self.get_parameter('wait_for_base_link_timeout_s').value)
        self.hold_after_takeoff_s: float = float(self.get_parameter('hold_after_takeoff_s').value)
        self.hold_between_waypoints_s: float = float(self.get_parameter('hold_between_waypoints_s').value)

        # Leitura robusta do array de strings vindo do YAML
        wp_param = self.get_parameter('waypoints_list')
        wp_items: List[str] = []
        if wp_param.type_ == Parameter.Type.STRING_ARRAY:
            wp_items = list(wp_param.value)
        elif wp_param.type_ == Parameter.Type.STRING:
            wp_items = [str(wp_param.value)]
        else:
            wp_items = []

        parsed = parse_waypoints_list(wp_items)
        self.waypoints: List[Tuple[int, int, int, int, float]] = parsed

        # ---------------- Estado ----------------
        self._pos_m = [0.0, 0.0, 0.0]
        self._yaw_rad = 0.0
        self._lock = threading.Lock()
        self._started = False
        self._start_evt = threading.Event()

        # Estados dos novos tópicos
        self._reach_waypoint_state = False       # True quando parado num waypoint, False quando deslocando
        self._move_next_request = False          # Sinal recebido para ir ao próximo waypoint

        # ---------------- TF ----------------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._tf_timer = self.create_timer(1.0 / 30.0, self._publish_tf, callback_group=self.cb_group)

        # ---------------- Serviços e tópicos ----------------
        self._start_srv = self.create_service(Trigger, '~/start_auto', self._on_start_auto, callback_group=self.cb_group)
        self._tello_cli = self.create_client(TelloAction, f'{self.ns_abs}/tello_action')
        self._wait_for_service(self._tello_cli, 'tello_action')

        self._reach_wp_pub = self.create_publisher(Bool, '~/reach_waypoint', 10)
        self._move_next_sub = self.create_subscription(Bool, '~/move_next_waypoint', self._on_move_next_msg, 10)

        # Timer para publicar continuamente o estado do reach_waypoint
        self._reach_wp_timer = self.create_timer(0.1, self._reach_wp_publish_timer_cb)

        # Thread principal de missão
        threading.Thread(target=self._startup_sequence, daemon=True).start()

    # ---------------- Utils ----------------
    def _wait_for_service(self, client, name: str):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Aguardando serviço {name}...', throttle_duration_sec=2.0)

    def _logd(self, msg: str):
        if self.debug:
            self.get_logger().info(f'[DEBUG] {msg}')

    def _reach_wp_publish_timer_cb(self):
        """Publica continuamente o estado atual do reach_waypoint."""
        msg = Bool()
        with self._lock:
            msg.data = self._reach_waypoint_state
        self._reach_wp_pub.publish(msg)

    def _set_reach_state(self, state: bool):
        """Apenas altera o estado interno (publicação é contínua)."""
        with self._lock:
            if self._reach_waypoint_state != state:
                self._reach_waypoint_state = state
                self._logd(f'Alterando reach_waypoint para {state}')

    def _on_move_next_msg(self, msg: Bool):
        with self._lock:
            self._move_next_request = bool(msg.data)
        self._logd(f'Recebido ~/move_next_waypoint = {msg.data}')

    # ---------------- Callback do serviço ----------------
    def _on_start_auto(self, request, response):
        with self._lock:
            if self._started:
                response.success = True
                response.message = 'Missão já iniciada.'
                return response
            self._started = True
            self._start_evt.set()
        response.success = True
        response.message = 'Missão autorizada a iniciar.'
        self._logd('Serviço start_auto recebido: iniciando missão.')
        return response

    # ---------------- TF dynamic publisher ----------------
    def _publish_tf(self):
        with self._lock:
            x, y, z = self._pos_m
            yaw = self._yaw_rad

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy

        self.tf_broadcaster.sendTransform(t)

    def _set_position_incremental(self, target_pos_m, travel_time_s: float):
        steps_hz = 30.0
        steps = max(1, int(travel_time_s * steps_hz))
        with self._lock:
            start = self._pos_m[:]
        dx = target_pos_m[0] - start[0]
        dy = target_pos_m[1] - start[1]
        dz = target_pos_m[2] - start[2]

        if steps <= 1 or travel_time_s <= 0.0:
            with self._lock:
                self._pos_m = [target_pos_m[0], target_pos_m[1], target_pos_m[2]]
            return

        for i in range(1, steps + 1):
            a = i / steps
            with self._lock:
                self._pos_m[0] = start[0] + dx * a
                self._pos_m[1] = start[1] + dy * a
                self._pos_m[2] = start[2] + dz * a
            time.sleep(1.0 / steps_hz)

    # ---------------- Comandos ----------------
    def _send_tello_cmd(self, cmd: str) -> bool:
        req = TelloAction.Request()
        req.cmd = cmd
        self._logd(f'Enviando TelloAction: "{cmd}"')
        future = self._tello_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            self.get_logger().error('Timeout ao aguardar resposta do TelloAction.')
            return False
        resp = future.result()
        ok = getattr(resp, 'success', True)
        msg = getattr(resp, 'message', '')
        if ok:
            self._logd(f'Resposta OK: {msg}')
        else:
            self.get_logger().error(f'TelloAction falhou: {msg}')
        return bool(ok)

    def _takeoff(self):
        if self._send_tello_cmd('takeoff'):
            with self._lock:
                self._pos_m[2] = float(self.takeoff_height_m)
            self._logd(f'Takeoff: z={self.takeoff_height_m:.2f} m')

    def _go_relative_cm(self, dx_cm: int, dy_cm: int, dz_cm: int, speed_cmps: int):
        # Ao começar a mover: reach_waypoint = False
        self._set_reach_state(False)

        cmd = f'go {dx_cm} {dy_cm} {dz_cm} {speed_cmps}'
        ok = self._send_tello_cmd(cmd)
        if not ok:
            return

        with self._lock:
            start = self._pos_m[:]
        target = (
            start[0] + cm_to_m(dx_cm),
            start[1] + cm_to_m(dy_cm),
            start[2] + cm_to_m(dz_cm),
        )
        dist_cm = math.sqrt(dx_cm**2 + dy_cm**2 + dz_cm**2)
        travel_time_s = dist_cm / float(max(1, speed_cmps))
        self._logd(f'GO: d=({dx_cm},{dy_cm},{dz_cm}) cm, v={speed_cmps} cm/s, t≈{travel_time_s:.2f} s')

        self._set_position_incremental(target, travel_time_s)

        # Ao terminar o movimento: chegou ao waypoint
        self._set_reach_state(True)

    # ---------------- Fluxos ----------------
    def _wait_for_base_link(self) -> bool:
        deadline = time.time() + self.wait_for_base_link_timeout_s
        self._logd(f'Aguardando TF {self.base_link_frame} <-> {self.probe_child_frame}...')
        while rclpy.ok() and time.time() < deadline:
            try:
                if self.tf_buffer.can_transform(self.probe_child_frame, self.base_link_frame, Time()):
                    self._logd('Frames da URDF detectados.')
                    return True
            except Exception:
                pass
            time.sleep(0.1)
        self.get_logger().warn('URDF/TF não detectada no tempo limite; seguindo mesmo assim.')
        return False

    def _startup_sequence(self):
        self._wait_for_base_link()

        if self.require_start_service:
            self._logd('Aguardando serviço ~/start_auto para iniciar missão...')
            self._start_evt.wait()

        try:
            if self.auto_takeoff:
                self._takeoff()
                if self.hold_after_takeoff_s > 0.0:
                    self._logd(f'Aguardando {self.hold_after_takeoff_s:.2f}s após takeoff...')
                    time.sleep(self.hold_after_takeoff_s)

            if self.auto_start and self.waypoints:
                for idx, (dx, dy, dz, spd, delay_s) in enumerate(self.waypoints):
                    with self._lock:
                        self._move_next_request = False

                    self._go_relative_cm(dx, dy, dz, spd)

                    wait_s = delay_s if delay_s is not None else self.hold_between_waypoints_s
                    if wait_s > 0.0:
                        deadline = time.time() + wait_s
                        self._logd(f'Waypoint {idx}: aguardando {wait_s:.2f}s + sinal ~/move_next_waypoint=True...')
                        while rclpy.ok():
                            with self._lock:
                                allow = self._move_next_request
                            if time.time() >= deadline and allow:
                                break
                            time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f'Erro no fluxo automático: {e}')


def main():
    rclpy.init(args=None)
    node = TelloNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
