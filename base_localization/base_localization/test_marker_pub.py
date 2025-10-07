#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def make_qos_marker() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class TestMarkerPub(Node):
    """
    Publica continuamente um LINE_STRIP com dois pontos:
      P0 = (0,0,0)
      P1 = (1,1,1)
    no frame indicado (default: 'map').
    """

    def __init__(self) -> None:
        super().__init__('test_marker_pub')

        # Parâmetros
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('marker_topic', '/test_marker/line')
        self.declare_parameter('line_width', 0.03)
        self.declare_parameter('color_r', 0.1)
        self.declare_parameter('color_g', 0.9)
        self.declare_parameter('color_b', 0.1)
        self.declare_parameter('color_a', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('lifetime_sec', 0.0)  # 0 = infinito

        getp = lambda n: self.get_parameter(n).get_parameter_value()

        self.frame_id = getp('frame_id').string_value
        self.marker_topic = getp('marker_topic').string_value
        self.line_width = float(getp('line_width').double_value)
        self.color = (
            float(getp('color_r').double_value),
            float(getp('color_g').double_value),
            float(getp('color_b').double_value),
            float(getp('color_a').double_value),
        )
        self.publish_rate_hz = float(getp('publish_rate_hz').double_value)
        self.lifetime_sec = float(getp('lifetime_sec').double_value)

        # Publisher
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, make_qos_marker())

        # Timer
        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            "test_marker_pub iniciado\n"
            f"  frame_id     : {self.frame_id}\n"
            f"  marker_topic : {self.marker_topic}\n"
            f"  publish_rate : {self.publish_rate_hz:.2f} Hz\n"
            f"  line_width   : {self.line_width}\n"
            f"  color RGBA   : {self.color}\n"
            f"  lifetime     : {self.lifetime_sec} s"
        )

    def _tick(self) -> None:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'test_marker'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.line_width
        m.color.r, m.color.g, m.color.b, m.color.a = self.color
        if self.lifetime_sec > 0.0:
            m.lifetime = Duration(seconds=self.lifetime_sec).to_msg()

        # Dois pontos: (0,0,0) → (1,1,1)
        p0 = Point(x=0.0, y=0.0, z=0.0)
        p1 = Point(x=1.0, y=1.0, z=1.0)
        m.points = [p0, p1]

        self.marker_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TestMarkerPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
