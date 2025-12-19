#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class JarabotSimSafetyStop(Node):
    """
    전/후(앞/뒤)만 안전정지 적용 (좌/우 회전은 간섭 안 함)

    - /cmd_vel_raw 입력
    - /cmd_vel 출력
    - /scan에서 전방/후방 섹터의 최소거리만 사용
    - 전진 시 front가 stop_dist 이하면 linear.x=0
    - 후진 시 back이 stop_dist 이하면 linear.x=0
    - angular.z(회전)는 그대로 통과
    """

    def __init__(self):
        super().__init__('jarabot_sim_safety_stop')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_in_topic', '/cmd_vel_raw')
        self.declare_parameter('cmd_out_topic', '/cmd_vel')

        # Safety distances
        self.declare_parameter('stop_dist', 0.50)
        self.declare_parameter('release_dist', 0.70)
        self.declare_parameter('hold_ms', 600)
        self.declare_parameter('scan_timeout_ms', 300)

        # Behavior
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_ms', 0)  # 0이면 입력 없어도 마지막 cmd 유지

        # Front/Back sector width
        self.declare_parameter('front_half_deg', 35.0)  # 전방 ±35°
        self.declare_parameter('back_half_deg', 35.0)   # 후방 ±35° (π 기준)

        # (옵션) 너무 가까우면 관통 방지용 강제 정지
        self.declare_parameter('critical_dist', 0.18)
        self.declare_parameter('critical_mode_stop', True)

        # Load params
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_in_topic = self.get_parameter('cmd_in_topic').value
        self.cmd_out_topic = self.get_parameter('cmd_out_topic').value

        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.release_dist = float(self.get_parameter('release_dist').value)
        self.hold_ms = int(self.get_parameter('hold_ms').value)
        self.scan_timeout_ms = int(self.get_parameter('scan_timeout_ms').value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.cmd_timeout_ms = int(self.get_parameter('cmd_timeout_ms').value)

        self.front_half = math.radians(float(self.get_parameter('front_half_deg').value))
        self.back_half = math.radians(float(self.get_parameter('back_half_deg').value))

        self.critical_dist = float(self.get_parameter('critical_dist').value)
        self.critical_mode_stop = bool(self.get_parameter('critical_mode_stop').value)

        if self.release_dist <= self.stop_dist:
            self.release_dist = self.stop_dist + 0.2
            self.get_logger().warn("release_dist <= stop_dist → release_dist 자동 보정")

        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 20.0
            self.get_logger().warn("publish_rate_hz <= 0 → 20Hz로 보정")

        # Pub/Sub
        self.pub_cmd = self.create_publisher(Twist, self.cmd_out_topic, 10)
        self.sub_cmd = self.create_subscription(Twist, self.cmd_in_topic, self.on_cmd, 10)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)

        # State
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.last_scan_time = self.get_clock().now()

        self.min_front = float('inf')
        self.min_back = float('inf')

        # blocked/hold: front/back 각각
        self.block_front = False
        self.block_back = False
        self.hold_front_until = self.get_clock().now()
        self.hold_back_until = self.get_clock().now()

        # Timer
        self.pub_timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_loop)

        self.get_logger().info(
            f"[Jarabot Safety Stop] ON (Front/Back only) | scan={self.scan_topic}, cmd_in={self.cmd_in_topic}, cmd_out={self.cmd_out_topic}, "
            f"stop={self.stop_dist:.2f}m, release={self.release_dist:.2f}m, critical={self.critical_dist:.2f}m, "
            f"front_half={math.degrees(self.front_half):.0f}deg, back_half={math.degrees(self.back_half):.0f}deg, "
            f"pub={self.publish_rate_hz:.1f}Hz, cmd_timeout_ms={self.cmd_timeout_ms}"
        )

    # ---------- callbacks ----------
    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_scan(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now()
        now = self.get_clock().now()

        mf = mb = float('inf')

        a = msg.angle_min
        for r in msg.ranges:
            if r is None or not math.isfinite(r) or r <= 0.0:
                a += msg.angle_increment
                continue
            if (msg.range_min > 0.0 and r < msg.range_min) or (msg.range_max > 0.0 and r > msg.range_max):
                a += msg.angle_increment
                continue

            ang = wrap_pi(a)

            # front: 0 기준 ±front_half
            if abs(ang) <= self.front_half:
                if r < mf:
                    mf = r

            # back: π(또는 -π) 기준 ±back_half  => | |ang| - π | <= back_half
            if abs(abs(ang) - math.pi) <= self.back_half:
                if r < mb:
                    mb = r

            a += msg.angle_increment

        self.min_front, self.min_back = mf, mb

        hold_dur = Duration(nanoseconds=self.hold_ms * 1_000_000)

        # front blocked
        if not self.block_front:
            if self.min_front <= self.stop_dist:
                self.block_front = True
                self.hold_front_until = now + hold_dur
        else:
            if now >= self.hold_front_until and self.min_front >= self.release_dist:
                self.block_front = False

        # back blocked
        if not self.block_back:
            if self.min_back <= self.stop_dist:
                self.block_back = True
                self.hold_back_until = now + hold_dur
        else:
            if now >= self.hold_back_until and self.min_back >= self.release_dist:
                self.block_back = False

    # ---------- helpers ----------
    def is_scan_timeout(self) -> bool:
        now = self.get_clock().now()
        dt_ms = (now - self.last_scan_time).nanoseconds / 1e6
        return dt_ms > self.scan_timeout_ms

    # ---------- publish loop ----------
    def publish_loop(self):
        out = Twist()
        out.linear = self.last_cmd.linear
        out.angular = self.last_cmd.angular  # ✅ 회전은 건드리지 않음

        # cmd timeout
        if self.cmd_timeout_ms > 0:
            now = self.get_clock().now()
            cmd_age_ms = (now - self.last_cmd_time).nanoseconds / 1e6
            if cmd_age_ms > self.cmd_timeout_ms:
                out.linear.x = 0.0
                out.angular.z = 0.0
                self.pub_cmd.publish(out)
                return

        # scan timeout이면 정지
        if self.is_scan_timeout():
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.pub_cmd.publish(out)
            return

        # ✅ 관통 방지: 너무 가까우면 무조건 정지(선속도만이라도)
        if min(self.min_front, self.min_back) <= self.critical_dist and self.critical_mode_stop:
            out.linear.x = 0.0
            self.pub_cmd.publish(out)
            return

        # ✅ 전/후만 차단 (회전은 통과)
        vx = out.linear.x
        if vx > 0.0 and self.block_front:
            out.linear.x = 0.0
        elif vx < 0.0 and self.block_back:
            out.linear.x = 0.0

        self.pub_cmd.publish(out)


def main():
    rclpy.init()
    node = JarabotSimSafetyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
