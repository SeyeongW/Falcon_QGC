#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped


class Phase0Precheck(Node):
    def __init__(self):
        super().__init__("phase0_precheck")

        self.state = None
        self.local_pose = None
        self.global_fix = None
        self.battery = None

        # MAVROS2 sensor/local/global 계열은 BEST_EFFORT로 나오는 경우가 많아서
        # 구독자도 BEST_EFFORT로 맞춰준다.
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # /mavros/state는 보통 기본 QoS로 잘 받지만,
        # 혹시 몰라 BEST_EFFORT로 통일한다.
        self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            self.sensor_qos,
        )

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_pose_callback,
            self.sensor_qos,
        )

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.global_fix_callback,
            self.sensor_qos,
        )

        self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.battery_callback,
            self.sensor_qos,
        )

    def state_callback(self, msg):
        self.state = msg

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def global_fix_callback(self, msg):
        self.global_fix = msg

    def battery_callback(self, msg):
        self.battery = msg

    def wait_for_data(self, timeout_sec=60.0):
        # PX4's EKF needs GPS lock + convergence before local/global position are
        # published (typically 30-60 s after SITL boot), so wait generously and
        # log progress rather than failing after a few seconds.
        start_time = time.time()
        last_progress = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            state_ok = self.state is not None
            local_ok = self.local_pose is not None
            global_ok = self.global_fix is not None
            battery_ok = self.battery is not None

            if state_ok and local_ok and global_ok and battery_ok:
                return True

            elapsed = time.time() - start_time

            # Progress log every 5 s so it's clear what is still missing.
            if elapsed - last_progress >= 5.0:
                last_progress = elapsed
                missing = [
                    name for name, ok in (
                        ("state", state_ok), ("local_pose", local_ok),
                        ("global_fix", global_ok), ("battery", battery_ok),
                    ) if not ok
                ]
                self.get_logger().info(
                    f"waiting for MAVROS/PX4 data ({elapsed:.0f}/{timeout_sec:.0f}s) — "
                    f"still missing: {', '.join(missing)}")

            if elapsed > timeout_sec:
                self.get_logger().error("Timeout while waiting for MAVROS/PX4 data")
                self.get_logger().error(f"state received: {state_ok}")
                self.get_logger().error(f"local pose received: {local_ok}")
                self.get_logger().error(f"global fix received: {global_ok}")
                self.get_logger().error(f"battery received: {battery_ok}")
                # Diagnostic hint: connected but no position => EKF/GPS not ready.
                if state_ok and self.state.connected and (not local_ok or not global_ok):
                    self.get_logger().error(
                        "FCU is connected but has no position estimate. "
                        "PX4 has no valid EKF/GPS fix yet — check PX4 system_status "
                        "(should be >=3 STANDBY), GPS lock, and the PX4<->Gazebo sim link.")
                return False

        return False

    def check_state(self):
        if self.state is None:
            self.get_logger().error("No MAVROS state received")
            return False

        self.get_logger().info("========== MAVROS STATE ==========")
        self.get_logger().info(f"connected: {self.state.connected}")
        self.get_logger().info(f"armed: {self.state.armed}")
        self.get_logger().info(f"mode: {self.state.mode}")

        if not self.state.connected:
            self.get_logger().error("FC is not connected through MAVROS")
            return False

        return True

    def check_local_position(self):
        if self.local_pose is None:
            self.get_logger().error("No local position received")
            return False

        p = self.local_pose.pose.position

        self.get_logger().info("========== LOCAL POSITION ==========")
        self.get_logger().info(f"x: {p.x:.2f}, y: {p.y:.2f}, z: {p.z:.2f}")

        return True

    def check_global_position(self):
        if self.global_fix is None:
            self.get_logger().error("No global position received")
            return False

        fix = self.global_fix

        self.get_logger().info("========== GLOBAL POSITION ==========")
        self.get_logger().info(f"lat: {fix.latitude:.7f}")
        self.get_logger().info(f"lon: {fix.longitude:.7f}")
        self.get_logger().info(f"alt: {fix.altitude:.2f}")
        self.get_logger().info(f"status: {fix.status.status}")

        # NavSatStatus:
        # -1 = no fix
        #  0 = fix
        #  1 = SBAS fix
        #  2 = GBAS fix
        if fix.status.status < 0:
            self.get_logger().error("GPS fix is not valid")
            return False

        return True

    def check_battery(self):
        if self.battery is None:
            self.get_logger().error("No battery data received")
            return False

        percentage = self.battery.percentage

        self.get_logger().info("========== BATTERY ==========")
        self.get_logger().info(f"voltage: {self.battery.voltage:.2f} V")

        if percentage >= 0.0:
            self.get_logger().info(f"percentage: {percentage * 100:.1f} %")
        else:
            self.get_logger().warn("battery percentage is unknown")

        if 0.0 < percentage < 0.20:
            self.get_logger().error("Battery is too low")
            return False

        return True

    def run(self):
        self.get_logger().info("[PHASE 0] Precheck start")

        if not self.wait_for_data(timeout_sec=60.0):
            return False

        checks = [
            self.check_state(),
            self.check_local_position(),
            self.check_global_position(),
            self.check_battery(),
        ]

        if all(checks):
            self.get_logger().info("[PHASE 0] Precheck complete: READY")
            return True

        self.get_logger().error("[PHASE 0] Precheck failed")
        return False


def main():
    rclpy.init()

    node = Phase0Precheck()

    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user")
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
