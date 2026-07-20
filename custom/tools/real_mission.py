#!/usr/bin/env python3
"""Real SITL mission over MAVROS — the honest replacement for fake_mavlink.py.

Instead of injecting fake MAVLink, this drives a REAL SITL (PX4 or ArduPilot)
through MAVROS exactly like a field flight, so QGroundControl (connected to the
same SITL on UDP 14550) shows genuine telemetry: map/GPS, arming, and — via the
ROS bridge — the live `/mavros/rc/out` actuator outputs.

Stack-selectable via the `stack` param (default ardupilot = the current sim;
switch to px4 for the eventual PX4 SITL target):

    stack=ardupilot   flight mode GUIDED,   return mode RTL
    stack=px4         flight mode OFFBOARD, return mode AUTO.RTL
                      (PX4 also requires setpoints to be streaming *before* the
                       OFFBOARD switch, which this node already does.)

Flight sequence (mirrors PX4-ROS2/offboard/src/offboard_sim_waypoints.cpp):

    CONNECT   wait for MAVROS <-> FCU link (/mavros/state.connected)
    WARMUP    wait for local position, set flight mode, then ARM. The autopilot
              refuses to arm until its pre-arm checks pass (GPS/EKF lock, and the
              GCS/QGC datalink) — so this step naturally "waits until it can arm"
              and logs why it hasn't yet.
    TAKEOFF   climb to `alt` by ramping the local-position setpoint
    WAYPOINTS fly a square (side `square_m`) at `alt`, advancing on arrival
    RTL       switch to the return mode, land, then exit on disarm

Prereqs (run these yourself, as in the project README):
    ArduPilot:  sim_vehicle.py -v ArduCopter -f JSON --console --map --out=udp:127.0.0.1:14551
    PX4:        make px4_sitl gz_x500        (PX4 SITL, later)
    MAVROS:     ros2 launch mavros apm.launch  fcu_url:=udp://:14551@     # ArduPilot
                ros2 launch mavros px4.launch  fcu_url:=udp://:14540@     # PX4
    Mission:    python3 custom/tools/real_mission.py [-p stack:=px4]

Tunables (ROS params):  stack (ardupilot|px4), alt (m), square_m (m), reach_m (m), mavros_ns
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class RealMission(Node):
    def __init__(self) -> None:
        super().__init__("vtol_gcs_real_mission")

        self._alt = float(self.declare_parameter("alt", 10.0).value)
        self._side = float(self.declare_parameter("square_m", 20.0).value)
        self._reach = float(self.declare_parameter("reach_m", 1.5).value)
        ns = str(self.declare_parameter("mavros_ns", "/mavros").value).rstrip("/")

        # Flight-stack-specific mode names.
        stack = str(self.declare_parameter("stack", "ardupilot").value).lower()
        if stack == "px4":
            self._flight_mode, self._rtl_mode = "OFFBOARD", "AUTO.RTL"
        else:
            self._flight_mode, self._rtl_mode = "GUIDED", "RTL"
        self._stack = stack

        # Square path in local ENU (origin = spawn/home), flown at `alt`.
        s = self._side
        self._waypoints = [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]
        self._wp_idx = 0

        self._state = State()
        self._pos = None            # (x, y, z) once local position is available
        self._phase = "CONNECT"
        self._ticks = 0
        self._takeoff_t = 0.0
        self._last_log = ""

        self._sp_pub = self.create_publisher(PoseStamped, f"{ns}/setpoint_position/local", 10)
        self.create_subscription(State, f"{ns}/state", self._on_state, 10)
        self.create_subscription(PoseStamped, f"{ns}/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)

        self._arming = self.create_client(CommandBool, f"{ns}/cmd/arming")
        self._set_mode = self.create_client(SetMode, f"{ns}/set_mode")

        # 20 Hz: both GUIDED and (especially) PX4 OFFBOARD want a continuous
        # setpoint stream — PX4 also needs it flowing *before* the mode switch.
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"real mission up [{stack}: {self._flight_mode}/{self._rtl_mode}]: "
            f"alt={self._alt} square={self._side}m ns={ns}")

    # --- callbacks -----------------------------------------------------------
    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._pos = (p.x, p.y, p.z)

    # --- helpers -------------------------------------------------------------
    def _set_flight_mode(self, mode: str) -> None:
        req = SetMode.Request()
        req.custom_mode = mode
        self._set_mode.call_async(req)

    def _arm(self) -> None:
        req = CommandBool.Request()
        req.value = True
        fut = self._arming.call_async(req)

        def _done(f):
            try:
                if not f.result().success:
                    self._log_once("arm rejected — ArduPilot pre-arm not satisfied "
                                   "(GPS/EKF lock, or GCS/QGC datalink). Is QGC connected?")
            except Exception:  # noqa: BLE001
                pass
        fut.add_done_callback(_done)

    def _log_once(self, text: str) -> None:
        if text != self._last_log:
            self.get_logger().info(text)
            self._last_log = text

    def _publish_sp(self, x: float, y: float, z: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self._sp_pub.publish(msg)

    # --- state machine -------------------------------------------------------
    def _tick(self) -> None:
        self._ticks += 1

        if self._phase == "CONNECT":
            if self._state.connected:
                self._log_once(">>> MAVROS connected to FCU")
                self._phase = "WARMUP"
            else:
                self._log_once("waiting for MAVROS <-> FCU link ...")
            return

        # From here on keep a setpoint streaming so GUIDED/OFFBOARD stays happy.
        if self._pos is None:
            self._log_once("waiting for /mavros/local_position/pose (EKF origin) ...")
            self._publish_sp(0.0, 0.0, 0.0)
            return

        px, py, _ = self._pos

        if self._phase == "WARMUP":
            self._publish_sp(px, py, 0.0)
            if self._state.mode != self._flight_mode:
                self._log_once(f"requesting {self._flight_mode} ...")
                if self._ticks % 20 == 0:
                    self._set_flight_mode(self._flight_mode)
            elif not self._state.armed:
                self._log_once(f"{self._flight_mode} set — arming (waiting until pre-arm/QGC ok) ...")
                if self._ticks % 20 == 0:
                    self._arm()
            else:
                self._log_once(f">>> ARMED in {self._flight_mode} — taking off")
                self._phase = "TAKEOFF"
                self._takeoff_t = 0.0
            return

        if self._phase == "TAKEOFF":
            self._takeoff_t += 0.05
            z = min(self._alt, self._takeoff_t * 1.5)   # ~1.5 m/s climb ramp
            self._publish_sp(px, py, self._alt)
            if self._pos[2] >= self._alt - 0.5:
                self._log_once(">>> reached altitude — flying waypoints")
                self._phase = "WAYPOINTS"
                self._wp_idx = 0
            else:
                self._publish_sp(px, py, z)
            return

        if self._phase == "WAYPOINTS":
            wx, wy = self._waypoints[self._wp_idx]
            self._publish_sp(wx, wy, self._alt)
            if math.hypot(px - wx, py - wy) < self._reach:
                self._log_once(f">>> reached waypoint {self._wp_idx + 1}/{len(self._waypoints)}")
                self._wp_idx += 1
                self._last_log = ""     # allow the next waypoint log
                if self._wp_idx >= len(self._waypoints):
                    self._log_once(">>> waypoints done — RTL")
                    self._phase = "RTL"
                    self._rtl_sent = False
            return

        if self._phase == "RTL":
            if not getattr(self, "_rtl_sent", False):
                self._set_flight_mode(self._rtl_mode)
                self._rtl_sent = True
            if not self._state.armed:
                self._log_once(">>> MISSION COMPLETE / DISARMED")
                rclpy.shutdown()
            return


def main() -> None:
    rclpy.init()
    node = RealMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
