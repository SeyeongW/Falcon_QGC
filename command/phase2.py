#!/usr/bin/env python3
"""
phase2_mavros_center_go_above_precision_land.py

Phase 2 for the current MAVROS-based Phase 1 handoff.

Assumed Phase 1 final state:
- PX4 mode: AUTO.LOITER  (QGC Hold/Loiter equivalent)
- VTOL state: MULTICOPTER
- Vehicle is armed
- MAVROS is connected

Phase 2:
1) Waits for MAVROS state, local pose, and target_info.
2) Keeps the gimbal at the Phase-1 geometry angle (-30 deg by default).
3) Enters OFFBOARD and actively centers the target with vehicle yaw and altitude commands.
4) Locks target ground position only after the target is centered.
5) Sets the gimbal downward and moves above the estimated survivor/target.
6) Starts precision landing in the same node:
   - uses downward camera ex/ey to command horizontal velocity
   - descends at the same time
   - sends LAND when low and centered
7) Prints PHASE 2 SUCCESS after LAND command is sent or landing condition is completed.

Important:
- If /mission/target_info is not centered, Phase 2 does NOT estimate immediately.
- Instead, it actively uses body yaw and altitude to center the target first.
- This preserves the original verified geometry: fixed -30 deg gimbal + centered image + triangle projection.

Frame note:
- MAVROS /mavros/local_position/pose is ENU-like:
  x = East, y = North, z = Up.
- The target ray is projected in this local ENU frame.
"""

import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32MultiArray, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def finite_or_default(v, default):
    if v is None or not math.isfinite(v):
        return default
    return v


def yaw_from_quaternion(q):
    # ROS ENU yaw from geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    qz = math.sin(half)
    qw = math.cos(half)
    return (0.0, 0.0, qz, qw)


@dataclass
class TargetInfo:
    detected: bool = False
    ex: float = 0.0
    ey: float = 0.0
    bearing_x: float = 0.0
    bearing_y: float = 0.0
    area_ratio: float = 0.0
    last_t: float = 0.0


@dataclass
class LocalPose:
    valid: bool = False
    x: float = 0.0   # East
    y: float = 0.0   # North
    z: float = 0.0   # Up
    yaw: float = 0.0 # ENU yaw


class Phase2MavrosGoAboveSurvivor(Node):
    def __init__(self):
        super().__init__("phase2_mavros_go_above_survivor")

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter("target_info_topic", "/mission/target_info")

        self.declare_parameter("gimbal_yaw_topic", "/model/gimbal_model/command/gimbal_yaw")
        self.declare_parameter("gimbal_pitch_topic", "/model/gimbal_model/command/gimbal_pitch")
        self.declare_parameter("gimbal_roll_topic", "/model/gimbal_model/command/gimbal_roll")

        # Gimbal pose used during Phase 1 target centering.
        self.declare_parameter("fixed_gimbal_yaw", 0.0)
        self.declare_parameter("fixed_gimbal_pitch", -0.5236)
        self.declare_parameter("fixed_gimbal_roll", 0.0)

        # Image bearing convention. For the current /mission/target_info,
        # bearing_y is positive when the target is lower in the image,
        # so it must be subtracted from the fixed downward pitch.
        self.declare_parameter("bearing_x_sign", 1.0)
        self.declare_parameter("bearing_y_sign", -1.0)

        # Safety gate for target projection. If the ray intersection is
        # unrealistically far, do not fly there.
        self.declare_parameter("max_target_distance_m", 120.0)

        # Gimbal pose after Phase 2 starts.
        self.declare_parameter("downward_gimbal_yaw", 0.0)
        self.declare_parameter("downward_gimbal_pitch", -1.5708)
        self.declare_parameter("downward_gimbal_roll", 0.0)

        # MAVROS local_position/pose uses z-up. If Gazebo ground is z=0, keep 0.
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("target_hover_altitude_m", 20.0)

        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("target_timeout_sec", 1.0)

        # Lock the target only when it is actually near the image center.
        # This reflects the original verified logic:
        # gimbal pitch fixed at -30 deg -> body yaw/z centers the target ->
        # use the centered bearing for the trigonometric ground intersection.
        self.declare_parameter("require_centered_target_for_lock", True)
        self.declare_parameter("center_x_threshold", 0.06)
        self.declare_parameter("center_y_threshold", 0.12)
        self.declare_parameter("center_hold_sec", 0.15)

        # Active centering before target lock. This ports the verified
        # center_body_yaw.py behavior into MAVROS Phase 2.
        self.declare_parameter("active_center_before_lock", True)
        self.declare_parameter("k_body_yaw", 0.45)
        # max_body_yaw_step is now a per-cycle yaw-rate limiter, not a direct P gain.
        self.declare_parameter("max_body_yaw_step", 0.008)
        self.declare_parameter("body_yaw_limit", 1.57)
        self.declare_parameter("invert_body_yaw", True)
        self.declare_parameter("yaw_deadband", 0.025)
        self.declare_parameter("max_yaw_error_cmd", 0.18)
        self.declare_parameter("k_vertical_z", 1.20)
        self.declare_parameter("max_vertical_step", 0.06)
        self.declare_parameter("vertical_z_limit", 25.0)
        self.declare_parameter("min_center_altitude_m", 5.0)
        self.declare_parameter("max_center_altitude_m", 80.0)
        # ENU z is Up. The old DDS/NED code used positive z as Down.
        # Default -1.0 means ey>0 commands descent in ENU.
        self.declare_parameter("vertical_z_sign", -1.0)

        self.declare_parameter("initial_data_timeout_sec", 20.0)
        self.declare_parameter("offboard_warmup_cycles", 40)
        self.declare_parameter("offboard_request_interval_sec", 1.0)
        self.declare_parameter("auto_arm", False)

        self.declare_parameter("xy_alpha", 0.18)
        self.declare_parameter("z_alpha", 0.60)
        self.declare_parameter("max_xy_step_m", 1.5)
        self.declare_parameter("max_z_step_m", 0.8)

        self.declare_parameter("arrival_xy_threshold_m", 0.7)
        self.declare_parameter("arrival_z_threshold_m", 0.5)
        self.declare_parameter("arrival_hold_sec", 2.0)
        self.declare_parameter("mission_timeout_sec", 180.0)

        # If true, target position is estimated once and then fixed.
        self.declare_parameter("estimate_once_on_start", True)

        # ------------------------------------------------------------
        # Precision landing parameters
        # ------------------------------------------------------------
        self.declare_parameter("enable_precision_land_after_arrival", True)
        self.declare_parameter("precision_target_timeout_sec", 0.5)

        # Downward-camera image error -> body velocity.
        # body_right corrects image x, body_forward corrects image y.
        # Damped defaults are intentionally conservative to avoid oscillation
        # during the final descent phase.
        self.declare_parameter("land_k_xy", 0.12)
        self.declare_parameter("land_max_xy_speed", 0.85)
        self.declare_parameter("land_min_xy_speed_deadband", 0.05)
        self.declare_parameter("land_velocity_smoothing_alpha", 0.18)

        self.declare_parameter("land_invert_x", False)
        self.declare_parameter("land_invert_y", False)
        self.declare_parameter("land_swap_xy", False)
        self.declare_parameter("image_x_to_body_right_sign", 1.0)
        self.declare_parameter("image_y_to_body_forward_sign", -1.0)

        # Simultaneous descent while correcting image error.
        self.declare_parameter("descent_speed_fast", 0.45)
        self.declare_parameter("descent_speed_min", 0.10)
        self.declare_parameter("fast_descent_error", 0.10)
        self.declare_parameter("slow_descent_error", 0.45)
        self.declare_parameter("near_ground_altitude_m", 3.0)
        self.declare_parameter("near_ground_descent_speed", 0.12)
        self.declare_parameter("extreme_error_threshold", 1.20)

        # Final landing condition.
        self.declare_parameter("land_altitude_m", 0.8)
        self.declare_parameter("final_center_threshold", 0.12)
        self.declare_parameter("final_center_hold_sec", 0.5)
        self.declare_parameter("send_land_command", True)

        # ------------------------------------------------------------
        # Load parameters
        # ------------------------------------------------------------
        self.target_info_topic = str(self.get_parameter("target_info_topic").value)

        self.gimbal_yaw_topic = str(self.get_parameter("gimbal_yaw_topic").value)
        self.gimbal_pitch_topic = str(self.get_parameter("gimbal_pitch_topic").value)
        self.gimbal_roll_topic = str(self.get_parameter("gimbal_roll_topic").value)

        self.fixed_gimbal_yaw = float(self.get_parameter("fixed_gimbal_yaw").value)
        self.fixed_gimbal_pitch = float(self.get_parameter("fixed_gimbal_pitch").value)
        self.fixed_gimbal_roll = float(self.get_parameter("fixed_gimbal_roll").value)
        self.bearing_x_sign = float(self.get_parameter("bearing_x_sign").value)
        self.bearing_y_sign = float(self.get_parameter("bearing_y_sign").value)
        self.max_target_distance_m = abs(float(self.get_parameter("max_target_distance_m").value))

        self.downward_gimbal_yaw = float(self.get_parameter("downward_gimbal_yaw").value)
        self.downward_gimbal_pitch = float(self.get_parameter("downward_gimbal_pitch").value)
        self.downward_gimbal_roll = float(self.get_parameter("downward_gimbal_roll").value)

        self.ground_z = float(self.get_parameter("ground_z").value)
        self.target_hover_altitude_m = abs(float(self.get_parameter("target_hover_altitude_m").value))

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / max(self.control_rate_hz, 1.0)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.require_centered_target_for_lock = bool(self.get_parameter("require_centered_target_for_lock").value)
        self.center_x_threshold = float(self.get_parameter("center_x_threshold").value)
        self.center_y_threshold = float(self.get_parameter("center_y_threshold").value)
        self.center_hold_sec = float(self.get_parameter("center_hold_sec").value)
        self.active_center_before_lock = bool(self.get_parameter("active_center_before_lock").value)
        self.k_body_yaw = float(self.get_parameter("k_body_yaw").value)
        self.max_body_yaw_step = abs(float(self.get_parameter("max_body_yaw_step").value))
        self.body_yaw_limit = abs(float(self.get_parameter("body_yaw_limit").value))
        self.invert_body_yaw = bool(self.get_parameter("invert_body_yaw").value)
        self.yaw_deadband = abs(float(self.get_parameter("yaw_deadband").value))
        self.max_yaw_error_cmd = abs(float(self.get_parameter("max_yaw_error_cmd").value))
        self.k_vertical_z = float(self.get_parameter("k_vertical_z").value)
        self.max_vertical_step = abs(float(self.get_parameter("max_vertical_step").value))
        self.vertical_z_limit = abs(float(self.get_parameter("vertical_z_limit").value))
        self.min_center_altitude_m = abs(float(self.get_parameter("min_center_altitude_m").value))
        self.max_center_altitude_m = abs(float(self.get_parameter("max_center_altitude_m").value))
        self.vertical_z_sign = float(self.get_parameter("vertical_z_sign").value)
        self.initial_data_timeout_sec = float(self.get_parameter("initial_data_timeout_sec").value)
        self.offboard_warmup_cycles = int(self.get_parameter("offboard_warmup_cycles").value)
        self.offboard_request_interval_sec = float(self.get_parameter("offboard_request_interval_sec").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)

        self.xy_alpha = float(self.get_parameter("xy_alpha").value)
        self.z_alpha = float(self.get_parameter("z_alpha").value)
        self.max_xy_step_m = abs(float(self.get_parameter("max_xy_step_m").value))
        self.max_z_step_m = abs(float(self.get_parameter("max_z_step_m").value))

        self.arrival_xy_threshold_m = float(self.get_parameter("arrival_xy_threshold_m").value)
        self.arrival_z_threshold_m = float(self.get_parameter("arrival_z_threshold_m").value)
        self.arrival_hold_sec = float(self.get_parameter("arrival_hold_sec").value)
        self.mission_timeout_sec = float(self.get_parameter("mission_timeout_sec").value)
        self.estimate_once_on_start = bool(self.get_parameter("estimate_once_on_start").value)

        self.enable_precision_land_after_arrival = bool(self.get_parameter("enable_precision_land_after_arrival").value)
        self.precision_target_timeout_sec = float(self.get_parameter("precision_target_timeout_sec").value)

        self.land_k_xy = float(self.get_parameter("land_k_xy").value)
        self.land_max_xy_speed = abs(float(self.get_parameter("land_max_xy_speed").value))
        self.land_min_xy_speed_deadband = abs(float(self.get_parameter("land_min_xy_speed_deadband").value))
        self.land_velocity_smoothing_alpha = float(self.get_parameter("land_velocity_smoothing_alpha").value)

        self.land_invert_x = bool(self.get_parameter("land_invert_x").value)
        self.land_invert_y = bool(self.get_parameter("land_invert_y").value)
        self.land_swap_xy = bool(self.get_parameter("land_swap_xy").value)
        self.image_x_to_body_right_sign = float(self.get_parameter("image_x_to_body_right_sign").value)
        self.image_y_to_body_forward_sign = float(self.get_parameter("image_y_to_body_forward_sign").value)

        self.descent_speed_fast = abs(float(self.get_parameter("descent_speed_fast").value))
        self.descent_speed_min = abs(float(self.get_parameter("descent_speed_min").value))
        self.fast_descent_error = float(self.get_parameter("fast_descent_error").value)
        self.slow_descent_error = float(self.get_parameter("slow_descent_error").value)
        self.near_ground_altitude_m = float(self.get_parameter("near_ground_altitude_m").value)
        self.near_ground_descent_speed = abs(float(self.get_parameter("near_ground_descent_speed").value))
        self.extreme_error_threshold = float(self.get_parameter("extreme_error_threshold").value)

        self.land_altitude_m = float(self.get_parameter("land_altitude_m").value)
        self.final_center_threshold = float(self.get_parameter("final_center_threshold").value)
        self.final_center_hold_sec = float(self.get_parameter("final_center_hold_sec").value)
        self.send_land_command_enabled = bool(self.get_parameter("send_land_command").value)

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------
        self.state: Optional[State] = None
        self.extended_state: Optional[ExtendedState] = None
        self.target = TargetInfo()
        self.local = LocalPose()

        self.target_locked = False
        self.center_start_t: Optional[float] = None
        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None
        self.target_z: Optional[float] = None
        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_z: Optional[float] = None

        self.cmd_x: Optional[float] = None
        self.cmd_y: Optional[float] = None
        self.cmd_z: Optional[float] = None
        self.cmd_yaw: float = 0.0

        # Active-centering reference. We hold XY, then adjust only yaw and z
        # until the target is centered enough to lock the ground intersection.
        self.center_ref_locked = False
        self.center_hold_x = 0.0
        self.center_hold_y = 0.0
        self.center_initial_z = 0.0
        self.center_initial_yaw = 0.0
        self.center_z_cmd = 0.0
        self.center_yaw_cmd = 0.0

        self.loop_count = 0
        self.start_time = self.now_s()
        self.last_mode_request_t = 0.0
        self.arrival_start_t: Optional[float] = None
        self.precision_land_started = False
        self.final_center_start_t: Optional[float] = None
        self.land_command_sent = False
        self.success = False
        self.shutdown_requested = False

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.raw_vx = 0.0
        self.raw_vy = 0.0
        self.raw_vz = 0.0
        self.last_v_forward = 0.0
        self.last_v_right = 0.0

        # ------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------
        normal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ------------------------------------------------------------
        # ROS IO
        # ------------------------------------------------------------
        self.create_subscription(State, "/mavros/state", self.state_cb, mavros_qos)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self.extended_state_cb, mavros_qos)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.local_pose_cb, mavros_qos)
        self.create_subscription(Float32MultiArray, self.target_info_topic, self.target_cb, normal_qos)

        self.pub_setpoint = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.pub_velocity = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.pub_gimbal_yaw = self.create_publisher(Float64, self.gimbal_yaw_topic, 10)
        self.pub_gimbal_pitch = self.create_publisher(Float64, self.gimbal_pitch_topic, 10)
        self.pub_gimbal_roll = self.create_publisher(Float64, self.gimbal_roll_topic, 10)

        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info("============================================================")
        self.get_logger().info("PHASE 2 MAVROS STARTED: AUTO.LOITER -> OFFBOARD -> go above target")
        self.get_logger().info(f"target_info_topic={self.target_info_topic}")
        self.get_logger().info(f"hover_altitude={self.target_hover_altitude_m:.2f} m, ground_z={self.ground_z:.2f}")
        self.get_logger().info(
            f"bearing_sign=({self.bearing_x_sign:+.1f}, {self.bearing_y_sign:+.1f}), "
            f"max_target_distance={self.max_target_distance_m:.1f} m"
        )
        self.get_logger().info(
            f"center_gate={self.require_centered_target_for_lock}, "
            f"active_center={self.active_center_before_lock}, "
            f"threshold=({self.center_x_threshold:.3f}, {self.center_y_threshold:.3f}), "
            f"hold={self.center_hold_sec:.2f}s"
        )
        self.get_logger().info(
            f"center_gains: k_yaw={self.k_body_yaw:.3f}, max_yaw_step={self.max_body_yaw_step:.3f}, "
            f"yaw_deadband={self.yaw_deadband:.3f}, max_yaw_error_cmd={self.max_yaw_error_cmd:.3f}, "
            f"invert_yaw={self.invert_body_yaw}, k_z={self.k_vertical_z:.2f}, "
            f"max_z_step={self.max_vertical_step:.2f}, z_limit={self.vertical_z_limit:.1f}, "
            f"alt_window=({self.min_center_altitude_m:.1f}, {self.max_center_altitude_m:.1f})m"
        )
        self.get_logger().info(
            f"precision_land={self.enable_precision_land_after_arrival}, "
            f"land_k_xy={self.land_k_xy:.2f}, land_max_xy_speed={self.land_max_xy_speed:.2f}, "
            f"descent_fast={self.descent_speed_fast:.2f}, descent_min={self.descent_speed_min:.2f}"
        )
        self.get_logger().info("Expected Phase 1 final state: MULTICOPTER + AUTO.LOITER + armed")
        self.get_logger().info("============================================================")

    # ------------------------------------------------------------
    # Time
    # ------------------------------------------------------------
    def now_s(self):
        return time.time()

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def state_cb(self, msg: State):
        self.state = msg

    def extended_state_cb(self, msg: ExtendedState):
        self.extended_state = msg

    def local_pose_cb(self, msg: PoseStamped):
        self.local.valid = True
        self.local.x = finite_or_default(float(msg.pose.position.x), self.local.x)
        self.local.y = finite_or_default(float(msg.pose.position.y), self.local.y)
        self.local.z = finite_or_default(float(msg.pose.position.z), self.local.z)
        self.local.yaw = finite_or_default(yaw_from_quaternion(msg.pose.orientation), self.local.yaw)

        if self.cmd_x is None:
            self.cmd_x = self.local.x
            self.cmd_y = self.local.y
            self.cmd_z = self.local.z
            self.cmd_yaw = self.local.yaw

    def target_cb(self, msg: Float32MultiArray):
        d = list(msg.data)
        if len(d) < 17:
            return

        self.target.detected = d[0] > 0.5
        self.target.ex = float(d[5])
        self.target.ey = float(d[6])
        self.target.bearing_x = float(d[7])
        self.target.bearing_y = float(d[8])
        self.target.area_ratio = float(d[14])
        self.target.last_t = self.now_s()

    # ------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------
    def target_fresh(self):
        return self.target.detected and (self.now_s() - self.target.last_t) <= self.target_timeout_sec

    def target_centered_now(self):
        return (
            self.target_fresh()
            and abs(self.target.ex) <= self.center_x_threshold
            and abs(self.target.ey) <= self.center_y_threshold
        )

    def target_centered_stable(self):
        if not self.require_centered_target_for_lock:
            return True

        now = self.now_s()
        if not self.target_centered_now():
            self.center_start_t = None
            return False

        if self.center_start_t is None:
            self.center_start_t = now
            return False

        return (now - self.center_start_t) >= self.center_hold_sec

    def mavros_ready(self):
        return self.state is not None and self.state.connected and self.local.valid

    def vtol_state_name(self):
        if self.extended_state is None:
            return "UNKNOWN"
        names = {
            0: "UNDEFINED",
            1: "TRANSITION_TO_FW",
            2: "TRANSITION_TO_MC",
            3: "MULTICOPTER",
            4: "FIXED_WING",
        }
        return names.get(self.extended_state.vtol_state, f"UNKNOWN({self.extended_state.vtol_state})")

    # ------------------------------------------------------------
    # Publishers / services
    # ------------------------------------------------------------
    def publish_gimbal_fixed(self):
        self.pub_gimbal_yaw.publish(Float64(data=float(self.fixed_gimbal_yaw)))
        self.pub_gimbal_pitch.publish(Float64(data=float(self.fixed_gimbal_pitch)))
        self.pub_gimbal_roll.publish(Float64(data=float(self.fixed_gimbal_roll)))

    def publish_gimbal_downward(self):
        self.pub_gimbal_yaw.publish(Float64(data=float(self.downward_gimbal_yaw)))
        self.pub_gimbal_pitch.publish(Float64(data=float(self.downward_gimbal_pitch)))
        self.pub_gimbal_roll.publish(Float64(data=float(self.downward_gimbal_roll)))

    def publish_setpoint(self, x, y, z, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub_setpoint.publish(msg)

    def publish_velocity_setpoint(self, vx, vy, vz, yaw):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.pub_velocity.publish(msg)

    def request_mode_async(self, mode_name):
        if not self.set_mode_client.service_is_ready():
            self.set_mode_client.wait_for_service(timeout_sec=0.05)
            return

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_name
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self._mode_result_cb(f, mode_name))
        self.get_logger().warn(f"[PHASE2] Request mode: {mode_name}")

    def _mode_result_cb(self, future, mode_name):
        try:
            result = future.result()
            self.get_logger().info(f"[PHASE2] set_mode {mode_name} result: {result}")
        except Exception as exc:
            self.get_logger().error(f"[PHASE2] set_mode {mode_name} failed: {exc}")

    def request_arm_async(self):
        if not self.arming_client.service_is_ready():
            self.arming_client.wait_for_service(timeout_sec=0.05)
            return

        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(self._arm_result_cb)
        self.get_logger().warn("[PHASE2] Request ARM")

    def _arm_result_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"[PHASE2] arm result: {result}")
        except Exception as exc:
            self.get_logger().error(f"[PHASE2] arm failed: {exc}")

    def request_land_async(self):
        if self.land_command_sent:
            return

        self.land_command_sent = True

        if self.land_client.service_is_ready():
            req = CommandTOL.Request()
            req.min_pitch = 0.0
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.altitude = 0.0
            future = self.land_client.call_async(req)
            future.add_done_callback(self._land_result_cb)
            self.get_logger().warn("[PHASE2_LAND] /mavros/cmd/land requested")
            return

        # Fallback. On PX4 this is usually accepted as AUTO.LAND.
        self.get_logger().warn("[PHASE2_LAND] /mavros/cmd/land unavailable. Trying AUTO.LAND mode.")
        self.request_mode_async("AUTO.LAND")

    def _land_result_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"[PHASE2_LAND] land result: {result}")
        except Exception as exc:
            self.get_logger().error(f"[PHASE2_LAND] land failed: {exc}")

    # ------------------------------------------------------------
    # Active centering before target lock
    # ------------------------------------------------------------
    def ensure_center_reference(self):
        if self.center_ref_locked:
            return
        if not self.local.valid:
            return
        self.center_hold_x = self.local.x
        self.center_hold_y = self.local.y
        self.center_initial_z = self.local.z
        self.center_initial_yaw = self.local.yaw
        self.center_z_cmd = self.local.z
        self.center_yaw_cmd = self.local.yaw
        self.center_ref_locked = True
        self.get_logger().warn(
            f"[PHASE2] Centering reference locked: "
            f"hold_xy=({self.center_hold_x:.2f},{self.center_hold_y:.2f}), "
            f"z={self.center_initial_z:.2f}, yaw={self.center_initial_yaw:+.3f}"
        )

    def update_centering_command(self):
        self.ensure_center_reference()
        if not self.center_ref_locked:
            return

        # If detection becomes stale, do not chase the last noisy value.
        # Just keep publishing the last safe setpoint.
        if not self.target_fresh():
            return

        # ------------------------------------------------------------
        # Damped yaw centering
        # ------------------------------------------------------------
        # Old behavior integrated yaw error every cycle. That made the vehicle
        # overshoot hard, especially near +/-pi yaw wrapping. Here yaw_cmd is a
        # bounded offset from the *current* yaw, then rate-limited.
        yaw_sign = -1.0 if self.invert_body_yaw else 1.0
        ex_eff = 0.0 if abs(self.target.ex) < self.yaw_deadband else self.target.ex

        desired_yaw_offset = clamp(
            yaw_sign * self.k_body_yaw * ex_eff,
            -self.max_yaw_error_cmd,
            self.max_yaw_error_cmd,
        )
        desired_yaw = wrap_pi(self.local.yaw + desired_yaw_offset)

        yaw_step = clamp(
            wrap_pi(desired_yaw - self.center_yaw_cmd),
            -self.max_body_yaw_step,
            self.max_body_yaw_step,
        )
        self.center_yaw_cmd = wrap_pi(self.center_yaw_cmd + yaw_step)

        # ------------------------------------------------------------
        # Damped altitude centering
        # ------------------------------------------------------------
        dz = clamp(
            self.vertical_z_sign * self.k_vertical_z * self.target.ey * self.dt,
            -self.max_vertical_step,
            self.max_vertical_step,
        )
        # MAVROS local_position/pose uses z-up.
        # Keep altitude command inside both the relative centering limit
        # and an absolute safe altitude window.
        z_min_by_limit = self.center_initial_z - self.vertical_z_limit
        z_max_by_limit = self.center_initial_z + self.vertical_z_limit
        z_min_by_altitude = self.ground_z + self.min_center_altitude_m
        z_max_by_altitude = self.ground_z + self.max_center_altitude_m

        z_min = max(z_min_by_limit, z_min_by_altitude)
        z_max = min(z_max_by_limit, z_max_by_altitude)

        self.center_z_cmd = clamp(
            self.center_z_cmd + dz,
            z_min,
            z_max,
        )

    def request_offboard_if_needed(self, now):
        # PX4/MAVROS needs setpoint streaming before OFFBOARD.
        if self.loop_count < self.offboard_warmup_cycles:
            return

        mode = self.state.mode if self.state is not None else "UNKNOWN"

        if self.auto_arm and self.state is not None and not self.state.armed:
            if now - self.last_mode_request_t >= self.offboard_request_interval_sec:
                self.request_arm_async()
                self.last_mode_request_t = now
                return

        if mode != "OFFBOARD" and now - self.last_mode_request_t >= self.offboard_request_interval_sec:
            self.request_mode_async("OFFBOARD")
            self.last_mode_request_t = now

    # ------------------------------------------------------------
    # Target estimation
    # ------------------------------------------------------------
    def estimate_and_lock_target(self):
        if self.target_locked and self.estimate_once_on_start:
            return True

        if not self.local.valid:
            return False

        if not self.target_fresh():
            return False

        if not self.target_centered_stable():
            return False

        # Camera ray in local ENU frame.
        # yaw=0 points local +x, yaw=+pi/2 points local +y.
        gimbal_yaw = self.fixed_gimbal_yaw + self.bearing_x_sign * self.target.bearing_x
        gimbal_pitch = self.fixed_gimbal_pitch + self.bearing_y_sign * self.target.bearing_y
        ray_yaw = wrap_pi(self.local.yaw + gimbal_yaw)

        horizontal = math.cos(gimbal_pitch)
        ray_x = horizontal * math.cos(ray_yaw)
        ray_y = horizontal * math.sin(ray_yaw)
        ray_z = math.sin(gimbal_pitch)  # negative when looking down in z-up frame

        if abs(ray_z) < 1e-4:
            self.get_logger().warn("[PHASE2] Invalid ray: almost parallel to ground.")
            return False

        t = (self.ground_z - self.local.z) / ray_z
        if t <= 0.0 or not math.isfinite(t):
            self.get_logger().warn(
                f"[PHASE2] Invalid ground intersection: t={t:.2f}, "
                f"local_z={self.local.z:.2f}, ground_z={self.ground_z:.2f}, ray_z={ray_z:.3f}"
            )
            return False

        if t > self.max_target_distance_m:
            self.get_logger().warn(
                f"[PHASE2] Rejected target estimate: distance={t:.2f}m exceeds "
                f"max_target_distance_m={self.max_target_distance_m:.2f}m. "
                f"pitch={gimbal_pitch:+.3f}, bearing=({self.target.bearing_x:+.3f}, {self.target.bearing_y:+.3f})"
            )
            return False

        self.target_x = self.local.x + t * ray_x
        self.target_y = self.local.y + t * ray_y
        self.target_z = self.ground_z

        self.goal_x = self.target_x
        self.goal_y = self.target_y
        self.goal_z = self.ground_z + self.target_hover_altitude_m

        self.target_locked = True

        self.get_logger().warn(
            f"[PHASE2] Target estimate locked: "
            f"target_enu=({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}), "
            f"goal=({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_z:.2f}), "
            f"bearing=({self.target.bearing_x:+.3f}, {self.target.bearing_y:+.3f}), "
            f"bearing_sign=({self.bearing_x_sign:+.1f}, {self.bearing_y_sign:+.1f}), "
            f"yaw={self.local.yaw:+.3f}, ray_yaw={ray_yaw:+.3f}, "
            f"pitch={gimbal_pitch:+.3f}, t={t:.2f}"
        )
        return True

    def limit_step(self, current, desired, max_step):
        return current + clamp(desired - current, -max_step, max_step)

    # ------------------------------------------------------------
    # Precision landing after go-above arrival
    # ------------------------------------------------------------
    def altitude_m(self):
        return max(self.local.z - self.ground_z, 0.0)

    def precision_target_fresh(self):
        return self.target.detected and (self.now_s() - self.target.last_t) <= self.precision_target_timeout_sec

    def landing_image_error(self):
        ex = self.target.ex
        ey = self.target.ey

        if self.land_invert_x:
            ex = -ex
        if self.land_invert_y:
            ey = -ey
        if self.land_swap_xy:
            ex, ey = ey, ex

        return ex, ey

    def compute_precision_horizontal_velocity_enu(self):
        ex, ey = self.landing_image_error()
        alt = max(self.altitude_m(), 0.5)

        v_forward = self.land_k_xy * alt * self.image_y_to_body_forward_sign * ey
        v_right = self.land_k_xy * alt * self.image_x_to_body_right_sign * ex

        if abs(v_forward) < self.land_min_xy_speed_deadband:
            v_forward = 0.0
        if abs(v_right) < self.land_min_xy_speed_deadband:
            v_right = 0.0

        v_forward = clamp(v_forward, -self.land_max_xy_speed, self.land_max_xy_speed)
        v_right = clamp(v_right, -self.land_max_xy_speed, self.land_max_xy_speed)

        yaw = self.local.yaw
        c = math.cos(yaw)
        s = math.sin(yaw)

        # ENU: yaw=0 faces +x(East), yaw=+pi/2 faces +y(North).
        # body_forward=(cos yaw, sin yaw), body_right=(sin yaw, -cos yaw)
        vx_enu = c * v_forward + s * v_right
        vy_enu = s * v_forward - c * v_right

        self.last_v_forward = v_forward
        self.last_v_right = v_right

        return vx_enu, vy_enu

    def compute_precision_descent_velocity_enu(self):
        if not self.precision_target_fresh():
            return 0.0

        ex, ey = self.landing_image_error()
        err = math.sqrt(ex * ex + ey * ey)
        alt = self.altitude_m()

        if alt <= self.near_ground_altitude_m:
            descent = self.near_ground_descent_speed
        elif err >= self.extreme_error_threshold:
            descent = self.descent_speed_min
        elif err <= self.fast_descent_error:
            descent = self.descent_speed_fast
        elif err >= self.slow_descent_error:
            descent = self.descent_speed_min
        else:
            ratio = (self.slow_descent_error - err) / max(
                self.slow_descent_error - self.fast_descent_error,
                1e-6,
            )
            descent = self.descent_speed_min + ratio * (self.descent_speed_fast - self.descent_speed_min)

        # ENU z is positive up. Negative z velocity descends.
        return -abs(descent)

    def smooth_precision_velocity(self, vx, vy, vz):
        a = clamp(self.land_velocity_smoothing_alpha, 0.001, 1.0)
        self.cmd_vx = (1.0 - a) * self.cmd_vx + a * vx
        self.cmd_vy = (1.0 - a) * self.cmd_vy + a * vy
        self.cmd_vz = (1.0 - a) * self.cmd_vz + a * vz

    def start_precision_landing(self):
        self.precision_land_started = True
        self.arrival_start_t = None
        self.final_center_start_t = None
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.raw_vx = 0.0
        self.raw_vy = 0.0
        self.raw_vz = 0.0

        self.get_logger().warn("============================================================")
        self.get_logger().warn("[PHASE2] GO_ABOVE complete. Starting precision landing.")
        self.get_logger().warn("[PHASE2_LAND] Gimbal is downward; using ex/ey velocity correction + descent.")
        self.get_logger().warn("============================================================")

    def update_precision_landing_control(self):
        self.publish_gimbal_downward()

        if not self.precision_target_fresh():
            self.raw_vx = 0.0
            self.raw_vy = 0.0
            self.raw_vz = 0.0
            self.smooth_precision_velocity(0.0, 0.0, 0.0)
            return

        vx, vy = self.compute_precision_horizontal_velocity_enu()
        vz = self.compute_precision_descent_velocity_enu()

        self.raw_vx = vx
        self.raw_vy = vy
        self.raw_vz = vz

        self.smooth_precision_velocity(vx, vy, vz)

    def check_precision_land_condition(self):
        if not self.local.valid or not self.precision_target_fresh():
            self.final_center_start_t = None
            return

        alt = self.altitude_m()
        ex, ey = self.landing_image_error()
        err = math.sqrt(ex * ex + ey * ey)

        if err > self.final_center_threshold:
            self.final_center_start_t = None
            return

        now = self.now_s()
        if self.final_center_start_t is None:
            self.final_center_start_t = now
            return

        if alt <= self.land_altitude_m and (now - self.final_center_start_t) >= self.final_center_hold_sec:
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_vz = 0.0

            if self.send_land_command_enabled:
                self.request_land_async()

            self.success = True
            self.get_logger().warn("============================================================")
            self.get_logger().warn(
                f"[PHASE2] SUCCESS: precision landing condition met. "
                f"alt={alt:.2f}, err={err:.3f}, land_sent={self.land_command_sent}"
            )
            self.get_logger().warn("============================================================")

    def precision_landing_loop(self, now):
        self.publish_gimbal_downward()
        self.request_offboard_if_needed(now)

        self.update_precision_landing_control()
        self.check_precision_land_condition()
        self.publish_velocity_setpoint(self.cmd_vx, self.cmd_vy, self.cmd_vz, self.local.yaw)

        period = max(int(self.control_rate_hz / 2.0), 1)
        if self.loop_count % period == 0:
            mode = self.state.mode if self.state is not None else "UNKNOWN"
            armed = self.state.armed if self.state is not None else False
            ex, ey = self.landing_image_error()
            err = math.sqrt(ex * ex + ey * ey)
            self.get_logger().info(
                f"[PHASE2_PRECISION_LAND] mode={mode}, armed={armed}, vtol={self.vtol_state_name()}, "
                f"target={self.precision_target_fresh()}, ex={self.target.ex:+.3f}, ey={self.target.ey:+.3f}, "
                f"err={err:.3f}, alt={self.altitude_m():.2f}, "
                f"v_body_fwd={self.last_v_forward:+.2f}, v_body_right={self.last_v_right:+.2f}, "
                f"raw_v_enu=({self.raw_vx:+.2f},{self.raw_vy:+.2f},{self.raw_vz:+.2f}), "
                f"cmd_v_enu=({self.cmd_vx:+.2f},{self.cmd_vy:+.2f},{self.cmd_vz:+.2f}), "
                f"land_sent={self.land_command_sent}"
            )

    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------
    def loop(self):
        self.loop_count += 1
        now = self.now_s()

        if self.success:
            self.publish_gimbal_downward()
            self.publish_velocity_setpoint(0.0, 0.0, 0.0, self.local.yaw)
            return

        if self.precision_land_started:
            self.precision_landing_loop(now)
            return

        if now - self.start_time > self.mission_timeout_sec:
            self.get_logger().error("[PHASE2] Mission timeout")
            self.request_shutdown()
            return

        if not self.mavros_ready():
            if self.loop_count % int(max(self.control_rate_hz, 1.0)) == 0:
                connected = self.state.connected if self.state is not None else False
                self.get_logger().warn(
                    f"[PHASE2] Waiting MAVROS/local pose... connected={connected}, local_valid={self.local.valid}"
                )
            return

        # Initialize command at current vehicle pose.
        if self.cmd_x is None:
            self.cmd_x = self.local.x
            self.cmd_y = self.local.y
            self.cmd_z = self.local.z
            self.cmd_yaw = self.local.yaw

        # Before target lock, keep the original verified geometry:
        # fixed -30deg gimbal, hold XY, actively center using damped body yaw and altitude commands.
        # Only after target is centered do we estimate the ground position.
        if not self.target_locked:
            self.publish_gimbal_fixed()

            if self.active_center_before_lock:
                self.update_centering_command()
                if self.center_ref_locked:
                    self.publish_setpoint(
                        self.center_hold_x,
                        self.center_hold_y,
                        self.center_z_cmd,
                        self.center_yaw_cmd,
                    )
                    self.cmd_x = self.center_hold_x
                    self.cmd_y = self.center_hold_y
                    self.cmd_z = self.center_z_cmd
                    self.cmd_yaw = self.center_yaw_cmd
                else:
                    self.publish_setpoint(self.local.x, self.local.y, self.local.z, self.local.yaw)
            else:
                self.publish_setpoint(self.local.x, self.local.y, self.local.z, self.local.yaw)

            self.request_offboard_if_needed(now)

            ok = self.estimate_and_lock_target()
            if not ok:
                if self.loop_count % int(max(self.control_rate_hz, 1.0)) == 0:
                    mode = self.state.mode if self.state is not None else "UNKNOWN"
                    self.get_logger().warn(
                        "[PHASE2] Centering before target lock. "
                        f"mode={mode}, fresh={self.target_fresh()}, centered={self.target_centered_now()}, "
                        f"ex={self.target.ex:+.3f}, ey={self.target.ey:+.3f}, "
                        f"bearing=({self.target.bearing_x:+.3f},{self.target.bearing_y:+.3f}), "
                        f"center_cmd=({self.center_hold_x:.2f},{self.center_hold_y:.2f},{self.center_z_cmd:.2f}), "
                        f"yaw_cmd={self.center_yaw_cmd:+.3f}, vtol={self.vtol_state_name()}"
                    )
                return

            # Target locked: from now on Phase 2 looks downward and moves above it.
            self.publish_gimbal_downward()

        # Keep the gimbal pointing down after target lock.
        self.publish_gimbal_downward()

        # Smooth command toward goal.
        goal_x = float(self.goal_x)
        goal_y = float(self.goal_y)
        goal_z = float(self.goal_z)

        smooth_x = (1.0 - self.xy_alpha) * self.cmd_x + self.xy_alpha * goal_x
        smooth_y = (1.0 - self.xy_alpha) * self.cmd_y + self.xy_alpha * goal_y
        smooth_z = (1.0 - self.z_alpha) * self.cmd_z + self.z_alpha * goal_z

        self.cmd_x = self.limit_step(self.cmd_x, smooth_x, self.max_xy_step_m)
        self.cmd_y = self.limit_step(self.cmd_y, smooth_y, self.max_xy_step_m)
        self.cmd_z = self.limit_step(self.cmd_z, smooth_z, self.max_z_step_m)

        dx = goal_x - self.local.x
        dy = goal_y - self.local.y
        dz = goal_z - self.local.z
        err_xy = math.sqrt(dx * dx + dy * dy)
        err_z = abs(dz)

        # Face toward goal while moving; keep current yaw when very close.
        if err_xy > 0.3:
            self.cmd_yaw = math.atan2(dy, dx)
        else:
            self.cmd_yaw = self.local.yaw

        self.publish_setpoint(self.cmd_x, self.cmd_y, self.cmd_z, self.cmd_yaw)

        # Keep requesting OFFBOARD if PX4 drops back to AUTO.LOITER/POSCTL.
        self.request_offboard_if_needed(now)

        if err_xy <= self.arrival_xy_threshold_m and err_z <= self.arrival_z_threshold_m:
            if self.arrival_start_t is None:
                self.arrival_start_t = now
                self.get_logger().warn(
                    f"[PHASE2] Arrived threshold entered. err_xy={err_xy:.2f}, err_z={err_z:.2f}"
                )
            elif now - self.arrival_start_t >= self.arrival_hold_sec:
                self.cmd_x = goal_x
                self.cmd_y = goal_y
                self.cmd_z = goal_z

                if self.enable_precision_land_after_arrival:
                    self.start_precision_landing()
                else:
                    self.success = True
                    self.get_logger().warn("============================================================")
                    self.get_logger().warn(
                        f"[PHASE2] SUCCESS: holding above estimated survivor. "
                        f"goal=({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f}), "
                        f"err_xy={err_xy:.2f}, err_z={err_z:.2f}"
                    )
                    self.get_logger().warn("============================================================")
            return
        else:
            self.arrival_start_t = None

        period = max(int(self.control_rate_hz / 2.0), 1)
        if self.loop_count % period == 0:
            mode = self.state.mode if self.state is not None else "UNKNOWN"
            armed = self.state.armed if self.state is not None else False
            self.get_logger().info(
                f"[PHASE2] GO_ABOVE running. mode={mode}, armed={armed}, vtol={self.vtol_state_name()}, "
                f"target=({self.target_x:.2f},{self.target_y:.2f},{self.target_z:.2f}) "
                f"goal=({goal_x:.2f},{goal_y:.2f},{goal_z:.2f}) "
                f"cmd=({self.cmd_x:.2f},{self.cmd_y:.2f},{self.cmd_z:.2f}) "
                f"curr=({self.local.x:.2f},{self.local.y:.2f},{self.local.z:.2f}) "
                f"err_xy={err_xy:.2f}, err_z={err_z:.2f}, yaw={self.local.yaw:+.3f}"
            )

    def request_shutdown(self):
        if self.shutdown_requested:
            return
        self.shutdown_requested = True
        try:
            self.timer.cancel()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Phase2MavrosGoAboveSurvivor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Stopped by user")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
