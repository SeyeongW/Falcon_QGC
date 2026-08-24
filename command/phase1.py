#!/usr/bin/env python3

import math
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

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix

from common.phase1_mission import (
    MAV_VTOL_STATE_MC,
    get_mission_gps_points,
)


class Phase1OffboardToLoiter(Node):
    def __init__(self):
        super().__init__("phase1_offboard_to_loiter")

        # ============================================================
        # MAVROS STATE DATA
        # ============================================================
        self.state = None
        self.extended_state = None
        self.global_fix = None
        self.local_pose = None

        # ============================================================
        # QoS
        # ============================================================
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ============================================================
        # SUBSCRIBERS
        # ============================================================
        self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            self.qos,
        )

        self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self.extended_state_callback,
            self.qos,
        )

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.global_fix_callback,
            self.qos,
        )

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_pose_callback,
            self.qos,
        )

        # ============================================================
        # PUBLISHERS
        # ============================================================
        self.local_setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10,
        )

        # ============================================================
        # SERVICES
        # ============================================================
        self.arming_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming",
        )

        self.set_mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode",
        )

        # ============================================================
        # OFFBOARD SETPOINT STREAM
        #
        # WP1 하나만 active_setpoint로 사용한다.
        # ARM / OFFBOARD 진입 중에도 계속 20 Hz로 송신한다.
        # AUTO.LOITER가 실제 확인된 이후에만 stream을 중단한다.
        # ============================================================
        self.setpoint_rate_hz = 20.0
        self.active_setpoint = None

        self.setpoint_timer = self.create_timer(
            1.0 / self.setpoint_rate_hz,
            self.setpoint_timer_callback,
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def state_callback(self, msg):
        self.state = msg

    def extended_state_callback(self, msg):
        self.extended_state = msg

    def global_fix_callback(self, msg):
        self.global_fix = msg

    def local_pose_callback(self, msg):
        self.local_pose = msg

    # ================================================================
    # OFFBOARD SETPOINT STREAM
    # ================================================================

    def setpoint_timer_callback(self):
        if self.active_setpoint is None:
            return

        x, y, z, yaw = self.active_setpoint

        self.publish_local_setpoint(
            x,
            y,
            z,
            yaw,
        )

    def start_setpoint_stream(
        self,
        x,
        y,
        z,
        yaw,
    ):
        """
        Start continuously publishing ONE position setpoint.

        In Phase 1 this setpoint is the WP1 coordinate loaded from
        common/phase1_mission.py.

        No intermediate takeoff coordinate is generated.
        """
        self.active_setpoint = (
            float(x),
            float(y),
            float(z),
            float(yaw),
        )

        # 첫 timer tick을 기다리지 않고 즉시 한 번 송신
        self.publish_local_setpoint(
            x,
            y,
            z,
            yaw,
        )

        self.get_logger().info(
            "[PHASE 1] Continuous WP1 setpoint stream START "
            f"({self.setpoint_rate_hz:.1f} Hz)"
        )

        self.get_logger().info(
            "[PHASE 1] Active WP1 setpoint: "
            f"x={x:.2f}, "
            f"y={y:.2f}, "
            f"z={z:.2f}, "
            f"yaw={math.degrees(yaw):.1f} deg"
        )

    def stop_setpoint_stream(self):
        if self.active_setpoint is not None:
            self.get_logger().info(
                "[PHASE 1] Continuous WP1 setpoint stream STOP"
            )

        self.active_setpoint = None

    # ================================================================
    # VEHICLE STATE
    # ================================================================

    def vtol_state_name(self, value):
        names = {
            0: "UNDEFINED",
            1: "TRANSITION_TO_FW",
            2: "TRANSITION_TO_MC",
            3: "MULTICOPTER",
            4: "FIXED_WING",
        }

        return names.get(
            value,
            f"UNKNOWN({value})",
        )

    def get_current_yaw(self):
        """
        Extract current yaw from /mavros/local_position/pose quaternion.

        The current heading is frozen when Phase 1 begins so that Phase 1
        does not command an unnecessary yaw rotation while flying to WP1.
        """
        if self.local_pose is None:
            return 0.0

        q = self.local_pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    # ================================================================
    # INITIAL DATA
    # ================================================================

    def wait_initial_data(
        self,
        timeout_sec=15.0,
    ):
        self.get_logger().info(
            "[PHASE 1] Waiting for MAVROS / PX4 data"
        )

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            state_ok = (
                self.state is not None
                and self.state.connected
            )

            gps_ok = (
                self.global_fix is not None
                and self.global_fix.status.status >= 0
            )

            ext_ok = (
                self.extended_state is not None
            )

            local_ok = (
                self.local_pose is not None
            )

            if (
                state_ok
                and gps_ok
                and ext_ok
                and local_ok
            ):
                self.get_logger().info(
                    "[PHASE 1] Initial data OK"
                )

                self.get_logger().info(
                    f"[PHASE 1] mode: "
                    f"{self.state.mode}"
                )

                self.get_logger().info(
                    f"[PHASE 1] armed: "
                    f"{self.state.armed}"
                )

                self.get_logger().info(
                    f"[PHASE 1] GPS: "
                    f"lat={self.global_fix.latitude:.7f}, "
                    f"lon={self.global_fix.longitude:.7f}, "
                    f"alt={self.global_fix.altitude:.2f}"
                )

                self.get_logger().info(
                    f"[PHASE 1] LOCAL: "
                    f"x={self.local_pose.pose.position.x:.2f}, "
                    f"y={self.local_pose.pose.position.y:.2f}, "
                    f"z={self.local_pose.pose.position.z:.2f}"
                )

                current_yaw = self.get_current_yaw()

                self.get_logger().info(
                    f"[PHASE 1] Current yaw: "
                    f"{math.degrees(current_yaw):.1f} deg"
                )

                self.get_logger().info(
                    f"[PHASE 1] VTOL state: "
                    f"{self.vtol_state_name(self.extended_state.vtol_state)}"
                )

                self.get_logger().info(
                    f"[PHASE 1] landed_state: "
                    f"{self.extended_state.landed_state}"
                )

                return True

            if time.time() - start > timeout_sec:
                self.get_logger().error(
                    "[PHASE 1] Initial data timeout"
                )

                self.get_logger().error(
                    f"state_ok: {state_ok}"
                )

                self.get_logger().error(
                    f"gps_ok: {gps_ok}"
                )

                self.get_logger().error(
                    f"extended_state_ok: {ext_ok}"
                )

                self.get_logger().error(
                    f"local_pose_ok: {local_ok}"
                )

                return False

        return False

    # ================================================================
    # SERVICE HELPERS
    # ================================================================

    def wait_service(
        self,
        client,
        name,
        timeout_sec=5.0,
    ):
        start = time.time()

        while rclpy.ok():

            if client.wait_for_service(
                timeout_sec=0.0
            ):
                return True

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            if time.time() - start > timeout_sec:
                self.get_logger().error(
                    f"[PHASE 1] Service unavailable: "
                    f"{name}"
                )
                return False

        return False

    def call_service(
        self,
        client,
        request,
        name,
        timeout_sec=10.0,
    ):
        """
        Service 호출 중에도 spin_once()를 계속 수행한다.

        따라서 OFFBOARD setpoint timer 역시 계속 살아 있다.
        """
        future = client.call_async(
            request
        )

        start = time.time()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            if future.done():

                try:
                    result = future.result()

                except Exception as exc:

                    self.get_logger().error(
                        f"[PHASE 1] {name} "
                        f"service exception: {exc}"
                    )

                    return None

                self.get_logger().info(
                    f"[PHASE 1] {name} result: "
                    f"{result}"
                )

                return result

            if time.time() - start > timeout_sec:

                self.get_logger().error(
                    f"[PHASE 1] {name} timeout"
                )

                return None

        return None

    # ================================================================
    # MODE
    # ================================================================

    def set_mode(
        self,
        mode_name,
    ):
        self.get_logger().info(
            f"[PHASE 1] Request mode: "
            f"{mode_name}"
        )

        if not self.wait_service(
            self.set_mode_client,
            "/mavros/set_mode",
        ):
            return False

        req = SetMode.Request()

        req.base_mode = 0
        req.custom_mode = mode_name

        result = self.call_service(
            self.set_mode_client,
            req,
            f"set_mode {mode_name}",
        )

        if (
            result is not None
            and result.mode_sent
        ):
            self.get_logger().info(
                f"[PHASE 1] Mode request sent: "
                f"{mode_name}"
            )

            return True

        self.get_logger().error(
            f"[PHASE 1] Mode request failed: "
            f"{mode_name}"
        )

        return False

    def wait_mode(
        self,
        mode_name,
        timeout_sec=20.0,
    ):
        self.get_logger().info(
            f"[PHASE 1] Waiting until mode = "
            f"{mode_name}"
        )

        start = time.time()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            current = (
                self.state.mode
                if self.state
                else "UNKNOWN"
            )

            if current == mode_name:

                self.get_logger().info(
                    f"[PHASE 1] Current mode = "
                    f"{mode_name}"
                )

                return True

            if time.time() - start > timeout_sec:

                self.get_logger().error(
                    "[PHASE 1] Mode wait timeout. "
                    f"current={current}, "
                    f"target={mode_name}"
                )

                return False

        return False

    # ================================================================
    # ARM
    # ================================================================

    def arm(self):

        if (
            self.state is not None
            and self.state.armed
        ):
            self.get_logger().info(
                "[PHASE 1] Vehicle already armed"
            )

            return True

        self.get_logger().info(
            "[PHASE 1] Request ARM"
        )

        if not self.wait_service(
            self.arming_client,
            "/mavros/cmd/arming",
        ):
            return False

        req = CommandBool.Request()
        req.value = True

        result = self.call_service(
            self.arming_client,
            req,
            "arm",
        )

        if (
            result is not None
            and result.success
        ):
            self.get_logger().info(
                "[PHASE 1] ARM accepted"
            )

            return True

        self.get_logger().error(
            "[PHASE 1] ARM failed"
        )

        return False

    # ================================================================
    # GPS -> LOCAL ENU
    # ================================================================

    def gps_to_local_enu(
        self,
        ref_lat,
        ref_lon,
        target_lat,
        target_lon,
    ):
        """
        Convert the GPS displacement from the current GPS reference
        to local ENU displacement.

        Returns:
            east [m], north [m]
        """
        earth_radius_m = 6378137.0

        d_lat = math.radians(
            target_lat - ref_lat
        )

        d_lon = math.radians(
            target_lon - ref_lon
        )

        north = (
            d_lat
            * earth_radius_m
        )

        east = (
            d_lon
            * earth_radius_m
            * math.cos(
                math.radians(ref_lat)
            )
        )

        return east, north

    # ================================================================
    # LOCAL POSITION SETPOINT
    # ================================================================

    def make_local_pose_setpoint(
        self,
        x,
        y,
        z,
        yaw,
    ):
        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = "map"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0

        msg.pose.orientation.z = math.sin(
            yaw * 0.5
        )

        msg.pose.orientation.w = math.cos(
            yaw * 0.5
        )

        return msg

    def publish_local_setpoint(
        self,
        x,
        y,
        z,
        yaw,
    ):
        msg = self.make_local_pose_setpoint(
            x,
            y,
            z,
            yaw,
        )

        self.local_setpoint_pub.publish(
            msg
        )

    # ================================================================
    # FORCE POSCTL BEFORE ARM
    # ================================================================

    def force_posctl_before_arm(self):

        current = (
            self.state.mode
            if self.state
            else "UNKNOWN"
        )

        if current == "POSCTL":

            self.get_logger().info(
                "[PHASE 1] Mode already POSCTL before ARM"
            )

            return True

        self.get_logger().info(
            f"[PHASE 1] Current mode is {current}. "
            "Switching to POSCTL before ARM."
        )

        if not self.set_mode(
            "POSCTL"
        ):
            return False

        if not self.wait_mode(
            "POSCTL",
            timeout_sec=10.0,
        ):
            return False

        return True

    # ================================================================
    # ENTER OFFBOARD
    # ================================================================

    def enter_offboard_with_continuous_setpoint(
        self,
        target_x,
        target_y,
        target_z,
        target_yaw,
        prestream_sec=2.0,
        arm_timeout_sec=15.0,
        offboard_timeout_sec=15.0,
    ):
        """
        Start streaming the FINAL WP1 setpoint before ARM/OFFBOARD.

        IMPORTANT:
        There is no temporary vertical-takeoff setpoint.

        From the first OFFBOARD position command until WP1 arrival,
        the active position target remains exactly the same WP1.
        """

        # ============================================================
        # WP1 SETPOINT STREAM START
        # ============================================================

        self.start_setpoint_stream(
            target_x,
            target_y,
            target_z,
            target_yaw,
        )

        self.get_logger().info(
            "[PHASE 1] Pre-streaming FINAL WP1 setpoint "
            "before ARM / OFFBOARD"
        )

        start = time.time()

        while (
            rclpy.ok()
            and time.time() - start < prestream_sec
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

        # ============================================================
        # ARM
        # ============================================================

        if not self.arm():
            return False

        self.get_logger().info(
            "[PHASE 1] Waiting armed while "
            "continuing FINAL WP1 setpoint stream"
        )

        arm_start = time.time()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            if (
                self.state is not None
                and self.state.armed
            ):
                self.get_logger().info(
                    "[PHASE 1] Vehicle armed"
                )

                break

            if (
                time.time() - arm_start
                > arm_timeout_sec
            ):
                self.get_logger().error(
                    "[PHASE 1] Arm wait timeout"
                )

                return False

        # ============================================================
        # OFFBOARD
        # ============================================================

        self.get_logger().info(
            "[PHASE 1] Requesting OFFBOARD while "
            "streaming FINAL WP1 setpoint"
        )

        offboard_start = time.time()
        last_request = 0.0

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            now = time.time()

            current_mode = (
                self.state.mode
                if self.state
                else "UNKNOWN"
            )

            if current_mode == "OFFBOARD":

                self.get_logger().info(
                    "[PHASE 1] Current mode = OFFBOARD"
                )

                return True

            if (
                now - last_request
                > 1.0
            ):
                self.set_mode(
                    "OFFBOARD"
                )

                last_request = time.time()

            if (
                time.time() - offboard_start
                > offboard_timeout_sec
            ):
                self.get_logger().error(
                    "[PHASE 1] OFFBOARD mode not entered. "
                    f"current={current_mode}"
                )

                return False

        return False

    # ================================================================
    # OFFBOARD -> WP1
    # ================================================================

    def offboard_go_wp1_and_hold(
        self,
        wp1_lat,
        wp1_lon,
        wp1_alt_m,
        xy_threshold_m=2.0,
        z_threshold_m=1.0,
        wp1_hold_time_sec=3.0,
        timeout_sec=180.0,
    ):
        """
        Phase 1:

        1. Read WP1 from common/phase1_mission.py.
        2. Convert WP1 latitude / longitude to local ENU.
        3. Freeze current yaw.
        4. Stream FINAL WP1 as the only OFFBOARD position setpoint.
        5. ARM.
        6. Enter OFFBOARD.
        7. Fly directly toward WP1.
        8. When WP1 is reached, keep EXACTLY the same WP1 setpoint
           for wp1_hold_time_sec.
        9. Return True while the WP1 stream remains active.
       10. Caller changes OFFBOARD -> AUTO.LOITER.

        IMPORTANT:
        No temporary takeoff coordinate is created.
        No "current XY + WP1 altitude" target is created.
        No second position setpoint is generated.
        """

        # ============================================================
        # CURRENT REFERENCE
        # ============================================================

        home_lat = float(
            self.global_fix.latitude
        )

        home_lon = float(
            self.global_fix.longitude
        )

        current_local = (
            self.local_pose.pose.position
        )

        start_x = float(
            current_local.x
        )

        start_y = float(
            current_local.y
        )

        start_z = float(
            current_local.z
        )

        # ============================================================
        # WP1 GPS -> LOCAL ENU
        # ============================================================

        target_east, target_north = (
            self.gps_to_local_enu(
                ref_lat=home_lat,
                ref_lon=home_lon,
                target_lat=wp1_lat,
                target_lon=wp1_lon,
            )
        )

        target_x = (
            start_x
            + target_east
        )

        target_y = (
            start_y
            + target_north
        )

        target_z = float(
            wp1_alt_m
        )

        # ============================================================
        # CURRENT HEADING HOLD
        #
        # 기존 yaw=0.0 강제 명령 제거.
        # 별도의 yaw 행동 없이 현재 heading 유지.
        # ============================================================

        target_yaw = (
            self.get_current_yaw()
        )

        self.get_logger().info(
            "[PHASE 1] =================================================="
        )

        self.get_logger().info(
            "[PHASE 1] FINAL WP1 loaded from phase1_mission.py"
        )

        self.get_logger().info(
            f"[PHASE 1] WP1 GPS: "
            f"lat={wp1_lat:.7f}, "
            f"lon={wp1_lon:.7f}, "
            f"alt={wp1_alt_m:.2f}"
        )

        self.get_logger().info(
            f"[PHASE 1] Current local: "
            f"x={start_x:.2f}, "
            f"y={start_y:.2f}, "
            f"z={start_z:.2f}"
        )

        self.get_logger().info(
            f"[PHASE 1] FINAL WP1 local target: "
            f"x={target_x:.2f}, "
            f"y={target_y:.2f}, "
            f"z={target_z:.2f}"
        )

        self.get_logger().info(
            f"[PHASE 1] Heading hold: "
            f"{math.degrees(target_yaw):.1f} deg"
        )

        initial_xy_distance = math.hypot(
            target_x - start_x,
            target_y - start_y,
        )

        initial_z_distance = abs(
            target_z - start_z
        )

        self.get_logger().info(
            f"[PHASE 1] Initial WP1 error: "
            f"XY={initial_xy_distance:.2f} m, "
            f"Z={initial_z_distance:.2f} m"
        )

        self.get_logger().info(
            "[PHASE 1] =================================================="
        )

        # ============================================================
        # ARM -> OFFBOARD
        #
        # FINAL WP1 is already the active setpoint.
        # ============================================================

        if not self.enter_offboard_with_continuous_setpoint(
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw,
            prestream_sec=2.0,
            arm_timeout_sec=15.0,
            offboard_timeout_sec=15.0,
        ):
            return False

        # ============================================================
        # DIRECTLY FOLLOW WP1
        # ============================================================

        self.get_logger().info(
            "[PHASE 1] Direct WP1 tracking START"
        )

        self.get_logger().info(
            "[PHASE 1] No intermediate position setpoint will be used"
        )

        mission_start = time.time()
        hold_start = None
        last_print = 0.0

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            now = time.time()

            if self.local_pose is None:
                continue

            pos = (
                self.local_pose.pose.position
            )

            # ========================================================
            # ERROR TO FINAL WP1
            # ========================================================

            ex = (
                target_x
                - pos.x
            )

            ey = (
                target_y
                - pos.y
            )

            ez = (
                target_z
                - pos.z
            )

            err_xy = math.hypot(
                ex,
                ey,
            )

            err_z = abs(
                ez
            )

            current_mode = (
                self.state.mode
                if self.state
                else "UNKNOWN"
            )

            # ========================================================
            # STATUS LOG
            # ========================================================

            if (
                now - last_print
                > 1.0
            ):
                self.get_logger().info(
                    f"[PHASE 1] WP1 TRACKING: "
                    f"mode={current_mode}, "
                    f"err_xy={err_xy:.2f}, "
                    f"err_z={err_z:.2f}, "
                    f"pos=({pos.x:.2f},"
                    f"{pos.y:.2f},"
                    f"{pos.z:.2f}), "
                    f"WP1=({target_x:.2f},"
                    f"{target_y:.2f},"
                    f"{target_z:.2f})"
                )

                last_print = now

            # ========================================================
            # OFFBOARD SAFETY CHECK
            # ========================================================

            if current_mode != "OFFBOARD":

                self.get_logger().error(
                    "[PHASE 1] OFFBOARD lost while tracking WP1. "
                    f"current={current_mode}"
                )

                return False

            # ========================================================
            # WP1 ARRIVAL / HOLD
            #
            # IMPORTANT:
            # active_setpoint is NEVER changed.
            #
            # Even after entering this threshold, the exact same
            # WP1 position continues to be sent at 20 Hz.
            # ========================================================

            if (
                err_xy <= xy_threshold_m
                and err_z <= z_threshold_m
            ):

                if hold_start is None:

                    hold_start = now

                    self.get_logger().info(
                        "[PHASE 1] WP1 reached. "
                        "Keeping SAME WP1 setpoint..."
                    )

                elif (
                    now - hold_start
                    >= wp1_hold_time_sec
                ):

                    self.get_logger().info(
                        "[PHASE 1] WP1 hold complete"
                    )

                    self.get_logger().info(
                        "[PHASE 1] WP1 setpoint stream remains active "
                        "until AUTO.LOITER is confirmed"
                    )

                    return True

            else:
                # Threshold 밖으로 다시 나가면 hold timer 재시작
                hold_start = None

            # ========================================================
            # TIMEOUT
            # ========================================================

            if (
                now - mission_start
                > timeout_sec
            ):
                self.get_logger().error(
                    "[PHASE 1] WP1 tracking timeout. "
                    f"err_xy={err_xy:.2f}, "
                    f"err_z={err_z:.2f}"
                )

                return False

        return False

    # ================================================================
    # OFFBOARD -> AUTO.LOITER
    # ================================================================

    def switch_offboard_to_loiter_continuous(
        self,
        timeout_sec=15.0,
        request_interval_sec=1.0,
    ):
        """
        Switch OFFBOARD -> AUTO.LOITER.

        IMPORTANT:
        The FINAL WP1 setpoint continues to be transmitted until
        PX4 /mavros/state positively reports AUTO.LOITER.

        Only after AUTO.LOITER confirmation is the OFFBOARD stream stopped.
        """

        if self.active_setpoint is None:

            self.get_logger().error(
                "[PHASE 1] Cannot switch safely to AUTO.LOITER: "
                "no active WP1 setpoint stream"
            )

            return False

        self.get_logger().info(
            "[PHASE 1] Switching OFFBOARD -> AUTO.LOITER "
            "while KEEPING FINAL WP1 setpoint stream active"
        )

        start = time.time()
        last_request = 0.0

        while rclpy.ok():

            # subscription + setpoint timer 유지
            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            now = time.time()

            current_mode = (
                self.state.mode
                if self.state
                else "UNKNOWN"
            )

            # ========================================================
            # AUTO.LOITER CONFIRMED
            # ========================================================

            if current_mode == "AUTO.LOITER":

                self.get_logger().info(
                    "[PHASE 1] AUTO.LOITER confirmed"
                )

                # 이제 OFFBOARD setpoint stream 제거 가능
                self.stop_setpoint_stream()

                return True

            # ========================================================
            # MODE REQUEST
            # ========================================================

            if (
                now - last_request
                >= request_interval_sec
            ):

                self.set_mode(
                    "AUTO.LOITER"
                )

                last_request = time.time()

            # ========================================================
            # TIMEOUT
            # ========================================================

            if (
                time.time() - start
                > timeout_sec
            ):

                self.get_logger().error(
                    "[PHASE 1] AUTO.LOITER transition timeout. "
                    f"current={current_mode}"
                )

                # LOITER 확인 실패 시 WP1 stream을 의도적으로 유지
                return False

        return False

    # ================================================================
    # MAIN PHASE
    # ================================================================

    def run(self):

        self.get_logger().info(
            "[PHASE 1] "
            "ARM -> OFFBOARD -> DIRECT WP1 -> HOLD WP1 -> AUTO.LOITER"
        )

        # ============================================================
        # INITIAL DATA
        # ============================================================

        if not self.wait_initial_data():
            return False

        # ============================================================
        # START CONDITIONS
        # ============================================================

        if self.state.armed:

            self.get_logger().error(
                "[PHASE 1] Vehicle is already armed. "
                "Start from DISARMED."
            )

            return False

        if (
            self.extended_state.vtol_state
            != MAV_VTOL_STATE_MC
        ):

            self.get_logger().error(
                "[PHASE 1] Vehicle must start in MULTICOPTER. "
                f"Current: "
                f"{self.vtol_state_name(self.extended_state.vtol_state)}"
            )

            return False

        if (
            self.extended_state.landed_state
            != 1
        ):

            self.get_logger().error(
                "[PHASE 1] Vehicle must start ON_GROUND."
            )

            return False

        # ============================================================
        # POSCTL
        # ============================================================

        if not self.force_posctl_before_arm():
            return False

        # ============================================================
        # LOAD FINAL WP1
        #
        # Mission upload / clear / push / pull is intentionally
        # NOT performed here.
        #
        # WP1 latitude / longitude / altitude are defined only in
        # common/phase1_mission.py.
        # ============================================================

        points = (
            get_mission_gps_points()
        )

        wp1_lat, wp1_lon, wp1_alt = (
            points["wp1"]
        )

        self.get_logger().info(
            "[PHASE 1] Loaded FINAL WP1 from phase1_mission.py: "
            f"lat={wp1_lat:.7f}, "
            f"lon={wp1_lon:.7f}, "
            f"alt={wp1_alt:.1f}"
        )

        # ============================================================
        # DIRECT WP1
        # ============================================================

        if not self.offboard_go_wp1_and_hold(
            wp1_lat=wp1_lat,
            wp1_lon=wp1_lon,
            wp1_alt_m=wp1_alt,
            xy_threshold_m=2.0,
            z_threshold_m=1.0,
            wp1_hold_time_sec=3.0,
            timeout_sec=180.0,
        ):
            return False

        # ============================================================
        # WP1 -> AUTO.LOITER
        # ============================================================

        self.get_logger().info(
            "[PHASE 1] WP1 reached and held. "
            "Requesting AUTO.LOITER with continuous WP1 setpoint."
        )

        if not self.switch_offboard_to_loiter_continuous(
            timeout_sec=15.0,
            request_interval_sec=1.0,
        ):
            return False

        # ============================================================
        # SUCCESS
        # ============================================================

        self.get_logger().info(
            "[PHASE 1] SUCCESS: "
            "vehicle reached WP1 and is now in AUTO.LOITER"
        )

        return True


# ====================================================================
# MAIN
# ====================================================================

def main():
    rclpy.init()

    node = Phase1OffboardToLoiter()

    try:
        ok = node.run()

    except KeyboardInterrupt:

        node.get_logger().warn(
            "[PHASE 1] Interrupted by user"
        )

        ok = False

    finally:

        # AUTO.LOITER가 확인된 정상 상황에서는 이미 inactive.
        #
        # OFFBOARD 중 실패한 경우 node shutdown으로 stream이 종료되며,
        # 이후 PX4에 설정된 OFFBOARD loss failsafe가 적용된다.
        node.stop_setpoint_stream()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
