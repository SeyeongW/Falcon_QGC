#!/usr/bin/env python3
"""
phase3_auto_mission_vtol_land.py

Phase 3:
- MAVROS AUTO.MISSION 기반 VTOL 임무.
- Phase 1과 같은 구조로 이륙 후 고정익 전환.
- 고정익으로 waypoint 5개를 비행.
- 마지막 waypoint 이후 멀티콥터로 복귀.
- VTOL_LAND / AUTO.LAND로 착륙 완료.
- 착륙 확인 후 PHASE 3 SUCCESS 출력.

Expected start state:
- MAVROS connected
- Vehicle disarmed
- VTOL state MULTICOPTER
- landed_state ON_GROUND
- GPS/global position valid
"""

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

from mavros_msgs.msg import State, ExtendedState, WaypointReached, Waypoint
from mavros_msgs.srv import (
    CommandBool,
    CommandLong,
    SetMode,
    WaypointClear,
    WaypointPush,
    WaypointSetCurrent,
)
from sensor_msgs.msg import NavSatFix


# ---------------------------------------------------------------------
# MAVLink mission constants
# ---------------------------------------------------------------------
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_VTOL_TAKEOFF = 84
MAV_CMD_NAV_VTOL_LAND = 85
MAV_CMD_DO_VTOL_TRANSITION = 3000

MAV_VTOL_STATE_MC = 3
MAV_VTOL_STATE_FW = 4

MAV_FRAME_GLOBAL_REL_ALT = 3
MAV_FRAME_MISSION = 2


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    earth_radius_m = 6378137.0

    d_lat = north_m / earth_radius_m
    d_lon = east_m / (earth_radius_m * math.cos(math.radians(lat_deg)))

    new_lat = lat_deg + math.degrees(d_lat)
    new_lon = lon_deg + math.degrees(d_lon)

    return new_lat, new_lon


def make_wp(
    command,
    lat,
    lon,
    alt,
    frame=MAV_FRAME_GLOBAL_REL_ALT,
    is_current=False,
    autocontinue=True,
    param1=0.0,
    param2=0.0,
    param3=0.0,
    param4=0.0,
):
    wp = Waypoint()

    wp.frame = int(frame)
    wp.command = int(command)
    wp.is_current = bool(is_current)
    wp.autocontinue = bool(autocontinue)

    wp.param1 = float(param1)
    wp.param2 = float(param2)
    wp.param3 = float(param3)
    wp.param4 = float(param4)

    wp.x_lat = float(lat)
    wp.y_long = float(lon)
    wp.z_alt = float(alt)

    return wp


def build_pentagon_offsets(center_north_m, center_east_m, side_length_m=200.0):
    """
    정오각형 waypoint offset 생성.
    반환: [(north_m, east_m), ...] 5개
    """
    radius_m = side_length_m / (2.0 * math.sin(math.pi / 5.0))
    points = []

    start_angle_rad = math.radians(90.0)

    for i in range(5):
        angle = start_angle_rad - i * 2.0 * math.pi / 5.0
        north = center_north_m + radius_m * math.sin(angle)
        east = center_east_m + radius_m * math.cos(angle)
        points.append((north, east))

    return points


def build_phase3_vtol_patrol_land_mission(
    home_lat,
    home_lon,
    mission_alt_m=30.0,
    takeoff_north_m=30.0,
    pentagon_center_north_m=350.0,
    pentagon_center_east_m=0.0,
    pentagon_side_m=200.0,
):
    """
    Phase 3 AUTO.MISSION 생성.

    Mission sequence:
    0. VTOL_TAKEOFF
    1. DO_VTOL_TRANSITION to FIXED_WING
    2. Pentagon waypoint 1
    3. Pentagon waypoint 2
    4. Pentagon waypoint 3
    5. Pentagon waypoint 4
    6. Pentagon waypoint 5
    7. DO_VTOL_TRANSITION to MULTICOPTER
    8. VTOL_LAND

    Phase 1과 달리 Phase 3는 마지막에 실제 착륙까지 수행한다.
    """

    takeoff_lat, takeoff_lon = offset_latlon(
        home_lat,
        home_lon,
        north_m=takeoff_north_m,
        east_m=0.0,
    )

    pentagon_offsets = build_pentagon_offsets(
        center_north_m=pentagon_center_north_m,
        center_east_m=pentagon_center_east_m,
        side_length_m=pentagon_side_m,
    )

    pentagon_latlon = [
        offset_latlon(
            home_lat,
            home_lon,
            north_m=north_m,
            east_m=east_m,
        )
        for north_m, east_m in pentagon_offsets
    ]

    last_lat, last_lon = pentagon_latlon[-1]

    waypoints = []

    # 0. VTOL 이륙
    waypoints.append(
        make_wp(
            command=MAV_CMD_NAV_VTOL_TAKEOFF,
            lat=takeoff_lat,
            lon=takeoff_lon,
            alt=mission_alt_m,
            frame=MAV_FRAME_GLOBAL_REL_ALT,
            is_current=True,
        )
    )

    # 1. 고정익 천이
    waypoints.append(
        make_wp(
            command=MAV_CMD_DO_VTOL_TRANSITION,
            lat=takeoff_lat,
            lon=takeoff_lon,
            alt=mission_alt_m,
            frame=MAV_FRAME_MISSION,
            param1=MAV_VTOL_STATE_FW,
        )
    )

    # 2~6. 고정익 waypoint 5개
    for lat, lon in pentagon_latlon:
        waypoints.append(
            make_wp(
                command=MAV_CMD_NAV_WAYPOINT,
                lat=lat,
                lon=lon,
                alt=mission_alt_m,
                frame=MAV_FRAME_GLOBAL_REL_ALT,
                param2=35.0,  # acceptance radius
            )
        )

    # 7. 멀티콥터 역천이
    waypoints.append(
        make_wp(
            command=MAV_CMD_DO_VTOL_TRANSITION,
            lat=last_lat,
            lon=last_lon,
            alt=mission_alt_m,
            frame=MAV_FRAME_MISSION,
            param1=MAV_VTOL_STATE_MC,
        )
    )

    # 8. VTOL 착륙
    waypoints.append(
        make_wp(
            command=MAV_CMD_NAV_VTOL_LAND,
            lat=last_lat,
            lon=last_lon,
            alt=0.0,
            frame=MAV_FRAME_GLOBAL_REL_ALT,
        )
    )

    mission_info = {
        "start": (home_lat, home_lon),
        "takeoff": (takeoff_lat, takeoff_lon),
        "fw_wps": pentagon_latlon,
        "mc_wp": (last_lat, last_lon),
        "land": (last_lat, last_lon),
        "alt_m": mission_alt_m,
        "pentagon_side_m": pentagon_side_m,
        "count": len(waypoints),
        "final_fw_seq": 6,
        "mc_transition_seq": 7,
        "land_seq": 8,
    }

    return waypoints, mission_info


class Phase3AutoMissionVTOLLand(Node):
    def __init__(self):
        super().__init__("phase3_auto_mission_vtol_land")

        self.state = None
        self.extended_state = None
        self.global_fix = None
        self.last_reached_seq = -1

        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(State, "/mavros/state", self.state_callback, self.qos)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self.extended_state_callback, self.qos)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.global_fix_callback, self.qos)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self.waypoint_reached_callback, 10)

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.command_long_client = self.create_client(CommandLong, "/mavros/cmd/command")

        self.mission_clear_client = self.create_client(WaypointClear, "/mavros/mission/clear")
        self.mission_push_client = self.create_client(WaypointPush, "/mavros/mission/push")
        self.mission_set_current_client = self.create_client(WaypointSetCurrent, "/mavros/mission/set_current")

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------
    def state_callback(self, msg):
        self.state = msg

    def extended_state_callback(self, msg):
        self.extended_state = msg

    def global_fix_callback(self, msg):
        self.global_fix = msg

    def waypoint_reached_callback(self, msg):
        self.last_reached_seq = msg.wp_seq
        self.get_logger().info(f"[PHASE 3] Mission reached seq: {msg.wp_seq}")

    # -----------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------
    def vtol_state_name(self, value):
        names = {
            0: "UNDEFINED",
            1: "TRANSITION_TO_FW",
            2: "TRANSITION_TO_MC",
            3: "MULTICOPTER",
            4: "FIXED_WING",
        }
        return names.get(value, f"UNKNOWN({value})")

    def landed_state_name(self, value):
        names = {
            0: "UNDEFINED",
            1: "ON_GROUND",
            2: "IN_AIR",
            3: "TAKEOFF",
            4: "LANDING",
        }
        return names.get(value, f"UNKNOWN({value})")

    def wait_initial_data(self, timeout_sec=15.0):
        self.get_logger().info("[PHASE 3] Waiting for MAVROS / PX4 data")

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            state_ok = self.state is not None and self.state.connected
            gps_ok = self.global_fix is not None and self.global_fix.status.status >= 0
            ext_ok = self.extended_state is not None

            if state_ok and gps_ok and ext_ok:
                self.get_logger().info("[PHASE 3] Initial data OK")
                self.get_logger().info(f"[PHASE 3] mode: {self.state.mode}")
                self.get_logger().info(f"[PHASE 3] armed: {self.state.armed}")
                self.get_logger().info(
                    f"[PHASE 3] GPS: lat={self.global_fix.latitude:.7f}, "
                    f"lon={self.global_fix.longitude:.7f}, alt={self.global_fix.altitude:.2f}"
                )
                self.get_logger().info(
                    f"[PHASE 3] VTOL state: {self.vtol_state_name(self.extended_state.vtol_state)}"
                )
                self.get_logger().info(
                    f"[PHASE 3] landed_state: {self.landed_state_name(self.extended_state.landed_state)}"
                )
                return True

            if time.time() - start > timeout_sec:
                self.get_logger().error("[PHASE 3] Initial data timeout")
                self.get_logger().error(f"state_ok: {state_ok}")
                self.get_logger().error(f"gps_ok: {gps_ok}")
                self.get_logger().error(f"extended_state_ok: {ext_ok}")
                return False

        return False

    def wait_service(self, client, name, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"[PHASE 3] Service unavailable: {name}")
            return False
        return True

    def call_service(self, client, request, name, timeout_sec=10.0):
        future = client.call_async(request)
        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if future.done():
                result = future.result()
                self.get_logger().info(f"[PHASE 3] {name} result: {result}")
                return result

            if time.time() - start > timeout_sec:
                self.get_logger().error(f"[PHASE 3] {name} timeout")
                return None

        return None

    def set_mode(self, mode_name):
        self.get_logger().info(f"[PHASE 3] Request mode: {mode_name}")

        if not self.wait_service(self.set_mode_client, "/mavros/set_mode"):
            return False

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_name

        result = self.call_service(self.set_mode_client, req, f"set_mode {mode_name}")

        if result is not None and result.mode_sent:
            self.get_logger().info(f"[PHASE 3] Mode request sent: {mode_name}")
            return True

        self.get_logger().error(f"[PHASE 3] Mode request failed: {mode_name}")
        return False

    def wait_mode(self, mode_name, timeout_sec=20.0):
        self.get_logger().info(f"[PHASE 3] Waiting until mode = {mode_name}")

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            current = self.state.mode if self.state else "UNKNOWN"

            if current == mode_name:
                self.get_logger().info(f"[PHASE 3] Current mode = {mode_name}")
                return True

            if time.time() - start > timeout_sec:
                self.get_logger().error(
                    f"[PHASE 3] Mode wait timeout. current={current}, target={mode_name}"
                )
                return False

        return False

    def arm(self):
        self.get_logger().info("[PHASE 3] Request ARM")

        if not self.wait_service(self.arming_client, "/mavros/cmd/arming"):
            return False

        req = CommandBool.Request()
        req.value = True

        result = self.call_service(self.arming_client, req, "arm")

        if result is not None and result.success:
            self.get_logger().info("[PHASE 3] ARM accepted")
            return True

        self.get_logger().error("[PHASE 3] ARM failed")
        return False

    def disarm(self):
        self.get_logger().info("[PHASE 3] Request DISARM")

        if not self.wait_service(self.arming_client, "/mavros/cmd/arming"):
            return False

        req = CommandBool.Request()
        req.value = False

        result = self.call_service(self.arming_client, req, "disarm")

        if result is not None and result.success:
            self.get_logger().info("[PHASE 3] DISARM accepted")
            return True

        self.get_logger().warn("[PHASE 3] DISARM request failed or already disarmed")
        return False

    def wait_armed(self, armed=True, timeout_sec=15.0):
        label = "armed" if armed else "disarmed"
        self.get_logger().info(f"[PHASE 3] Waiting until {label}")

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.state is not None and self.state.armed == armed:
                self.get_logger().info(f"[PHASE 3] Vehicle {label}")
                return True

            if time.time() - start > timeout_sec:
                self.get_logger().error(f"[PHASE 3] {label} wait timeout")
                return False

        return False

    def clear_mission(self):
        self.get_logger().info("[PHASE 3] Clear existing mission")

        if not self.wait_service(self.mission_clear_client, "/mavros/mission/clear"):
            return False

        req = WaypointClear.Request()
        result = self.call_service(self.mission_clear_client, req, "mission_clear")

        if result is not None and result.success:
            self.get_logger().info("[PHASE 3] Mission clear success")
            return True

        self.get_logger().error("[PHASE 3] Mission clear failed")
        return False

    def set_current_mission(self, seq=0):
        self.get_logger().info(f"[PHASE 3] Set current mission seq: {seq}")

        if not self.wait_service(self.mission_set_current_client, "/mavros/mission/set_current"):
            return False

        req = WaypointSetCurrent.Request()
        req.wp_seq = int(seq)

        result = self.call_service(self.mission_set_current_client, req, "mission_set_current")

        if result is not None and result.success:
            self.get_logger().info("[PHASE 3] Set current mission success")
            return True

        self.get_logger().error("[PHASE 3] Set current mission failed")
        return False

    def push_mission(self, waypoints):
        self.get_logger().info("[PHASE 3] Push VTOL patrol + land mission to PX4")

        if not self.wait_service(self.mission_push_client, "/mavros/mission/push"):
            return False

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints

        result = self.call_service(self.mission_push_client, req, "mission_push", timeout_sec=15.0)

        if result is not None and result.success:
            self.get_logger().info(
                f"[PHASE 3] Mission push success. transferred={result.wp_transfered}"
            )
            return True

        self.get_logger().error("[PHASE 3] Mission push failed")
        return False

    def request_vtol_transition_to_mc(self):
        self.get_logger().info("[PHASE 3] Request VTOL transition to MULTICOPTER")

        if not self.wait_service(self.command_long_client, "/mavros/cmd/command"):
            return False

        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_DO_VTOL_TRANSITION
        req.confirmation = 0
        req.param1 = float(MAV_VTOL_STATE_MC)
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        result = self.call_service(
            self.command_long_client,
            req,
            "VTOL transition to MC",
            timeout_sec=10.0,
        )

        if result is not None and result.success:
            self.get_logger().info("[PHASE 3] VTOL transition to MC accepted")
            return True

        self.get_logger().error("[PHASE 3] VTOL transition to MC failed")
        return False

    def log_mission_info(self, mission_info):
        self.get_logger().info("[PHASE 3] Built AUTO.MISSION VTOL patrol + landing sequence")
        self.get_logger().info(
            f"[PHASE 3] start:   {mission_info['start'][0]:.7f}, {mission_info['start'][1]:.7f}"
        )
        self.get_logger().info(
            f"[PHASE 3] takeoff: {mission_info['takeoff'][0]:.7f}, {mission_info['takeoff'][1]:.7f}"
        )

        for idx, (lat, lon) in enumerate(mission_info["fw_wps"], start=1):
            self.get_logger().info(f"[PHASE 3] fw_wp{idx}:  {lat:.7f}, {lon:.7f}")

        self.get_logger().info(
            f"[PHASE 3] mc_wp:   {mission_info['mc_wp'][0]:.7f}, {mission_info['mc_wp'][1]:.7f}"
        )
        self.get_logger().info(
            f"[PHASE 3] land:    {mission_info['land'][0]:.7f}, {mission_info['land'][1]:.7f}"
        )
        self.get_logger().info(f"[PHASE 3] mission alt: {mission_info['alt_m']:.1f} m")
        self.get_logger().info(f"[PHASE 3] pentagon side: {mission_info['pentagon_side_m']:.1f} m")
        self.get_logger().info(f"[PHASE 3] item count: {mission_info['count']}")
        self.get_logger().info(
            f"[PHASE 3] final_fw_seq={mission_info['final_fw_seq']}, "
            f"mc_transition_seq={mission_info['mc_transition_seq']}, land_seq={mission_info['land_seq']}"
        )

    # -----------------------------------------------------------------
    # Mission monitors
    # -----------------------------------------------------------------
    def monitor_until_mc_transition_complete(
        self,
        final_fw_seq,
        mission_timeout_sec=360.0,
        mc_transition_timeout_sec=90.0,
    ):
        self.get_logger().info(
            f"[PHASE 3] Monitoring mission until final FW seq {final_fw_seq} and MC back transition"
        )

        mission_start = time.time()
        mc_transition_start = None
        last_print = 0.0
        saw_fw = False
        mc_transition_requested = False

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            now = time.time()
            current_mode = self.state.mode if self.state else "UNKNOWN"
            current_vtol = self.extended_state.vtol_state if self.extended_state else 0

            if current_vtol == MAV_VTOL_STATE_FW:
                saw_fw = True

            if now - last_print > 2.0:
                mc_elapsed = 0.0
                if mc_transition_start is not None:
                    mc_elapsed = now - mc_transition_start

                self.get_logger().info(
                    f"[PHASE 3] mode={current_mode}, "
                    f"vtol={self.vtol_state_name(current_vtol)}, "
                    f"reached_seq={self.last_reached_seq}, "
                    f"saw_fw={saw_fw}, "
                    f"mc_transition_requested={mc_transition_requested}, "
                    f"mc_elapsed={mc_elapsed:.1f}s"
                )
                last_print = now

            if saw_fw and current_vtol == MAV_VTOL_STATE_MC:
                self.get_logger().info("[PHASE 3] Back transition to MC complete")
                return True

            if (
                not mc_transition_requested
                and saw_fw
                and current_vtol == MAV_VTOL_STATE_FW
                and self.last_reached_seq >= final_fw_seq
            ):
                self.get_logger().warn(
                    f"[PHASE 3] Final FW seq {final_fw_seq} reached while still FIXED_WING. "
                    "Sending explicit MC transition command."
                )

                if not self.request_vtol_transition_to_mc():
                    return False

                mc_transition_requested = True
                mc_transition_start = now

            allowed_modes = ["AUTO.MISSION", "AUTO.LOITER"]
            if current_mode not in allowed_modes:
                self.get_logger().error(
                    f"[PHASE 3] Mission aborted or failsafe triggered before landing: current={current_mode}"
                )
                return False

            if not mc_transition_requested:
                if now - mission_start > mission_timeout_sec:
                    self.get_logger().error("[PHASE 3] Mission monitor timeout before MC transition")
                    return False
            else:
                if mc_transition_start is not None and now - mc_transition_start > mc_transition_timeout_sec:
                    self.get_logger().error(
                        f"[PHASE 3] MC transition timeout. current_vtol={self.vtol_state_name(current_vtol)}"
                    )
                    return False

        return False

    def monitor_landing_complete(self, timeout_sec=180.0):
        self.get_logger().info("[PHASE 3] Monitoring landing until ON_GROUND")

        start = time.time()
        last_print = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            now = time.time()
            current_mode = self.state.mode if self.state else "UNKNOWN"
            landed_state = self.extended_state.landed_state if self.extended_state else 0
            current_vtol = self.extended_state.vtol_state if self.extended_state else 0
            armed = self.state.armed if self.state else False

            if now - last_print > 2.0:
                self.get_logger().info(
                    f"[PHASE 3] landing monitor: mode={current_mode}, "
                    f"vtol={self.vtol_state_name(current_vtol)}, "
                    f"landed_state={self.landed_state_name(landed_state)}, "
                    f"armed={armed}, reached_seq={self.last_reached_seq}"
                )
                last_print = now

            if landed_state == 1:
                self.get_logger().info("[PHASE 3] Vehicle ON_GROUND confirmed")
                return True

            if now - start > timeout_sec:
                self.get_logger().error("[PHASE 3] Landing monitor timeout")
                return False

        return False

    def run(self):
        self.get_logger().info("[PHASE 3] AUTO.MISSION VTOL waypoint patrol + landing start")

        if not self.wait_initial_data():
            return False

        if self.state.armed:
            self.get_logger().error("[PHASE 3] Vehicle is already armed. Start from DISARMED.")
            return False

        if self.extended_state.vtol_state != MAV_VTOL_STATE_MC:
            self.get_logger().error(
                f"[PHASE 3] Vehicle must start in MULTICOPTER. "
                f"Current: {self.vtol_state_name(self.extended_state.vtol_state)}"
            )
            return False

        if self.extended_state.landed_state != 1:
            self.get_logger().error("[PHASE 3] Vehicle must start ON_GROUND.")
            return False

        if not self.clear_mission():
            return False

        lat0 = self.global_fix.latitude
        lon0 = self.global_fix.longitude

        waypoints, mission_info = build_phase3_vtol_patrol_land_mission(
            home_lat=lat0,
            home_lon=lon0,
            mission_alt_m=30.0,
            takeoff_north_m=30.0,
            pentagon_center_north_m=350.0,
            pentagon_center_east_m=0.0,
            pentagon_side_m=200.0,
        )

        self.log_mission_info(mission_info)

        if not self.push_mission(waypoints):
            return False

        if not self.set_current_mission(0):
            return False

        # Arm the SAME way QGC "Start Mission" does: switch to AUTO.MISSION FIRST,
        # then arm. PX4 requires manual control (RC/joystick) to arm in a manual
        # mode like POSCTL — SITL has none, so arming in POSCTL is denied. Arming
        # in AUTO.MISSION needs no manual control and auto-starts the mission.
        if not self.set_mode("AUTO.MISSION"):
            return False

        if not self.wait_mode("AUTO.MISSION", timeout_sec=20.0):
            self.get_logger().error("[PHASE 3] AUTO.MISSION not entered")
            return False

        if not self.arm():
            return False

        if not self.wait_armed(True):
            return False

        if not self.monitor_until_mc_transition_complete(
            final_fw_seq=mission_info["final_fw_seq"],
            mission_timeout_sec=360.0,
            mc_transition_timeout_sec=90.0,
        ):
            return False

        # After MC back-transition, explicitly start landing.
        # This is more robust than assuming PX4 automatically advances from DO_VTOL_TRANSITION to VTOL_LAND.
        self.get_logger().info("[PHASE 3] Starting landing mission item")

        if not self.set_current_mission(mission_info["land_seq"]):
            self.get_logger().warn("[PHASE 3] Failed to set VTOL_LAND mission item. Trying AUTO.LAND fallback.")
            if not self.set_mode("AUTO.LAND"):
                return False
        else:
            if not self.set_mode("AUTO.MISSION"):
                self.get_logger().warn("[PHASE 3] AUTO.MISSION request for landing failed. Trying AUTO.LAND fallback.")
                if not self.set_mode("AUTO.LAND"):
                    return False

        # If AUTO.MISSION VTOL_LAND does not start quickly, AUTO.LAND is used as fallback.
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.1)

        current_mode = self.state.mode if self.state else "UNKNOWN"
        if current_mode not in ["AUTO.MISSION", "AUTO.LAND", "AUTO.RTL"]:
            self.get_logger().warn(
                f"[PHASE 3] Unexpected landing mode={current_mode}. Requesting AUTO.LAND fallback."
            )
            if not self.set_mode("AUTO.LAND"):
                return False

        if not self.monitor_landing_complete(timeout_sec=180.0):
            self.get_logger().warn("[PHASE 3] VTOL_LAND monitor failed. Trying AUTO.LAND fallback once more.")

            if not self.set_mode("AUTO.LAND"):
                return False

            if not self.monitor_landing_complete(timeout_sec=180.0):
                return False

        # Disarm after landing. Some PX4 configs auto-disarm; this handles both.
        if self.state is not None and self.state.armed:
            self.disarm()
            self.wait_armed(False, timeout_sec=10.0)

        self.get_logger().info("[PHASE 3] SUCCESS: waypoint patrol and landing complete")
        return True


def main():
    rclpy.init()

    node = Phase3AutoMissionVTOLLand()

    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("[PHASE 3] Interrupted by user")
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
