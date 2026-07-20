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

from mavros_msgs.msg import State, ExtendedState, WaypointReached
from mavros_msgs.srv import (
    CommandBool,
    CommandLong,
    SetMode,
    WaypointClear,
    WaypointPush,
    WaypointSetCurrent,
)
from sensor_msgs.msg import NavSatFix

from common.phase1_mission import build_phase1_vtol_transit_mission


class Phase1AutoMissionVTOL(Node):
    def __init__(self):
        super().__init__("phase1_auto_mission_vtol")

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
            WaypointReached,
            "/mavros/mission/reached",
            self.waypoint_reached_callback,
            10,
        )

        self.arming_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming",
        )

        self.set_mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode",
        )

        self.command_long_client = self.create_client(
            CommandLong,
            "/mavros/cmd/command",
        )

        self.mission_clear_client = self.create_client(
            WaypointClear,
            "/mavros/mission/clear",
        )

        self.mission_push_client = self.create_client(
            WaypointPush,
            "/mavros/mission/push",
        )

        self.mission_set_current_client = self.create_client(
            WaypointSetCurrent,
            "/mavros/mission/set_current",
        )

    def state_callback(self, msg):
        self.state = msg

    def extended_state_callback(self, msg):
        self.extended_state = msg

    def global_fix_callback(self, msg):
        self.global_fix = msg

    def waypoint_reached_callback(self, msg):
        self.last_reached_seq = msg.wp_seq
        self.get_logger().info(f"[PHASE 1] Mission reached seq: {msg.wp_seq}")

    def vtol_state_name(self, value):
        names = {
            0: "UNDEFINED",
            1: "TRANSITION_TO_FW",
            2: "TRANSITION_TO_MC",
            3: "MULTICOPTER",
            4: "FIXED_WING",
        }
        return names.get(value, f"UNKNOWN({value})")

    def wait_initial_data(self, timeout_sec=15.0):
        self.get_logger().info("[PHASE 1] Waiting for MAVROS / PX4 data")

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            state_ok = self.state is not None and self.state.connected
            gps_ok = self.global_fix is not None and self.global_fix.status.status >= 0
            ext_ok = self.extended_state is not None

            if state_ok and gps_ok and ext_ok:
                self.get_logger().info("[PHASE 1] Initial data OK")
                self.get_logger().info(f"[PHASE 1] mode: {self.state.mode}")
                self.get_logger().info(f"[PHASE 1] armed: {self.state.armed}")
                self.get_logger().info(
                    f"[PHASE 1] GPS: lat={self.global_fix.latitude:.7f}, "
                    f"lon={self.global_fix.longitude:.7f}, "
                    f"alt={self.global_fix.altitude:.2f}"
                )
                self.get_logger().info(
                    f"[PHASE 1] VTOL state: {self.vtol_state_name(self.extended_state.vtol_state)}"
                )
                self.get_logger().info(
                    f"[PHASE 1] landed_state: {self.extended_state.landed_state}"
                )
                return True

            if time.time() - start > timeout_sec:
                self.get_logger().error("[PHASE 1] Initial data timeout")
                self.get_logger().error(f"state_ok: {state_ok}")
                self.get_logger().error(f"gps_ok: {gps_ok}")
                self.get_logger().error(f"extended_state_ok: {ext_ok}")
                return False

        return False

    def wait_service(self, client, name, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"[PHASE 1] Service unavailable: {name}")
            return False
        return True

    def call_service(self, client, request, name, timeout_sec=10.0):
        future = client.call_async(request)
        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if future.done():
                result = future.result()
                self.get_logger().info(f"[PHASE 1] {name} result: {result}")
                return result

            if time.time() - start > timeout_sec:
                self.get_logger().error(f"[PHASE 1] {name} timeout")
                return None

        return None

    def set_mode(self, mode_name):
        self.get_logger().info(f"[PHASE 1] Request mode: {mode_name}")

        if not self.wait_service(self.set_mode_client, "/mavros/set_mode"):
            return False

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_name

        result = self.call_service(
            self.set_mode_client,
            req,
            f"set_mode {mode_name}",
        )

        if result is not None and result.mode_sent:
            self.get_logger().info(f"[PHASE 1] Mode request sent: {mode_name}")
            return True

        self.get_logger().error(f"[PHASE 1] Mode request failed: {mode_name}")
        return False

    def wait_mode(self, mode_name, timeout_sec=20.0):
        self.get_logger().info(f"[PHASE 1] Waiting until mode = {mode_name}")

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            current = self.state.mode if self.state else "UNKNOWN"

            if current == mode_name:
                self.get_logger().info(f"[PHASE 1] Current mode = {mode_name}")
                return True

            if time.time() - start > timeout_sec:
                self.get_logger().error(
                    f"[PHASE 1] Mode wait timeout. current={current}, target={mode_name}"
                )
                return False

        return False

    def arm(self):
        self.get_logger().info("[PHASE 1] Request ARM")

        if not self.wait_service(self.arming_client, "/mavros/cmd/arming"):
            return False

        req = CommandBool.Request()
        req.value = True

        result = self.call_service(
            self.arming_client,
            req,
            "arm",
        )

        if result is not None and result.success:
            self.get_logger().info("[PHASE 1] ARM accepted")
            return True

        self.get_logger().error("[PHASE 1] ARM failed")
        return False

    def wait_armed(self, timeout_sec=15.0):
        self.get_logger().info("[PHASE 1] Waiting until armed")

        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.state is not None and self.state.armed:
                self.get_logger().info("[PHASE 1] Vehicle armed")
                return True

            if time.time() - start > timeout_sec:
                self.get_logger().error("[PHASE 1] Arm wait timeout")
                return False

        return False

    def clear_mission(self):
        self.get_logger().info("[PHASE 1] Clear existing mission")

        if not self.wait_service(self.mission_clear_client, "/mavros/mission/clear"):
            return False

        req = WaypointClear.Request()

        result = self.call_service(
            self.mission_clear_client,
            req,
            "mission_clear",
        )

        if result is not None and result.success:
            self.get_logger().info("[PHASE 1] Mission clear success")
            return True

        self.get_logger().error("[PHASE 1] Mission clear failed")
        return False

    def set_current_mission(self, seq=0):
        self.get_logger().info(f"[PHASE 1] Set current mission seq: {seq}")

        if not self.wait_service(
            self.mission_set_current_client,
            "/mavros/mission/set_current",
        ):
            return False

        req = WaypointSetCurrent.Request()
        req.wp_seq = int(seq)

        result = self.call_service(
            self.mission_set_current_client,
            req,
            "mission_set_current",
        )

        if result is not None and result.success:
            self.get_logger().info("[PHASE 1] Set current mission success")
            return True

        self.get_logger().error("[PHASE 1] Set current mission failed")
        return False

    def push_mission(self, waypoints):
        self.get_logger().info("[PHASE 1] Push VTOL mission to PX4")

        if not self.wait_service(self.mission_push_client, "/mavros/mission/push"):
            return False

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints

        result = self.call_service(
            self.mission_push_client,
            req,
            "mission_push",
            timeout_sec=15.0,
        )

        if result is not None and result.success:
            self.get_logger().info(
                f"[PHASE 1] Mission push success. transferred={result.wp_transfered}"
            )
            return True

        self.get_logger().error("[PHASE 1] Mission push failed")
        return False

    def log_mission_info(self, mission_info):
        self.get_logger().info("[PHASE 1] Built AUTO.MISSION VTOL sequence")
        self.get_logger().info(
            f"[PHASE 1] start:   {mission_info['start'][0]:.7f}, {mission_info['start'][1]:.7f}"
        )
        self.get_logger().info(
            f"[PHASE 1] takeoff: {mission_info['takeoff'][0]:.7f}, {mission_info['takeoff'][1]:.7f}"
        )

        if "fw_wps" in mission_info:
            for idx, (lat, lon) in enumerate(mission_info["fw_wps"], start=1):
                self.get_logger().info(
                    f"[PHASE 1] fw_wp{idx}:  {lat:.7f}, {lon:.7f}"
                )
        else:
            self.get_logger().info(
                f"[PHASE 1] fw_wp:   {mission_info['fw_wp'][0]:.7f}, {mission_info['fw_wp'][1]:.7f}"
            )

        self.get_logger().info(
            f"[PHASE 1] mc_wp:   {mission_info['mc_wp'][0]:.7f}, {mission_info['mc_wp'][1]:.7f}"
        )

        if "pentagon_side_m" in mission_info:
            self.get_logger().info(
                f"[PHASE 1] pentagon side: {mission_info['pentagon_side_m']:.1f} m"
            )

        self.get_logger().info(f"[PHASE 1] item count: {mission_info['count']}")

    def request_vtol_transition_to_mc(self):
        self.get_logger().info("[PHASE 1] Request VTOL transition to MULTICOPTER")

        if not self.wait_service(self.command_long_client, "/mavros/cmd/command"):
            return False

        req = CommandLong.Request()
        req.broadcast = False
        req.command = 3000  # MAV_CMD_DO_VTOL_TRANSITION
        req.confirmation = 0
        req.param1 = 3.0  # MAV_VTOL_STATE_MC / MULTICOPTER
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
            self.get_logger().info("[PHASE 1] VTOL transition to MC accepted")
            return True

        self.get_logger().error("[PHASE 1] VTOL transition to MC failed")
        return False

    def monitor_until_mc_back_transition(
        self,
        final_fw_seq=5,
        mission_timeout_sec=300.0,
        mc_transition_timeout_sec=90.0,
    ):
        self.get_logger().info(
            f"[PHASE 1] Monitoring mission until final FW seq {final_fw_seq} and MC back transition"
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

            if current_vtol == 4:
                saw_fw = True

            if now - last_print > 2.0:
                mc_elapsed = 0.0
                if mc_transition_start is not None:
                    mc_elapsed = now - mc_transition_start

                self.get_logger().info(
                    f"[PHASE 1] mode={current_mode}, "
                    f"vtol={self.vtol_state_name(current_vtol)}, "
                    f"reached_seq={self.last_reached_seq}, "
                    f"saw_fw={saw_fw}, "
                    f"mc_transition_requested={mc_transition_requested}, "
                    f"mc_elapsed={mc_elapsed:.1f}s"
                )
                last_print = now

            # Success condition: aircraft has flown as FW, then returned to MC.
            if saw_fw and current_vtol == 3:
                self.get_logger().info("[PHASE 1] Back transition to MC complete")
                return True

            # PX4 mission may stay in FW after the final FW waypoint.
            # In that case, explicitly request VTOL back-transition only after seq >= final_fw_seq.
            if (
                not mc_transition_requested
                and saw_fw
                and current_vtol == 4
                and self.last_reached_seq >= final_fw_seq
            ):
                self.get_logger().warn(
                    f"[PHASE 1] Final FW seq {final_fw_seq} reached while still FIXED_WING. "
                    "Sending explicit MC transition command."
                )

                if not self.request_vtol_transition_to_mc():
                    return False

                mc_transition_requested = True
                mc_transition_start = now

            allowed_modes = ["AUTO.MISSION", "AUTO.LOITER"]
            if current_mode not in allowed_modes:
                self.get_logger().error(
                    f"[PHASE 1] Mission aborted or failsafe triggered: current={current_mode}"
                )
                return False

            # Before requesting MC transition, use the mission timeout.
            # After requesting MC transition, give PX4 a separate transition timeout.
            # This prevents timeout exactly while PX4 is already in TRANSITION_TO_MC.
            if not mc_transition_requested:
                if now - mission_start > mission_timeout_sec:
                    self.get_logger().error("[PHASE 1] Mission monitor timeout before MC transition request")
                    return False
            else:
                if mc_transition_start is not None and now - mc_transition_start > mc_transition_timeout_sec:
                    self.get_logger().error(
                        f"[PHASE 1] MC transition timeout. "
                        f"current_vtol={self.vtol_state_name(current_vtol)}"
                    )
                    return False

        return False

    def run(self):
        self.get_logger().info("[PHASE 1] AUTO.MISSION VTOL phase start")

        if not self.wait_initial_data():
            return False

        if self.state.armed:
            self.get_logger().error("[PHASE 1] Vehicle is already armed. Start from DISARMED.")
            return False

        if self.extended_state.vtol_state != 3:
            self.get_logger().error(
                f"[PHASE 1] Vehicle must start in MULTICOPTER. "
                f"Current: {self.vtol_state_name(self.extended_state.vtol_state)}"
            )
            return False

        if self.extended_state.landed_state != 1:
            self.get_logger().error("[PHASE 1] Vehicle must start ON_GROUND.")
            return False

        if not self.clear_mission():
            return False

        lat0 = self.global_fix.latitude
        lon0 = self.global_fix.longitude

        waypoints, mission_info = build_phase1_vtol_transit_mission(
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
        # in the AUTO.MISSION mode needs no manual control, and (with a valid
        # mission whose first item is a TAKEOFF) it auto-starts the mission.
        if not self.set_mode("AUTO.MISSION"):
            return False

        if not self.wait_mode("AUTO.MISSION", timeout_sec=20.0):
            self.get_logger().error("[PHASE 1] AUTO.MISSION not entered")
            return False

        if not self.arm():
            return False

        if not self.wait_armed():
            return False

        # Current mission geometry reaches the final fixed-wing waypoint at seq 5.
        # After seq 5, explicitly command VTOL back-transition to multicopter,
        # then switch to PX4 Hold/Loiter mode for the Phase 2 handoff.
        if not self.monitor_until_mc_back_transition(final_fw_seq=5, mission_timeout_sec=300.0, mc_transition_timeout_sec=90.0):
            return False

        self.get_logger().info("[PHASE 1] Multicopter mode confirmed")
        self.get_logger().info("[PHASE 1] Switching to AUTO.LOITER for Hold / Phase 2 handoff")

        if not self.set_mode("AUTO.LOITER"):
            return False

        if not self.wait_mode("AUTO.LOITER", timeout_sec=15.0):
            self.get_logger().error("[PHASE 1] AUTO.LOITER mode not entered")
            return False

        time.sleep(2.0)

        self.get_logger().info("[PHASE 1] SUCCESS")
        return True


def main():
    rclpy.init()

    node = Phase1AutoMissionVTOL()

    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("[PHASE 1] Interrupted by user")
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
