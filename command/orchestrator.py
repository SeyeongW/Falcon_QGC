#!/usr/bin/env python3
"""Onboard phase orchestrator for the custom QGC mission panel.

QGC publishes a phase number on `command/run_phase` (std_msgs/Int32) when the
operator clicks a phase button. This node looks up that id in its local dynamic
catalog, runs the corresponding local Python script independently, and streams
live progress back on `command/status` (std_msgs/String, JSON payload).

Independent payload controls arrive on `command/run_action` (std_msgs/String):
`camera:on`, `camera:off`, `gripper:open`, `gripper:close`, `gripper:stop`,
or `failsafe:run`.

The phase catalog is published on `command/catalog` (std_msgs/String, JSON).
`phases.json` supplies display metadata and may map ids to any Python script
below this directory. Unlisted `phaseN.py` files are discovered automatically.
The catalog is reloaded at runtime, so adding a valid local phase does not
require restarting this node.

Status JSON:
    {"phase": int,
     "state": "idle|running|awaiting_confirmation|done|failed",
     "msg": str, "progress": float(-1..1), "done": [completed phase ids],
     "prompt": ""|"ok"|"ok_again"|"land",
     "actions": {camera/gripper availability and live state}}

Catalog JSON:
    {"version": 1, "phases": [
        {"id": int, "title": str, "desc": str, "script": str,
         "independent": true,
         "confirmation": "none"|"ok"|"ok_again"|"land",
         "start_action": "run_script"|"start_mission",
         "on_ok": "complete"|"start_mission"}]}

Run on the aircraft mission computer:
    python3 command/orchestrator.py   (needs MAVROS running for vehicle control)
"""
import fcntl
import json
import os
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from std_msgs.msg import Bool, Empty, Int32, String
from mavros_msgs.msg import ExtendedState, State, WaypointList, WaypointReached
from mavros_msgs.srv import CommandLong, SetMode

from common.phase_catalog import CatalogError, catalog_payload, load_phase_catalog


# Directory that holds this orchestrator and the phaseN.py scripts. Derived from
# this file's own location so the folder can live anywhere (any user's home, /opt,
# a USB mount, a colcon workspace) and still find the phases — no hardcoded path.
COMMAND_DIR = os.path.dirname(os.path.abspath(__file__))
CATALOG_REFRESH_SEC = 1.0

# VTOL states (mavros ExtendedState.vtol_state)
VTOL_TRANS_TO_FW = 1
VTOL_TRANS_TO_MC = 2
VTOL_MC = 3
VTOL_FW = 4

# PX4 flight mode used to hand control back to the GCS on abort: HOLD makes the
# vehicle hover in place (multicopter) / loiter (fixed wing) and wait for the
# operator's next command from QGC.
PX4_HOLD_MODE = "AUTO.LOITER"
PX4_MISSION_MODE = "AUTO.MISSION"
PX4_LAND_MODE = "AUTO.LAND"
MAV_CMD_MISSION_START = 300
MISSION_MODE_REQUEST_INTERVAL_SEC = 1.0
MISSION_MODE_ENTRY_TIMEOUT_SEC = 15.0
LAND_MODE_REQUEST_INTERVAL_SEC = 1.0
LAND_MODE_ENTRY_TIMEOUT_SEC = 15.0
MAV_CMD_NAV_WAYPOINT = 16

CAMERA_SCRIPT = "robo_jinheui_pt.py"
GRIPPER_SCRIPTS = {
    "open": "gripper_open.py",
    "close": "gripper_close.py",
}
FAILSAFE_SCRIPT = "failsafe.py"


class PhaseOrchestrator(Node):
    def __init__(self):
        super().__init__("phase_orchestrator")

        self.status_pub = self.create_publisher(String, "command/status", 10)
        catalog_qos = QoSProfile(depth=1)
        catalog_qos.reliability = ReliabilityPolicy.RELIABLE
        catalog_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.catalog_pub = self.create_publisher(
            String, "command/catalog", catalog_qos
        )
        self.create_subscription(Int32, "command/run_phase", self._on_run_phase, 10)
        self.create_subscription(
            String,
            "command/phase_response",
            self._on_phase_response,
            10,
        )
        self.create_subscription(String, "command/run_action", self._on_run_action, 10)
        self.create_subscription(
            Bool,
            "/mission/ready_for_land",
            self._on_ready_for_land,
            10,
        )
        # GCS take-over: abort the running phase and hand control back (HOLD).
        self.create_subscription(Empty, "command/abort", self._on_abort, 10)
        self.set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.mission_start_cli = self.create_client(
            CommandLong,
            "/mavros/cmd/command",
        )
        self.land_confirm_pub = self.create_publisher(
            Bool,
            "/mission/land_confirm",
            10,
        )

        # Live vehicle state used to describe the current mission section.
        self._last_wp = -1
        self._uploaded_mission_last_wp = -1
        self._vtol = 0
        self._landed_state = 0
        self._mode = ""
        self._armed = False

        rel = QoSProfile(depth=10)
        rel.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self._on_wp, 10)
        self.create_subscription(
            WaypointList,
            "/mavros/mission/waypoints",
            self._on_mission_waypoints,
            10,
        )
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_ext, qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state", self._on_state, rel)

        # Orchestration state.
        self._done = set()      # completed phase ids
        self._running = None    # currently running phase id (or None)
        self._pending_confirmation = None
        self._ready_for_land_seen = False
        self._mission_monitor_phase = None
        self._mission_completion_seq = -1
        self._mission_mode_confirmed = False
        self._mission_mode_request_started = 0.0
        self._mission_mode_last_request = 0.0
        self._mission_start_sent = False
        self._land_confirm_sent = False
        self._waiting_for_landing_phase = None
        self._land_handoff_phase = None
        self._land_mode_request_started = 0.0
        self._land_mode_last_request = 0.0
        self._proc = None
        self._camera_proc = None
        self._camera_stopping = False
        self._gripper_proc = None
        self._gripper_busy = False
        self._gripper_state = "unknown"
        self._failsafe_proc = None
        self._action_msg = ""
        self._shutting_down = False
        self._last_log = ""     # latest stdout line from the running phase
        self._aborting = False  # True while a GCS take-over is tearing a phase down
        self._phases = {}       # validated local phase definitions, keyed by id
        self._catalog_json = ""
        self._last_catalog_error = ""

        # Latest status payload, republished every tick so a late subscriber
        # (e.g. QGC connecting after boot) always sees the current state.
        self._status = {
            "phase": -1,
            "state": "idle",
            "msg": "대기 중",
            "progress": -1.0,
            "done": [],
            "prompt": "",
        }

        self._reload_catalog(force=True)
        self.create_timer(CATALOG_REFRESH_SEC, self._reload_catalog)
        self.create_timer(0.5, self._tick)  # push live status ~2 Hz
        self._publish("idle", -1, "대기 중", phase=-1)
        self.get_logger().info(
            f"orchestrator up ({len(self._phases)} phases, dir={COMMAND_DIR})")

    # --- dynamic phase catalog ----------------------------------------------
    def _reload_catalog(self, force=False):
        """Reload local phase definitions and publish changes to FGC."""
        try:
            phases = load_phase_catalog(COMMAND_DIR)
            payload = json.dumps(catalog_payload(phases), ensure_ascii=False)
        except CatalogError as exc:
            error = str(exc)
            if error != self._last_catalog_error:
                self.get_logger().error(f"phase catalog 갱신 실패: {error}")
                self._last_catalog_error = error
            return

        self._last_catalog_error = ""
        if not force and payload == self._catalog_json:
            return

        self._phases = phases
        self._catalog_json = payload
        self._done.intersection_update(phases)
        if hasattr(self, "_status"):
            self._status["done"] = sorted(self._done)
        self._publish_catalog()

        ids = ", ".join(str(phase_id) for phase_id in phases) or "none"
        self.get_logger().info(f"phase catalog 갱신됨 (ids: {ids})")
        if any(not hasattr(phase, "available") for phase in phases.values()):
            self.get_logger().warning(
                "common/phase_catalog.py가 이전 버전입니다. command 폴더 전체를 "
                "같은 버전으로 갱신하세요. 기본 Phase 동작으로 호환 실행합니다."
            )

    def _publish_catalog(self):
        if not self._catalog_json:
            return
        msg = String()
        msg.data = self._catalog_json
        self.catalog_pub.publish(msg)

    # --- vehicle state callbacks --------------------------------------------
    def _on_wp(self, m):
        self._last_wp = m.wp_seq
        if (
            self._mission_monitor_phase is not None
            and self._mission_completion_seq >= 0
            and m.wp_seq >= self._mission_completion_seq
        ):
            phase_id = self._mission_monitor_phase
            phase = self._phases.get(phase_id)
            title = phase.title if phase is not None else f"Phase {phase_id}"
            self._clear_mission_monitor()
            # A mission which ends with a normal Waypoint can otherwise retain
            # firmware-dependent end-of-mission behavior. Explicit HOLD keeps
            # the aircraft at the final target while the operator confirms it.
            self._set_hold_mode()
            if (
                phase is not None
                and getattr(phase, "start_action", "run_script")
                == "start_mission"
                and getattr(phase, "confirmation", "none") != "none"
            ):
                self._publish_phase_confirmation(phase)
            else:
                self._done.add(phase_id)
                self._running = None
                self._publish(
                    "done",
                    1.0,
                    f"{title} 마지막 Waypoint 도착 완료",
                    phase=phase_id,
                )

    def _on_mission_waypoints(self, msg):
        nav_waypoint_sequences = [
            sequence
            for sequence, waypoint in enumerate(msg.waypoints)
            if waypoint.command == MAV_CMD_NAV_WAYPOINT
        ]
        self._uploaded_mission_last_wp = (
            max(nav_waypoint_sequences) if nav_waypoint_sequences else -1
        )

    def _on_ext(self, m):
        self._vtol = m.vtol_state
        self._landed_state = m.landed_state

        phase_id = self._waiting_for_landing_phase
        if phase_id is not None and self._landed_state == 1:
            phase = self._phases.get(phase_id)
            self._waiting_for_landing_phase = None
            if phase is None:
                self._running = None
                self._publish(
                    "failed",
                    -1,
                    f"Phase {phase_id} 설정이 사라졌습니다",
                    phase=phase_id,
                )
                return
            self._publish_phase_confirmation(phase)

    def _on_state(self, m):
        self._mode = m.mode
        self._armed = m.armed

    def _on_ready_for_land(self, msg):
        """Show the Land confirmation as soon as a vision phase is aligned."""
        if not bool(msg.data) or self._running not in (2, 4):
            return

        phase = self._phases.get(self._running)
        if phase is None or getattr(phase, "ready_action", "none") != "land":
            return
        if self._ready_for_land_seen or self._land_handoff_phase is not None:
            return

        self._ready_for_land_seen = True
        self._publish_phase_confirmation(phase)

    # --- current-section description (mainly for the phase-1 VTOL mission) ---
    def _section_desc(self):
        # VTOL transitions are the clearest signal.
        if self._vtol == VTOL_TRANS_TO_FW:
            return "고정익 천이 중"
        if self._vtol == VTOL_TRANS_TO_MC:
            return "멀티콥터 역천이 중"

        seq = self._last_wp
        # phase-1 mission layout: 0=takeoff, 1=trans_fw, 2..6=WP1..5, 7=trans_mc, 8=land.
        # After reaching item S the vehicle heads toward S+1.
        if seq < 0:
            if self._armed:
                return "이륙 중"
            return None
        if seq == 0:
            return "이륙 완료 · 천이 준비"
        if 1 <= seq <= 5:
            wing = " (고정익)" if self._vtol == VTOL_FW else ""
            return f"WP{seq} 이동 중{wing}"
        if seq == 6:
            return "WP5 도달 · 역천이 준비"
        if seq >= 7:
            return "복귀 / 착륙 구간"
        return f"구간 seq {seq}"

    def _progress(self):
        if self._mission_monitor_phase is not None and self._mission_completion_seq >= 0:
            if self._last_wp < 0:
                return 0.0
            return min(
                1.0,
                max(0.0, self._last_wp / max(1, self._mission_completion_seq)),
            )
        # Rough progress for the phase-1 mission (8 items); unknown otherwise.
        if self._last_wp >= 0:
            return min(1.0, max(0.0, self._last_wp / 8.0))
        return -1.0

    def _clear_mission_monitor(self):
        self._mission_monitor_phase = None
        self._mission_completion_seq = -1
        self._mission_mode_confirmed = False
        self._mission_mode_request_started = 0.0
        self._mission_mode_last_request = 0.0

    def _request_mission_mode(self):
        if not self.set_mode_cli.service_is_ready():
            return False

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = PX4_MISSION_MODE
        self.set_mode_cli.call_async(request)
        self._mission_mode_last_request = time.monotonic()
        return True

    def _request_mission_start(self):
        """Explicitly start the uploaded mission after entering AUTO.MISSION."""
        if not self.mission_start_cli.service_is_ready():
            return False

        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_MISSION_START
        request.confirmation = 0
        request.param1 = 0.0  # first mission item
        request.param2 = 0.0  # last mission item (autopilot decides)
        self.mission_start_cli.call_async(request)
        self._mission_start_sent = True
        return True

    def _start_uploaded_mission(self, phase_id, phase):
        if self._uploaded_mission_last_wp < 0:
            if self._pending_confirmation is not None:
                self._publish(
                    "awaiting_confirmation",
                    1.0,
                    "업로드된 일반 Waypoint 미션이 없습니다. 미션 업로드 후 OK를 다시 누르세요.",
                    phase=phase_id,
                    prompt=self._pending_confirmation,
                )
            else:
                self._running = None
                self._publish(
                    "failed",
                    -1,
                    "업로드된 일반 Waypoint 미션이 없습니다. 미션 업로드 후 Phase를 다시 실행하세요.",
                    phase=phase_id,
                )
            return False

        if not self.set_mode_cli.service_is_ready():
            if self._pending_confirmation is not None:
                self._publish(
                    "awaiting_confirmation",
                    1.0,
                    "/mavros/set_mode 연결 대기 중입니다. 연결 확인 후 OK를 다시 누르세요.",
                    phase=phase_id,
                    prompt=self._pending_confirmation,
                )
            else:
                self._running = None
                self._publish(
                    "failed",
                    -1,
                    "/mavros/set_mode 연결 대기 중입니다. 연결 확인 후 Phase를 다시 실행하세요.",
                    phase=phase_id,
                )
            return False

        now = time.monotonic()
        self._pending_confirmation = None
        self._mission_monitor_phase = phase_id
        self._mission_completion_seq = self._uploaded_mission_last_wp
        self._mission_mode_confirmed = False
        self._mission_mode_request_started = now
        self._mission_mode_last_request = 0.0
        self._mission_start_sent = False
        self._last_wp = -1

        self._request_mission_mode()
        self._publish(
            "running",
            0.0,
            f"{phase.title} — AUTO.MISSION 전환 요청",
            phase=phase_id,
        )
        return True

    def _tick_mission_monitor(self):
        phase_id = self._mission_monitor_phase
        if phase_id is None:
            return

        now = time.monotonic()
        if self._mode == PX4_MISSION_MODE:
            self._mission_mode_confirmed = True
            if not self._mission_start_sent:
                self._request_mission_start()
            text = (
                f"Mission 비행 중 — WP {self._last_wp}/{self._mission_completion_seq}"
                if self._last_wp >= 0
                else "Mission 모드 진입 완료 — 첫 Waypoint 이동 중"
            )
            self._publish("running", self._progress(), text, phase=phase_id)
            return

        if self._mission_mode_confirmed:
            self._running = None
            self._clear_mission_monitor()
            self._publish(
                "failed",
                -1,
                f"Phase {phase_id} Mission 비행 중 모드 이탈: {self._mode or 'UNKNOWN'}",
                phase=phase_id,
            )
            return

        if now - self._mission_mode_request_started > MISSION_MODE_ENTRY_TIMEOUT_SEC:
            self._running = None
            self._clear_mission_monitor()
            self._publish(
                "failed",
                -1,
                f"Phase {phase_id} AUTO.MISSION 전환 시간 초과",
                phase=phase_id,
            )
            return

        if now - self._mission_mode_last_request >= MISSION_MODE_REQUEST_INTERVAL_SEC:
            self._request_mission_mode()

        self._publish(
            "running",
            0.0,
            "AUTO.MISSION 모드 전환 확인 중",
            phase=phase_id,
        )

    def _clear_land_handoff(self):
        self._land_handoff_phase = None
        self._land_mode_request_started = 0.0
        self._land_mode_last_request = 0.0

    def _request_land_mode(self):
        if not self.set_mode_cli.service_is_ready():
            return False

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = PX4_LAND_MODE
        self.set_mode_cli.call_async(request)
        self._land_mode_last_request = time.monotonic()
        return True

    def _start_land_handoff(self, phase_id):
        now = time.monotonic()
        self._land_handoff_phase = phase_id
        self._land_mode_request_started = now
        self._land_mode_last_request = 0.0
        self._request_land_mode()
        self._publish(
            "running",
            self._progress(),
            "Land 승인됨 — AUTO.LAND 전환 요청",
            phase=phase_id,
        )

    def _publish_land_confirm(self, confirmed):
        msg = Bool()
        msg.data = bool(confirmed)
        self.land_confirm_pub.publish(msg)

    def _tick_land_handoff(self):
        phase_id = self._land_handoff_phase
        if phase_id is None:
            return

        now = time.monotonic()
        if self._mode == PX4_LAND_MODE:
            phase = self._phases.get(phase_id)
            title = phase.title if phase is not None else f"Phase {phase_id}"
            self._clear_land_handoff()
            self._running = None
            self._pending_confirmation = None
            self._done.add(phase_id)
            self._publish(
                "done",
                1.0,
                f"{title} 완료 — AUTO.LAND 인가됨",
                phase=phase_id,
            )
            return

        if now - self._land_mode_request_started > LAND_MODE_ENTRY_TIMEOUT_SEC:
            self._clear_land_handoff()
            if self._proc is not None and self._proc.poll() is None:
                self._proc.terminate()
            self._publish(
                "failed",
                -1,
                f"Phase {phase_id} AUTO.LAND 전환 시간 초과",
                phase=phase_id,
            )
            return

        if now - self._land_mode_last_request >= LAND_MODE_REQUEST_INTERVAL_SEC:
            self._request_land_mode()

        self._publish(
            "running",
            self._progress(),
            "AUTO.LAND 모드 전환 확인 중",
            phase=phase_id,
        )

    def _publish_phase_confirmation(self, phase):
        confirmation = getattr(phase, "confirmation", "none")
        confirm_after = getattr(phase, "confirm_after", "process_exit")
        start_action = getattr(phase, "start_action", "run_script")
        on_ok = getattr(phase, "on_ok", "complete")
        self._pending_confirmation = confirmation
        if confirmation == "land":
            confirmation_message = "정렬이 완료 됐습니다. Land 하시겠습니까?"
        elif confirm_after == "landed":
            confirmation_message = (
                f"{phase.title} 착륙 완료 — 위치 확인 후 OK 또는 Again을 선택하세요"
            )
        elif start_action == "start_mission":
            confirmation_message = (
                f"{phase.title} Mission 종료 — 확인 후 OK를 눌러주세요"
            )
        elif on_ok == "start_mission":
            confirmation_message = (
                f"{phase.title} 호버링 완료 — OK를 누르면 업로드 미션을 시작합니다"
            )
        else:
            confirmation_message = f"{phase.title} 정상 종료 — 사용자 확인 대기"

        self._publish(
            "awaiting_confirmation",
            1.0,
            confirmation_message,
            phase=phase.phase_id,
            prompt=confirmation,
        )

    # --- camera / gripper actions ------------------------------------------
    @staticmethod
    def _action_script_path(script_name):
        return os.path.join(COMMAND_DIR, script_name)

    def _action_status(self):
        camera_running = (
            self._camera_proc is not None
            and self._camera_proc.poll() is None
        )
        failsafe_proc = getattr(self, "_failsafe_proc", None)
        return {
            "camera_available": os.path.isfile(
                self._action_script_path(CAMERA_SCRIPT)
            ),
            "camera_running": camera_running,
            "gripper_open_available": os.path.isfile(
                self._action_script_path(GRIPPER_SCRIPTS["open"])
            ),
            "gripper_close_available": os.path.isfile(
                self._action_script_path(GRIPPER_SCRIPTS["close"])
            ),
            "gripper_busy": self._gripper_busy,
            "gripper_state": self._gripper_state,
            "failsafe_available": os.path.isfile(
                self._action_script_path(FAILSAFE_SCRIPT)
            ),
            "failsafe_running": (
                failsafe_proc is not None
                and failsafe_proc.poll() is None
            ),
            "msg": self._action_msg,
        }

    def _on_run_action(self, msg):
        command = msg.data.strip().lower()
        if command == "camera:on":
            self._start_camera()
        elif command == "camera:off":
            self._stop_camera()
        elif command in ("gripper:open", "gripper:close"):
            self._start_gripper(command.split(":", 1)[1])
        elif command == "gripper:stop":
            self._stop_gripper()
        elif command == "failsafe:run":
            self._start_failsafe()
        else:
            self._action_msg = f"알 수 없는 장치 명령: {msg.data}"
            self.get_logger().warning(self._action_msg)
            self._republish()

    def _start_camera(self):
        if self._camera_proc is not None and self._camera_proc.poll() is None:
            self._action_msg = "Cam이 이미 ON 상태입니다"
            self._republish()
            return

        script = self._action_script_path(CAMERA_SCRIPT)
        if not os.path.isfile(script):
            self._camera_proc = None
            self._action_msg = f"{CAMERA_SCRIPT} 없음"
            self.get_logger().error(self._action_msg)
            self._republish()
            return

        try:
            self._camera_proc = subprocess.Popen(
                [sys.executable, script],
                cwd=COMMAND_DIR,
            )
        except Exception as exc:  # noqa: BLE001
            self._camera_proc = None
            self._action_msg = f"Cam 실행 실패: {exc}"
            self.get_logger().error(self._action_msg)
            self._republish()
            return

        self._camera_stopping = False
        self._action_msg = "Cam ON"
        self.get_logger().info(self._action_msg)
        self._republish()

    def _stop_camera(self):
        proc = self._camera_proc
        if proc is None or proc.poll() is not None:
            self._camera_proc = None
            self._camera_stopping = False
            self._action_msg = "Cam이 이미 OFF 상태입니다"
            self._republish()
            return

        self._camera_stopping = True
        self._action_msg = "Cam OFF 처리 중"
        proc.terminate()
        threading.Thread(
            target=self._kill_after,
            args=(proc, 2.0),
            daemon=True,
        ).start()
        self._republish()

    def _start_gripper(self, action):
        script_name = GRIPPER_SCRIPTS[action]
        script = self._action_script_path(script_name)
        if not os.path.isfile(script):
            self._action_msg = f"{script_name} 없음"
            self.get_logger().error(self._action_msg)
            self._republish()
            return

        # Gripper scripts may intentionally stay alive to keep applying force.
        # A new click must therefore supersede the previous command instead of
        # waiting for that process to exit. Stop the previous publisher first so
        # opposing open/close commands cannot fight each other, then launch the
        # newly requested action immediately.
        previous_proc = self._gripper_proc
        if previous_proc is not None and previous_proc.poll() is None:
            # Detach it before sending SIGTERM so its monitor thread cannot
            # briefly publish a failure over the replacement command.
            self._gripper_proc = None
            self._gripper_busy = False
            previous_proc.terminate()
            threading.Thread(
                target=self._kill_after,
                args=(previous_proc, 2.0),
                daemon=True,
            ).start()

        try:
            proc = subprocess.Popen(
                [sys.executable, script],
                cwd=COMMAND_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
        except Exception as exc:  # noqa: BLE001
            self._gripper_proc = None
            self._gripper_busy = False
            self._action_msg = f"Gripper {action.title()} 실행 실패: {exc}"
            self.get_logger().error(self._action_msg)
            self._republish()
            return

        self._gripper_proc = proc
        # This process may stay alive to hold force. Report it as running so the
        # GCS can show the active gripper command and turn its button into a
        # stop control. The buttons remain enabled independently in the GCS.
        self._gripper_busy = True
        self._gripper_state = action
        self._action_msg = f"Gripper {action.title()} 실행 중"
        self.get_logger().info(self._action_msg)
        self._republish()
        threading.Thread(
            target=self._monitor_gripper,
            args=(action, proc),
            daemon=True,
        ).start()

    def _stop_gripper(self):
        proc = self._gripper_proc
        if proc is None or proc.poll() is not None:
            self._gripper_proc = None
            self._gripper_busy = False
            self._gripper_state = "stopped"
            self._action_msg = "Gripper가 이미 정지 상태입니다"
            self._republish()
            return

        action = self._gripper_state
        # Detach before SIGTERM so the monitor thread cannot report the
        # intentional stop as an action failure. SIGKILL follows after two
        # seconds if the payload script ignores SIGTERM.
        self._gripper_proc = None
        self._gripper_busy = False
        self._gripper_state = "stopped"
        self._action_msg = f"Gripper {action.title()} 종료"
        proc.terminate()
        threading.Thread(
            target=self._kill_after,
            args=(proc, 2.0),
            daemon=True,
        ).start()
        self.get_logger().info(self._action_msg)
        self._republish()

    def _start_failsafe(self):
        if self._running not in (2, 4):
            self._action_msg = "Failsafe는 Phase 2 또는 Phase 4 실행 중에만 사용할 수 있습니다"
            self.get_logger().warning(self._action_msg)
            self._republish()
            return

        if self._failsafe_proc is not None and self._failsafe_proc.poll() is None:
            self._action_msg = "Failsafe가 이미 실행 중입니다"
            self._republish()
            return

        script = self._action_script_path(FAILSAFE_SCRIPT)
        if not os.path.isfile(script):
            self._failsafe_proc = None
            self._action_msg = f"{FAILSAFE_SCRIPT} 없음"
            self.get_logger().error(self._action_msg)
            self._republish()
            return

        try:
            proc = subprocess.Popen(
                [sys.executable, script],
                cwd=COMMAND_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
        except Exception as exc:  # noqa: BLE001
            self._failsafe_proc = None
            self._action_msg = f"Failsafe 실행 실패: {exc}"
            self.get_logger().error(self._action_msg)
            self._republish()
            return

        self._failsafe_proc = proc
        self._action_msg = "Failsafe 실행 중"
        self.get_logger().warning(self._action_msg)
        self._republish()
        threading.Thread(
            target=self._monitor_failsafe,
            args=(proc,),
            daemon=True,
        ).start()

    def _monitor_failsafe(self, proc):
        try:
            output, _ = proc.communicate()
            rc = proc.returncode
        except Exception as exc:  # noqa: BLE001
            if self._failsafe_proc is not proc:
                return
            self._action_msg = f"Failsafe 실행 실패: {exc}"
            self.get_logger().error(self._action_msg)
        else:
            if self._failsafe_proc is not proc:
                return
            detail = output.strip().splitlines()[-1] if output.strip() else ""
            if rc == 0:
                self._action_msg = "Failsafe 완료"
                self.get_logger().info(self._action_msg)
            else:
                self._action_msg = (
                    f"Failsafe 실패 (exit {rc})"
                    + (f" — {detail}" if detail else "")
                )
                self.get_logger().error(self._action_msg)

        if self._failsafe_proc is proc:
            self._failsafe_proc = None
            self._republish()

    def _stop_failsafe(self, message):
        proc = getattr(self, "_failsafe_proc", None)
        if proc is None or proc.poll() is not None:
            self._failsafe_proc = None
            return

        # Failsafe is scoped to the active Phase 2/4 run. Detach it before
        # SIGTERM so its monitor cannot overwrite the intentional-stop status.
        self._failsafe_proc = None
        proc.terminate()
        threading.Thread(
            target=self._kill_after,
            args=(proc, 2.0),
            daemon=True,
        ).start()
        self._action_msg = message
        self.get_logger().info(self._action_msg)
        self._republish()

    def _monitor_gripper(self, action, proc):
        try:
            output, _ = proc.communicate()
            rc = proc.returncode
        except Exception as exc:  # noqa: BLE001
            # A superseded process must not overwrite the state of the newer
            # command that replaced it.
            if self._gripper_proc is not proc:
                return
            self._action_msg = f"Gripper {action.title()} 실행 실패: {exc}"
            self.get_logger().error(self._action_msg)
        else:
            if self._gripper_proc is not proc:
                return
            if rc == 0:
                self._action_msg = f"Gripper {action.title()} 완료"
                self.get_logger().info(self._action_msg)
            else:
                detail = output.strip().splitlines()[-1] if output.strip() else ""
                self._action_msg = (
                    f"Gripper {action.title()} 실패 (exit {rc})"
                    + (f" — {detail}" if detail else "")
                )
                self.get_logger().error(self._action_msg)

        if self._gripper_proc is proc:
            self._gripper_proc = None
            self._gripper_busy = False
            self._republish()

    def _update_action_processes(self):
        if self._camera_proc is None:
            return

        rc = self._camera_proc.poll()
        if rc is None:
            return

        self._camera_proc = None
        if self._camera_stopping:
            self._action_msg = "Cam OFF"
            self.get_logger().info(self._action_msg)
        else:
            self._action_msg = f"Cam이 예기치 않게 종료됨 (exit {rc})"
            self.get_logger().error(self._action_msg)
        self._camera_stopping = False

    # --- run-phase handling --------------------------------------------------
    def _on_run_phase(self, msg):
        try:
            n = int(msg.data)
            self._start_phase_request(n)
        except Exception as exc:  # noqa: BLE001
            # An exception raised from an rclpy subscription callback escapes
            # executor.spin_once() and terminates the whole orchestrator. Keep a
            # malformed request or mismatched catalog module local to this run.
            self.get_logger().error(f"Phase 실행 요청 처리 실패: {exc}")
            if self._proc is None:
                self._running = None
            self._publish("failed", -1, f"Phase 실행 요청 실패: {exc}", phase=-1)

    def _start_phase_request(self, n):
        phase = self._phases.get(n)
        if phase is None:
            self._publish("failed", -1, f"등록되지 않은 Phase: {n}", phase=n)
            return
        available = getattr(
            phase,
            "available",
            os.path.isfile(str(phase.script_path)),
        )
        if not available:
            self._publish(
                "failed",
                -1,
                f"Phase {n} 코드 대기 중: {phase.script}",
                phase=n,
            )
            return

        if self._running is not None:
            if self._status["state"] == "awaiting_confirmation":
                self._republish()
            else:
                self._publish(
                    "running",
                    self._progress(),
                    f"Phase {self._running} 실행 중 — 끝난 뒤 실행하세요",
                    phase=self._running,
                )
            return

        # Claim the run here (on the ROS executor thread) so a rapid second
        # request is rejected by the guard above rather than racing the worker.
        self._running = n
        self._clear_mission_monitor()
        self._waiting_for_landing_phase = None
        self._clear_land_handoff()
        self._ready_for_land_seen = False
        self._land_confirm_sent = False
        self._publish_land_confirm(False)
        if getattr(phase, "start_action", "run_script") == "start_mission":
            self._start_uploaded_mission(n, phase)
            return

        threading.Thread(target=self._run_phase, args=(phase,), daemon=True).start()

    def _run_phase(self, phase):
        n = phase.phase_id
        self._last_wp = -1
        self._last_log = ""
        script = str(phase.script_path)

        if not os.path.exists(script):
            self._running = None
            self._pending_confirmation = None
            self._publish("failed", -1, f"{script} 없음", phase=n)
            return

        self._publish("running", -1, f"{phase.title} 시작", phase=n)
        try:
            self._proc = subprocess.Popen(
                [sys.executable, script], cwd=COMMAND_DIR,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
        except Exception as e:  # noqa: BLE001
            self._running = None
            self._pending_confirmation = None
            self._publish("failed", -1, f"Phase {n} 실행 실패: {e}", phase=n)
            return

        for line in self._proc.stdout:
            line = line.rstrip()
            if line:
                self._last_log = line

        rc = self._proc.wait()
        self._proc = None
        self._stop_failsafe("Phase 종료 — Failsafe 프로세스 정리됨")

        if self._aborting:
            # The nonzero exit was an intentional GCS take-over, not a failure.
            self._aborting = False
            self._running = None
            self._pending_confirmation = None
            self._waiting_for_landing_phase = None
            self._clear_land_handoff()
            self._publish("idle", -1, "제어권 회수됨 — HOLD(제자리 호버링)", phase=-1)
        elif rc == 0:
            confirm_after = getattr(phase, "confirm_after", "process_exit")
            confirmation = getattr(phase, "confirmation", "none")
            if confirmation == "land":
                # A ready_for_land signal may have already opened the dialog
                # while the phase process was still running. Do not reopen it
                # after the process exits, especially after Land was approved.
                if self._land_confirm_sent:
                    self._running = None
                    self._pending_confirmation = None
                    self._done.add(n)
                    self._publish(
                        "done",
                        1.0,
                        f"{phase.title} OFFBOARD 착륙 완료",
                        phase=n,
                    )
                elif self._land_handoff_phase != n:
                    self._clear_land_handoff()
                    ready_action = getattr(phase, "ready_action", "none")
                    if self._pending_confirmation != "land" and (
                        ready_action != "land" or self._ready_for_land_seen
                    ):
                        self._publish_phase_confirmation(phase)
            elif confirm_after == "landed":
                self._clear_land_handoff()
                if self._landed_state == 1:
                    self._publish_phase_confirmation(phase)
                else:
                    self._waiting_for_landing_phase = n
                    self._publish(
                        "running",
                        1.0,
                        f"{phase.title} 착륙 완료 확인 중",
                        phase=n,
                    )
            elif confirmation != "none":
                self._clear_land_handoff()
                self._publish_phase_confirmation(phase)
            else:
                self._clear_land_handoff()
                self._running = None
                self._done.add(n)
                self._publish("done", 1.0, f"{phase.title} 완료", phase=n)
        else:
            self._running = None
            self._pending_confirmation = None
            self._waiting_for_landing_phase = None
            self._clear_land_handoff()
            self._publish("failed", -1, f"Phase {n} 실패 (exit {rc}) — {self._last_log}", phase=n)

    def _start_phase_retry(self, phase):
        retry_script_path = getattr(phase, "retry_script_path", None)
        retry_script = getattr(phase, "retry_script", None)
        confirmation = getattr(phase, "confirmation", "none")
        if retry_script_path is None:
            self._publish(
                "awaiting_confirmation",
                1.0,
                f"{phase.title} 재시도 스크립트가 설정되지 않았습니다",
                phase=phase.phase_id,
                prompt=confirmation,
            )
            return False

        if not retry_script_path.is_file():
            self._publish(
                "awaiting_confirmation",
                1.0,
                f"{retry_script} 없음 — 파일 추가 후 Again을 다시 누르세요",
                phase=phase.phase_id,
                prompt=confirmation,
            )
            return False

        self._pending_confirmation = None
        self._waiting_for_landing_phase = None
        self._clear_land_handoff()
        self._last_log = ""
        self._publish(
            "running",
            0.0,
            f"{phase.title} 재시도 위치 복귀 중",
            phase=phase.phase_id,
        )
        threading.Thread(
            target=self._run_retry_script,
            args=(phase,),
            daemon=True,
        ).start()
        return True

    def _run_retry_script(self, phase):
        retry_script = getattr(phase, "retry_script", None)
        retry_script_path = getattr(phase, "retry_script_path", None)
        try:
            self._proc = subprocess.Popen(
                [sys.executable, str(retry_script_path)],
                cwd=COMMAND_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as exc:  # noqa: BLE001
            self._running = None
            self._publish(
                "failed",
                -1,
                f"{retry_script} 실행 실패: {exc}",
                phase=phase.phase_id,
            )
            return

        for line in self._proc.stdout:
            line = line.rstrip()
            if line:
                self._last_log = line

        rc = self._proc.wait()
        self._proc = None

        if self._aborting:
            self._aborting = False
            self._running = None
            self._publish(
                "idle",
                -1,
                "재시도 중단 — HOLD(제자리 호버링)",
                phase=-1,
            )
        elif rc == 0:
            self._publish(
                "running",
                0.0,
                f"{phase.title} 시작점 복귀 완료 — 착륙 재시도 시작",
                phase=phase.phase_id,
            )
            self._run_phase(phase)
        else:
            self._running = None
            self._publish(
                "failed",
                -1,
                f"{retry_script} 실패 (exit {rc}) — {self._last_log}",
                phase=phase.phase_id,
            )

    def _on_phase_response(self, msg):
        response = msg.data.strip().lower()
        phase_id = self._running
        prompt = self._pending_confirmation

        if (
            phase_id is None
            or prompt is None
            or self._status["state"] != "awaiting_confirmation"
        ):
            self.get_logger().warning(
                f"확인 대기 상태가 아닌데 응답을 받음: {response}"
            )
            self._republish()
            return

        if prompt == "ok":
            valid_responses = {"ok"}
        elif prompt == "land":
            valid_responses = {"ok", "no"}
        else:
            valid_responses = {"ok", "again"}
        if response not in valid_responses:
            self.get_logger().warning(
                f"Phase {phase_id}의 올바르지 않은 응답: {response}"
            )
            self._republish()
            return

        phase = self._phases.get(phase_id)
        title = phase.title if phase is not None else f"Phase {phase_id}"

        if response == "ok":
            if prompt == "land":
                self._ready_for_land_seen = True
                self._pending_confirmation = None
                phase = self._phases.get(phase_id)
                if phase is not None and getattr(phase, "ready_action", "none") == "land":
                    self._land_confirm_sent = True
                    self._publish_land_confirm(True)
                    self._publish(
                        "running",
                        self._progress(),
                        "Land 승인됨 — Phase OFFBOARD 착륙 진행",
                        phase=phase_id,
                    )
                else:
                    self._start_land_handoff(phase_id)
                return
            if (
                phase is not None
                and getattr(phase, "on_ok", "complete") == "start_mission"
            ):
                self._start_uploaded_mission(phase_id, phase)
                return

            self._running = None
            self._pending_confirmation = None
            self._done.add(phase_id)
            self._publish("done", 1.0, f"{title} 완료 확인됨", phase=phase_id)
        elif response == "no":
            self._ready_for_land_seen = True
            # The phase code remains in OFFBOARD hover while awaiting approval.
            # Keep the decision pending so the operator can approve Land later
            # from the footer without re-running the alignment code.
            self._publish(
                "awaiting_confirmation",
                self._progress(),
                f"{title} Land 취소 — OFFBOARD 호버링 유지",
                phase=phase_id,
                prompt="land",
            )
        else:
            if (
                phase is not None
                and getattr(phase, "retry_script", None) is not None
            ):
                self._start_phase_retry(phase)
                return

            self._running = None
            self._pending_confirmation = None
            self._publish("idle", -1, f"{title} 재시도 선택됨", phase=-1)

    # --- GCS take-over (abort) ----------------------------------------------
    def _on_abort(self, _msg):
        """Operator took control from QGC: kill the running phase and hover.

        Terminates the phase subprocess (its stdout loop ends, `_run_phase`
        finalizes as "idle" because `_aborting` is set) and switches PX4 to HOLD
        so the vehicle hovers in place and waits for the operator's commands.
        """
        self._stop_failsafe("제어권 회수 — Failsafe 중단")
        proc = self._proc
        if self._running is not None and proc is not None:
            self._aborting = True
            self.get_logger().warning(
                f"GCS take-over: aborting phase {self._running}")
            self._publish("running", self._progress(),
                          "제어권 회수 중 — 임무 중단", phase=self._running)
            proc.terminate()   # SIGTERM; _run_phase's stdout loop unblocks on exit
            threading.Thread(target=self._kill_after, args=(proc, 2.0), daemon=True).start()
        elif self._running is not None:
            phase_id = self._running
            self._running = None
            self._pending_confirmation = None
            self._clear_mission_monitor()
            self._waiting_for_landing_phase = None
            self._clear_land_handoff()
            self.get_logger().info(
                f"GCS take-over: cancelling Phase {phase_id} workflow"
            )
            self._publish(
                "idle",
                -1,
                f"Phase {phase_id} 확인 취소 — HOLD(제자리 호버링)",
                phase=-1,
            )
        else:
            self.get_logger().info("GCS take-over: no phase running, switching to HOLD")
            self._publish("idle", -1, "제어권 회수 — HOLD(제자리 호버링)", phase=-1)

        self._set_hold_mode()

    @staticmethod
    def _kill_after(proc, timeout):
        """SIGKILL fallback if a phase ignores SIGTERM and keeps running.

        Polls (rather than wait()) so it never races the wait() call in the
        `_run_phase` worker thread on the same Popen object.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return
            time.sleep(0.1)
        proc.kill()

    def _set_hold_mode(self):
        """Ask PX4 (via MAVROS) to switch to HOLD so the vehicle hovers in place."""
        if not self.set_mode_cli.service_is_ready():
            self.get_logger().warning(
                "/mavros/set_mode not available — cannot force HOLD "
                "(is MAVROS running?)")
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = PX4_HOLD_MODE
        self.set_mode_cli.call_async(req)   # fire-and-forget; do not block the executor

    def _tick(self):
        self._update_action_processes()
        # While a phase runs, refresh the live section description; otherwise
        # just republish the last status so late subscribers stay in sync.
        if self._mission_monitor_phase is not None:
            self._tick_mission_monitor()
        elif self._land_handoff_phase is not None:
            self._tick_land_handoff()
        elif self._waiting_for_landing_phase is not None:
            self._publish(
                "running",
                1.0,
                f"Phase {self._waiting_for_landing_phase} 착륙 완료 확인 중",
                phase=self._waiting_for_landing_phase,
            )
        elif self._running is not None and self._status["state"] == "running":
            sec = self._section_desc()
            text = sec if sec else (self._last_log or f"Phase {self._running} 실행 중")
            if self._running in (2, 4):
                text = f"Vision Based Land — {self._last_log or '영상 인식 · Position 정렬 중'}"
            self._publish("running", self._progress(), text, phase=self._running)
        else:
            self._republish()

    # --- status publishing ---------------------------------------------------
    def _publish(self, state, progress, msg, phase=None, prompt=""):
        self._status = {
            "phase": self._running if phase is None else phase,
            "state": state,
            "msg": msg,
            "progress": float(progress),
            "done": sorted(self._done),
            "prompt": prompt,
        }
        self._republish()

    def _republish(self):
        if getattr(self, "_shutting_down", False):
            return

        payload = dict(self._status)
        payload["actions"] = self._action_status()
        s = String()
        s.data = json.dumps(payload, ensure_ascii=False)
        try:
            self.status_pub.publish(s)
        except Exception:  # noqa: BLE001
            # rclpy can invalidate a publisher between the check above and the
            # publish call while shutdown races a payload worker thread.
            if getattr(self, "_shutting_down", False) or not rclpy.ok():
                return
            raise


def _acquire_singleton_lock():
    """Hold an exclusive lock so only ONE orchestrator ever runs.

    Two orchestrators would both publish command/status with divergent state
    (one thinks a phase is done, the other doesn't), which makes the QGC panel
    flicker between phases. Return the open file (keep it referenced to hold the
    lock for the process lifetime) or None if another instance already has it.
    """
    lock_path = os.path.join(tempfile.gettempdir(), "phase_orchestrator.lock")
    lock_file = open(lock_path, "w")  # noqa: SIM115 (must outlive this function)
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        lock_file.close()
        return None
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    return lock_file


def main():
    lock = _acquire_singleton_lock()
    if lock is None:
        print("phase_orchestrator already running — this instance is exiting "
              "(avoids duplicate command/status publishers)", flush=True)
        return

    rclpy.init()
    node = PhaseOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Payload and phase monitor threads may still be unwinding. Prevent
        # them from publishing after destroy_node() invalidates ROS handles.
        node._shutting_down = True
        if node._proc is not None:
            node._proc.terminate()
        if node._camera_proc is not None:
            node._camera_proc.terminate()
        if node._gripper_proc is not None:
            node._gripper_proc.terminate()
        if node._failsafe_proc is not None:
            node._failsafe_proc.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
