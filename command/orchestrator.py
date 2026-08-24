#!/usr/bin/env python3
"""Onboard phase orchestrator for the custom QGC mission panel.

QGC publishes a phase number on `command/run_phase` (std_msgs/Int32) when the
operator clicks a phase button. This node looks up that id in its local dynamic
catalog, runs the corresponding local Python script independently, and streams
live progress back on `command/status` (std_msgs/String, JSON payload).

Independent payload controls arrive on `command/run_action` (std_msgs/String):
`camera:on`, `camera:off`, `gripper:open`, or `gripper:close`.

The phase catalog is published on `command/catalog` (std_msgs/String, JSON).
`phases.json` supplies display metadata and may map ids to any Python script
below this directory. Unlisted `phaseN.py` files are discovered automatically.
The catalog is reloaded at runtime, so adding a valid local phase does not
require restarting this node.

Status JSON:
    {"phase": int,
     "state": "idle|running|awaiting_confirmation|done|failed",
     "msg": str, "progress": float(-1..1), "done": [completed phase ids],
     "prompt": ""|"ok"|"ok_again",
     "actions": {camera/gripper availability and live state}}

Catalog JSON:
    {"version": 1, "phases": [
        {"id": int, "title": str, "desc": str, "script": str,
         "independent": true, "confirmation": "none"|"ok"|"ok_again",
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
from mavros_msgs.srv import SetMode

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
MISSION_MODE_REQUEST_INTERVAL_SEC = 1.0
MISSION_MODE_ENTRY_TIMEOUT_SEC = 15.0
LAND_MODE_REQUEST_INTERVAL_SEC = 1.0
LAND_MODE_ENTRY_TIMEOUT_SEC = 15.0
MAV_CMD_NAV_WAYPOINT = 16

CAMERA_SCRIPT = "cam.py"
GRIPPER_SCRIPTS = {
    "open": "gripper_open.py",
    "close": "gripper_close.py",
}


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
        self._mission_monitor_phase = None
        self._mission_completion_seq = -1
        self._mission_mode_confirmed = False
        self._mission_mode_request_started = 0.0
        self._mission_mode_last_request = 0.0
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
        self._action_msg = ""
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
            self._done.add(phase_id)
            self._running = None
            self._clear_mission_monitor()
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
        if not msg.data or self._running is None:
            return

        phase = self._phases.get(self._running)
        if phase is None or phase.ready_action != "land":
            return
        if self._land_handoff_phase == self._running:
            return

        self._start_land_handoff(self._running)

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

    def _start_uploaded_mission(self, phase_id, phase):
        if self._uploaded_mission_last_wp < 0:
            self._publish(
                "awaiting_confirmation",
                1.0,
                "업로드된 일반 Waypoint 미션이 없습니다. 미션 업로드 후 OK를 다시 누르세요.",
                phase=phase_id,
                prompt=self._pending_confirmation,
            )
            return False

        if not self.set_mode_cli.service_is_ready():
            self._publish(
                "awaiting_confirmation",
                1.0,
                "/mavros/set_mode 연결 대기 중입니다. 연결 확인 후 OK를 다시 누르세요.",
                phase=phase_id,
                prompt=self._pending_confirmation,
            )
            return False

        now = time.monotonic()
        self._pending_confirmation = None
        self._mission_monitor_phase = phase_id
        self._mission_completion_seq = self._uploaded_mission_last_wp
        self._mission_mode_confirmed = False
        self._mission_mode_request_started = now
        self._mission_mode_last_request = 0.0
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
            "착륙 위치 정렬 완료 — AUTO.LAND 전환 요청",
            phase=phase_id,
        )

    def _tick_land_handoff(self):
        phase_id = self._land_handoff_phase
        if phase_id is None:
            return

        now = time.monotonic()
        if self._mode == PX4_LAND_MODE:
            self._publish(
                "running",
                self._progress(),
                "AUTO.LAND 진입 완료 — 착륙 중",
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
        self._pending_confirmation = phase.confirmation
        if phase.confirm_after == "landed":
            confirmation_message = (
                f"{phase.title} 착륙 완료 — 위치 확인 후 OK 또는 Again을 선택하세요"
            )
        elif phase.on_ok == "start_mission":
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
            prompt=phase.confirmation,
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
        if self._gripper_busy:
            self._action_msg = "Gripper 동작 중 — 완료 후 다시 시도하세요"
            self._republish()
            return

        script_name = GRIPPER_SCRIPTS[action]
        script = self._action_script_path(script_name)
        if not os.path.isfile(script):
            self._action_msg = f"{script_name} 없음"
            self.get_logger().error(self._action_msg)
            self._republish()
            return

        self._gripper_busy = True
        self._action_msg = f"Gripper {action.title()} 실행 중"
        self._republish()
        threading.Thread(
            target=self._run_gripper,
            args=(action, script),
            daemon=True,
        ).start()

    def _run_gripper(self, action, script):
        try:
            self._gripper_proc = subprocess.Popen(
                [sys.executable, script],
                cwd=COMMAND_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            output, _ = self._gripper_proc.communicate()
            rc = self._gripper_proc.returncode
        except Exception as exc:  # noqa: BLE001
            self._action_msg = f"Gripper {action.title()} 실행 실패: {exc}"
            self.get_logger().error(self._action_msg)
        else:
            if rc == 0:
                self._gripper_state = action
                self._action_msg = f"Gripper {action.title()} 완료"
                self.get_logger().info(self._action_msg)
            else:
                detail = output.strip().splitlines()[-1] if output.strip() else ""
                self._action_msg = (
                    f"Gripper {action.title()} 실패 (exit {rc})"
                    + (f" — {detail}" if detail else "")
                )
                self.get_logger().error(self._action_msg)
        finally:
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
        n = int(msg.data)

        phase = self._phases.get(n)
        if phase is None:
            self._publish("failed", -1, f"등록되지 않은 Phase: {n}", phase=n)
            return
        if not phase.available:
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

        if self._aborting:
            # The nonzero exit was an intentional GCS take-over, not a failure.
            self._aborting = False
            self._running = None
            self._pending_confirmation = None
            self._waiting_for_landing_phase = None
            self._clear_land_handoff()
            self._publish("idle", -1, "제어권 회수됨 — HOLD(제자리 호버링)", phase=-1)
        elif rc == 0:
            self._clear_land_handoff()
            if phase.confirm_after == "landed":
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
            elif phase.confirmation != "none":
                self._publish_phase_confirmation(phase)
            else:
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
        if phase.retry_script_path is None:
            self._publish(
                "awaiting_confirmation",
                1.0,
                f"{phase.title} 재시도 스크립트가 설정되지 않았습니다",
                phase=phase.phase_id,
                prompt=phase.confirmation,
            )
            return False

        if not phase.retry_script_path.is_file():
            self._publish(
                "awaiting_confirmation",
                1.0,
                f"{phase.retry_script} 없음 — 파일 추가 후 Again을 다시 누르세요",
                phase=phase.phase_id,
                prompt=phase.confirmation,
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
        try:
            self._proc = subprocess.Popen(
                [sys.executable, str(phase.retry_script_path)],
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
                f"{phase.retry_script} 실행 실패: {exc}",
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
                f"{phase.retry_script} 실패 (exit {rc}) — {self._last_log}",
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

        valid_responses = {"ok"} if prompt == "ok" else {"ok", "again"}
        if response not in valid_responses:
            self.get_logger().warning(
                f"Phase {phase_id}의 올바르지 않은 응답: {response}"
            )
            self._republish()
            return

        phase = self._phases.get(phase_id)
        title = phase.title if phase is not None else f"Phase {phase_id}"

        if response == "ok":
            if phase is not None and phase.on_ok == "start_mission":
                self._start_uploaded_mission(phase_id, phase)
                return

            self._running = None
            self._pending_confirmation = None
            self._done.add(phase_id)
            self._publish("done", 1.0, f"{title} 완료 확인됨", phase=phase_id)
        else:
            if phase is not None and phase.retry_script is not None:
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
        payload = dict(self._status)
        payload["actions"] = self._action_status()
        s = String()
        s.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(s)


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
        if node._proc is not None:
            node._proc.terminate()
        if node._camera_proc is not None:
            node._camera_proc.terminate()
        if node._gripper_proc is not None:
            node._gripper_proc.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
