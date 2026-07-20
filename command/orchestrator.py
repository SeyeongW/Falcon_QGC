#!/usr/bin/env python3
"""Phase orchestrator for the custom QGC mission panel.

QGC publishes a phase number on `command/run_phase` (std_msgs/Int32) when the
operator clicks a phase button. This node then runs the matching `phaseN.py`
script (sequentially — phase N only runs once N-1 has completed) and streams
live progress back on `command/status` (std_msgs/String, JSON payload) so QGC can
show "running / done / failed" and a human-readable description of the current
section (e.g. "WP2 이동 중", "고정익 천이 중").

Status JSON:
    {"phase": int, "state": "idle|running|done|failed",
     "msg": str, "progress": float(-1..1), "done": [completed phase ids]}

Run:  python3 command/orchestrator.py   (needs MAVROS running for section text)
"""
import fcntl
import json
import os
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from std_msgs.msg import Empty, Int32, String
from mavros_msgs.msg import WaypointReached, ExtendedState, State
from mavros_msgs.srv import SetMode


# Directory that holds this orchestrator and the phaseN.py scripts. Derived from
# this file's own location so the folder can live anywhere (any user's home, /opt,
# a USB mount, a colcon workspace) and still find the phases — no hardcoded path.
COMMAND_DIR = os.path.dirname(os.path.abspath(__file__))
NUM_PHASES = 4  # phase0 .. phase3 (phase{N}.py must exist for each)

# VTOL states (mavros ExtendedState.vtol_state)
VTOL_TRANS_TO_FW = 1
VTOL_TRANS_TO_MC = 2
VTOL_MC = 3
VTOL_FW = 4

# PX4 flight mode used to hand control back to the GCS on abort: HOLD makes the
# vehicle hover in place (multicopter) / loiter (fixed wing) and wait for the
# operator's next command from QGC.
PX4_HOLD_MODE = "AUTO.LOITER"


class PhaseOrchestrator(Node):
    def __init__(self):
        super().__init__("phase_orchestrator")

        self.status_pub = self.create_publisher(String, "command/status", 10)
        self.create_subscription(Int32, "command/run_phase", self._on_run_phase, 10)
        # GCS take-over: abort the running phase and hand control back (HOLD).
        self.create_subscription(Empty, "command/abort", self._on_abort, 10)
        self.set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        # Live vehicle state used to describe the current mission section.
        self._last_wp = -1
        self._vtol = 0
        self._mode = ""
        self._armed = False

        rel = QoSProfile(depth=10)
        rel.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self._on_wp, 10)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_ext, qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state", self._on_state, rel)

        # Orchestration state.
        self._done = set()      # completed phase ids
        self._running = None    # currently running phase id (or None)
        self._proc = None
        self._last_log = ""     # latest stdout line from the running phase
        self._aborting = False  # True while a GCS take-over is tearing a phase down

        # Latest status payload, republished every tick so a late subscriber
        # (e.g. QGC connecting after boot) always sees the current state.
        self._status = {"phase": -1, "state": "idle", "msg": "대기 중",
                        "progress": -1.0, "done": []}

        self.create_timer(0.5, self._tick)  # push live status ~2 Hz
        self._publish("idle", -1, "대기 중", phase=-1)
        self.get_logger().info(
            f"orchestrator up (phases 0..{NUM_PHASES - 1}, dir={COMMAND_DIR})")

    # --- vehicle state callbacks --------------------------------------------
    def _on_wp(self, m):
        self._last_wp = m.wp_seq

    def _on_ext(self, m):
        self._vtol = m.vtol_state

    def _on_state(self, m):
        self._mode = m.mode
        self._armed = m.armed

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
        # Rough progress for the phase-1 mission (8 items); unknown otherwise.
        if self._last_wp >= 0:
            return min(1.0, max(0.0, self._last_wp / 8.0))
        return -1.0

    # --- run-phase handling --------------------------------------------------
    def _on_run_phase(self, msg):
        n = int(msg.data)

        if n < 0 or n >= NUM_PHASES:
            self._publish("failed", -1, f"잘못된 phase 번호: {n}", phase=n)
            return

        if self._running is not None:
            self._publish("running", self._progress(),
                          f"Phase {self._running} 실행 중 — 끝난 뒤 실행하세요", phase=self._running)
            return

        # Sequential gate: phase N needs N-1 done (phase 0 always allowed).
        if n != 0 and (n - 1) not in self._done:
            self._publish("failed", -1, f"이전 Phase {n - 1} 미완료 — 순서대로 실행하세요", phase=n)
            return

        # Claim the run here (on the ROS executor thread) so a rapid second
        # request is rejected by the guard above rather than racing the worker.
        self._running = n
        threading.Thread(target=self._run_phase, args=(n,), daemon=True).start()

    def _run_phase(self, n):
        self._last_wp = -1
        self._last_log = ""
        script = os.path.join(COMMAND_DIR, f"phase{n}.py")

        if not os.path.exists(script):
            self._running = None
            self._publish("failed", -1, f"{script} 없음", phase=n)
            return

        self._publish("running", -1, f"Phase {n} 시작", phase=n)
        try:
            self._proc = subprocess.Popen(
                ["python3", script], cwd=COMMAND_DIR,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
        except Exception as e:  # noqa: BLE001
            self._running = None
            self._publish("failed", -1, f"Phase {n} 실행 실패: {e}", phase=n)
            return

        for line in self._proc.stdout:
            line = line.rstrip()
            if line:
                self._last_log = line

        rc = self._proc.wait()
        self._proc = None
        self._running = None

        if self._aborting:
            # The nonzero exit was an intentional GCS take-over, not a failure.
            self._aborting = False
            self._publish("idle", -1, "제어권 회수됨 — HOLD(제자리 호버링)", phase=-1)
        elif rc == 0:
            self._done.add(n)
            self._publish("done", 1.0, f"Phase {n} 완료", phase=n)
        else:
            self._publish("failed", -1, f"Phase {n} 실패 (exit {rc}) — {self._last_log}", phase=n)

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
        # While a phase runs, refresh the live section description; otherwise
        # just republish the last status so late subscribers stay in sync.
        if self._running is not None:
            sec = self._section_desc()
            text = sec if sec else (self._last_log or f"Phase {self._running} 실행 중")
            self._publish("running", self._progress(), text, phase=self._running)
        else:
            self._republish()

    # --- status publishing ---------------------------------------------------
    def _publish(self, state, progress, msg, phase=None):
        self._status = {
            "phase": self._running if phase is None else phase,
            "state": state,
            "msg": msg,
            "progress": float(progress),
            "done": sorted(self._done),
        }
        self._republish()

    def _republish(self):
        s = String()
        s.data = json.dumps(self._status, ensure_ascii=False)
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
