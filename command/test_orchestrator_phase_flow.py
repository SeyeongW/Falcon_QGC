"""Unit tests for phase completion and operator confirmation."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch


COMMAND_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(COMMAND_DIR))

import orchestrator  # noqa: E402


class SuccessfulProcess:
    def __init__(self):
        self.stdout = ["phase complete\n"]

    @staticmethod
    def wait():
        return 0


class FakeSetModeClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)


class OrchestratorPhaseFlowTest(unittest.TestCase):
    def setUp(self):
        self.node = orchestrator.PhaseOrchestrator.__new__(
            orchestrator.PhaseOrchestrator
        )
        self.node._running = 0
        self.node._pending_confirmation = None
        self.node._ready_for_land_seen = False
        self.node._land_confirm_sent = False
        self.node.land_confirm_pub = Mock()
        self.node._proc = None
        self.node._last_wp = -1
        self.node._uploaded_mission_last_wp = -1
        self.node._mode = "AUTO.LOITER"
        self.node._vtol = 3
        self.node._landed_state = 2
        self.node._last_log = ""
        self.node._done = set()
        self.node._aborting = False
        self.node._status = {"state": "idle"}
        self.node._phases = {
            0: SimpleNamespace(
                phase_id=0,
                title="사전 점검",
                on_ok="complete",
                confirm_after="process_exit",
                ready_action="none",
                retry_script=None,
                retry_script_path=None,
            ),
            1: SimpleNamespace(
                phase_id=1,
                title="이륙 · 정찰",
                script="phase1.py",
                script_path=COMMAND_DIR / "phase1.py",
                start_action="start_mission",
                confirmation="ok",
                on_ok="complete",
                confirm_after="process_exit",
                ready_action="none",
                retry_script=None,
                retry_script_path=None,
                available=True,
            ),
            2: SimpleNamespace(
                phase_id=2,
                title="Vision Based Land",
                on_ok="complete",
                confirmation="land",
                confirm_after="process_exit",
                ready_action="land",
                retry_script=None,
                retry_script_path=None,
            ),
            3: SimpleNamespace(
                phase_id=3,
                title="구조자 이송 · 2차 미션",
                script="phase3.py",
                script_path=COMMAND_DIR / "phase3.py",
                start_action="start_mission",
                confirmation="ok",
                on_ok="complete",
                confirm_after="process_exit",
                ready_action="none",
                retry_script=None,
                retry_script_path=None,
                available=True,
            ),
        }
        self.node._mission_monitor_phase = None
        self.node._mission_completion_seq = -1
        self.node._mission_mode_confirmed = False
        self.node._mission_mode_request_started = 0.0
        self.node._mission_mode_last_request = 0.0
        self.node._waiting_for_landing_phase = None
        self.node._land_handoff_phase = None
        self.node._land_mode_request_started = 0.0
        self.node._land_mode_last_request = 0.0
        self.node._failsafe_proc = None
        self.node.set_mode_cli = FakeSetModeClient()
        self.node.mission_start_cli = FakeSetModeClient()
        self.node._publish = Mock()
        self.node._republish = Mock()
        self.node.get_logger = Mock(return_value=Mock())

    def test_ready_for_land_opens_confirmation_before_phase_exit(self):
        self.node._running = 2
        self.node._status = {"state": "running"}

        self.node._on_ready_for_land(SimpleNamespace(data=True))

        self.assertEqual(self.node._pending_confirmation, "land")
        self.assertTrue(self.node._ready_for_land_seen)
        self.node._publish.assert_called_once()
        self.assertEqual(self.node._publish.call_args.kwargs["prompt"], "land")

    def test_ready_for_land_false_does_not_open_confirmation(self):
        self.node._running = 2
        self.node._status = {"state": "running"}

        self.node._on_ready_for_land(SimpleNamespace(data=False))

        self.assertIsNone(self.node._pending_confirmation)
        self.node._publish.assert_not_called()

    def test_successful_phase_waits_for_configured_confirmation(self):
        phase = SimpleNamespace(
            phase_id=0,
            title="사전 점검",
            script_path=COMMAND_DIR / "phase0.py",
            confirmation="ok",
            on_ok="complete",
            confirm_after="process_exit",
        )

        with patch.object(
            orchestrator.subprocess,
            "Popen",
            return_value=SuccessfulProcess(),
        ):
            self.node._run_phase(phase)

        self.assertEqual(self.node._running, 0)
        self.assertEqual(self.node._pending_confirmation, "ok")
        self.assertNotIn(0, self.node._done)
        self.node._publish.assert_called_with(
            "awaiting_confirmation",
            1.0,
            "사전 점검 정상 종료 — 사용자 확인 대기",
            phase=0,
            prompt="ok",
        )

    def test_legacy_phase_definition_without_available_starts(self):
        phase = SimpleNamespace(
            phase_id=4,
            title="Legacy Phase",
            script="phase0.py",
            script_path=COMMAND_DIR / "phase0.py",
        )
        self.node._phases = {4: phase}
        self.node._running = None

        with patch.object(orchestrator.threading, "Thread") as thread:
            self.node._on_run_phase(SimpleNamespace(data=4))

        self.assertEqual(self.node._running, 4)
        thread.assert_called_once_with(
            target=self.node._run_phase,
            args=(phase,),
            daemon=True,
        )
        thread.return_value.start.assert_called_once_with()

    def test_completed_phase_can_be_started_again(self):
        phase = SimpleNamespace(
            phase_id=4,
            title="Repeatable Phase",
            script="phase0.py",
            script_path=COMMAND_DIR / "phase0.py",
            available=True,
        )
        self.node._phases = {4: phase}
        self.node._done = {4}
        self.node._running = None

        with patch.object(orchestrator.threading, "Thread") as thread:
            self.node._on_run_phase(SimpleNamespace(data=4))

        self.assertEqual(self.node._running, 4)
        thread.return_value.start.assert_called_once_with()

    def test_legacy_phase_definition_uses_basic_completion_defaults(self):
        phase = SimpleNamespace(
            phase_id=4,
            title="Legacy Phase",
            script_path=COMMAND_DIR / "phase0.py",
        )
        self.node._running = 4

        with patch.object(
            orchestrator.subprocess,
            "Popen",
            return_value=SuccessfulProcess(),
        ):
            self.node._run_phase(phase)

        self.assertIsNone(self.node._running)
        self.assertIn(4, self.node._done)
        self.node._publish.assert_called_with(
            "done",
            1.0,
            "Legacy Phase 완료",
            phase=4,
        )

    def test_phase_request_exception_does_not_escape_callback(self):
        self.node._running = None

        with patch.object(
            self.node,
            "_start_phase_request",
            side_effect=AttributeError("catalog mismatch"),
        ):
            self.node._on_run_phase(SimpleNamespace(data=0))

        self.assertIsNone(self.node._running)
        self.node._publish.assert_called_once_with(
            "failed",
            -1,
            "Phase 실행 요청 실패: catalog mismatch",
            phase=-1,
        )

    def test_republish_is_skipped_during_shutdown(self):
        self.node._shutting_down = True
        self.node.status_pub = Mock()

        orchestrator.PhaseOrchestrator._republish(self.node)

        self.node.status_pub.publish.assert_not_called()

    def test_ok_response_marks_pending_phase_done(self):
        self.node._pending_confirmation = "ok"
        self.node._status = {"state": "awaiting_confirmation"}

        self.node._on_phase_response(SimpleNamespace(data="ok"))

        self.assertIsNone(self.node._running)
        self.assertIsNone(self.node._pending_confirmation)
        self.assertIn(0, self.node._done)
        self.node._publish.assert_called_once_with(
            "done",
            1.0,
            "사전 점검 완료 확인됨",
            phase=0,
        )

    def test_unexpected_response_does_not_complete_phase(self):
        self.node._pending_confirmation = "ok"
        self.node._status = {"state": "awaiting_confirmation"}

        self.node._on_phase_response(SimpleNamespace(data="again"))

        self.assertEqual(self.node._running, 0)
        self.assertEqual(self.node._pending_confirmation, "ok")
        self.assertNotIn(0, self.node._done)
        self.node._publish.assert_not_called()
        self.node._republish.assert_called_once()

    def test_phase_one_and_three_requests_start_mission_immediately(self):
        self.node._uploaded_mission_last_wp = 4

        for phase_id in (1, 3):
            with self.subTest(phase_id=phase_id):
                self.node._running = None
                self.node._pending_confirmation = None
                self.node._mission_monitor_phase = None
                self.node.set_mode_cli.requests.clear()

                self.node._start_phase_request(phase_id)

                self.assertEqual(self.node._running, phase_id)
                self.assertIsNone(self.node._pending_confirmation)
                self.assertEqual(self.node._mission_monitor_phase, phase_id)
                self.assertEqual(self.node._mission_completion_seq, 4)
                self.assertEqual(len(self.node.set_mode_cli.requests), 1)
                self.assertEqual(
                    self.node.set_mode_cli.requests[0].custom_mode,
                    orchestrator.PX4_MISSION_MODE,
                )

    def test_direct_mission_start_fails_cleanly_when_no_mission_is_uploaded(self):
        self.node._running = None

        self.node._start_phase_request(1)

        self.assertIsNone(self.node._running)
        self.assertIsNone(self.node._pending_confirmation)
        self.assertIsNone(self.node._mission_monitor_phase)
        self.node._publish.assert_called_once_with(
            "failed",
            -1,
            "업로드된 일반 Waypoint 미션이 없습니다. 미션 업로드 후 Phase를 다시 실행하세요.",
            phase=1,
        )

    def test_last_uploaded_waypoint_waits_for_phase_one_ok(self):
        self.node._running = 1
        self.node._mission_monitor_phase = 1
        self.node._mission_completion_seq = 4

        self.node._on_wp(SimpleNamespace(wp_seq=4))

        self.assertEqual(self.node._running, 1)
        self.assertIsNone(self.node._mission_monitor_phase)
        self.assertEqual(self.node._pending_confirmation, "ok")
        self.assertNotIn(1, self.node._done)
        self.assertEqual(
            self.node.set_mode_cli.requests[-1].custom_mode,
            orchestrator.PX4_HOLD_MODE,
        )
        self.node._publish.assert_called_once_with(
            "awaiting_confirmation",
            1.0,
            "이륙 · 정찰 Mission 종료 — 확인 후 OK를 눌러주세요",
            phase=1,
            prompt="ok",
        )

    def test_ok_after_direct_mission_marks_phase_done_without_restarting(self):
        self.node._running = 1
        self.node._pending_confirmation = "ok"
        self.node._status = {"state": "awaiting_confirmation"}

        self.node._on_phase_response(SimpleNamespace(data="ok"))

        self.assertIsNone(self.node._running)
        self.assertIsNone(self.node._pending_confirmation)
        self.assertIn(1, self.node._done)
        self.assertEqual(self.node.set_mode_cli.requests, [])
        self.node._publish.assert_called_once_with(
            "done",
            1.0,
            "이륙 · 정찰 완료 확인됨",
            phase=1,
        )

    def test_waypoint_catalog_uses_last_navigation_waypoint(self):
        message = SimpleNamespace(
            waypoints=[
                SimpleNamespace(command=16),
                SimpleNamespace(command=16),
                SimpleNamespace(command=3000),
                SimpleNamespace(command=16),
                SimpleNamespace(command=85),
            ]
        )

        self.node._on_mission_waypoints(message)

        self.assertEqual(self.node._uploaded_mission_last_wp, 3)

    def test_mission_mode_also_sends_explicit_mission_start(self):
        self.node._mission_monitor_phase = 1
        self.node._mission_completion_seq = 3
        self.node._mission_mode_request_started = 0.0
        self.node._mission_mode_last_request = 0.0
        self.node._mode = orchestrator.PX4_MISSION_MODE
        self.node._mission_start_sent = False

        self.node._tick_mission_monitor()

        self.assertTrue(self.node._mission_start_sent)
        self.assertEqual(len(self.node.mission_start_cli.requests), 1)
        self.assertEqual(
            self.node.mission_start_cli.requests[0].command,
            orchestrator.MAV_CMD_MISSION_START,
        )

    def test_successful_phase_two_exit_waits_for_land_confirmation(self):
        phase = self.node._phases[2]
        phase.script_path = COMMAND_DIR / "phase0.py"
        self.node._running = 2
        self.node._ready_for_land_seen = True

        with patch.object(
            orchestrator.subprocess,
            "Popen",
            return_value=SuccessfulProcess(),
        ):
            self.node._run_phase(phase)

        self.assertIsNone(self.node._land_handoff_phase)
        self.assertEqual(self.node._pending_confirmation, "land")
        self.assertEqual(self.node.set_mode_cli.requests, [])
        self.node._publish.assert_called_with(
            "awaiting_confirmation",
            1.0,
            "정렬이 완료 됐습니다. Land 하시겠습니까?",
            phase=2,
            prompt="land",
        )

    def test_no_response_keeps_pending_offboard_hover_without_land_request(self):
        self.node._running = 2
        self.node._pending_confirmation = "land"
        self.node._status = {"state": "awaiting_confirmation"}

        self.node._on_phase_response(SimpleNamespace(data="no"))

        self.assertEqual(self.node._pending_confirmation, "land")
        self.assertEqual(self.node._running, 2)
        self.assertEqual(self.node.set_mode_cli.requests, [])
        self.assertNotIn(2, self.node._done)
        self.node._publish.assert_called_once_with(
            "awaiting_confirmation",
            -1.0,
            "Vision Based Land Land 취소 — OFFBOARD 호버링 유지",
            phase=2,
            prompt="land",
        )

    def test_ok_response_starts_phase_offboard_landing(self):
        self.node._running = 2
        self.node._pending_confirmation = "land"
        self.node._status = {"state": "awaiting_confirmation"}

        self.node._on_phase_response(SimpleNamespace(data="ok"))

        self.assertEqual(self.node._running, 2)
        self.assertIsNone(self.node._pending_confirmation)
        self.assertIsNone(self.node._land_handoff_phase)
        self.assertTrue(self.node._land_confirm_sent)
        self.node.land_confirm_pub.publish.assert_called_once()
        self.assertTrue(self.node.land_confirm_pub.publish.call_args.args[0].data)
        self.node._publish.assert_called_once_with(
            "running",
            -1.0,
            "Land 승인됨 — Phase OFFBOARD 착륙 진행",
            phase=2,
        )

    def test_auto_land_mode_confirmation_marks_phase_two_done(self):
        self.node._running = 2
        self.node._land_handoff_phase = 2
        self.node._mode = orchestrator.PX4_LAND_MODE

        self.node._tick_land_handoff()

        self.assertIsNone(self.node._running)
        self.assertIsNone(self.node._land_handoff_phase)
        self.assertIn(2, self.node._done)
        self.node._publish.assert_called_once_with(
            "done",
            1.0,
            "Vision Based Land 완료 — AUTO.LAND 인가됨",
            phase=2,
        )


if __name__ == "__main__":
    unittest.main()
