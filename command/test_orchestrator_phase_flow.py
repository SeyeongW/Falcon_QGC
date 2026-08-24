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
                on_ok="start_mission",
                confirm_after="process_exit",
                ready_action="none",
                retry_script=None,
                retry_script_path=None,
            ),
            2: SimpleNamespace(
                phase_id=2,
                title="대상 탐지 · 접근",
                on_ok="complete",
                confirmation="ok_again",
                confirm_after="landed",
                ready_action="land",
                retry_script="failsafe.py",
                retry_script_path=COMMAND_DIR / "failsafe.py",
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
        self.node.set_mode_cli = FakeSetModeClient()
        self.node._publish = Mock()
        self.node._republish = Mock()
        self.node.get_logger = Mock(return_value=Mock())

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

    def test_phase_one_ok_starts_uploaded_mission_monitor(self):
        self.node._running = 1
        self.node._pending_confirmation = "ok"
        self.node._status = {"state": "awaiting_confirmation"}
        self.node._uploaded_mission_last_wp = 4

        self.node._on_phase_response(SimpleNamespace(data="ok"))

        self.assertEqual(self.node._running, 1)
        self.assertIsNone(self.node._pending_confirmation)
        self.assertEqual(self.node._mission_monitor_phase, 1)
        self.assertEqual(self.node._mission_completion_seq, 4)
        self.assertEqual(len(self.node.set_mode_cli.requests), 1)
        self.assertEqual(
            self.node.set_mode_cli.requests[0].custom_mode,
            orchestrator.PX4_MISSION_MODE,
        )

    def test_phase_one_ok_waits_when_no_mission_is_uploaded(self):
        self.node._running = 1
        self.node._pending_confirmation = "ok"
        self.node._status = {"state": "awaiting_confirmation"}

        self.node._on_phase_response(SimpleNamespace(data="ok"))

        self.assertEqual(self.node._running, 1)
        self.assertEqual(self.node._pending_confirmation, "ok")
        self.assertIsNone(self.node._mission_monitor_phase)
        self.node._publish.assert_called_once_with(
            "awaiting_confirmation",
            1.0,
            "업로드된 일반 Waypoint 미션이 없습니다. 미션 업로드 후 OK를 다시 누르세요.",
            phase=1,
            prompt="ok",
        )

    def test_last_uploaded_waypoint_completes_phase_one(self):
        self.node._running = 1
        self.node._mission_monitor_phase = 1
        self.node._mission_completion_seq = 4

        self.node._on_wp(SimpleNamespace(wp_seq=4))

        self.assertIsNone(self.node._running)
        self.assertIsNone(self.node._mission_monitor_phase)
        self.assertIn(1, self.node._done)
        self.node._publish.assert_called_once_with(
            "done",
            1.0,
            "이륙 · 정찰 마지막 Waypoint 도착 완료",
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

    def test_ready_for_land_requests_auto_land(self):
        self.node._running = 2

        self.node._on_ready_for_land(SimpleNamespace(data=True))

        self.assertEqual(self.node._land_handoff_phase, 2)
        self.assertEqual(len(self.node.set_mode_cli.requests), 1)
        self.assertEqual(
            self.node.set_mode_cli.requests[0].custom_mode,
            orchestrator.PX4_LAND_MODE,
        )

    def test_phase_two_waits_for_landed_state_before_prompt(self):
        phase = SimpleNamespace(
            phase_id=2,
            title="대상 탐지 · 접근",
            script_path=COMMAND_DIR / "phase0.py",
            confirmation="ok_again",
            on_ok="complete",
            confirm_after="landed",
        )
        self.node._running = 2

        with patch.object(
            orchestrator.subprocess,
            "Popen",
            return_value=SuccessfulProcess(),
        ):
            self.node._run_phase(phase)

        self.assertEqual(self.node._waiting_for_landing_phase, 2)
        self.assertIsNone(self.node._pending_confirmation)
        self.node._publish.assert_called_with(
            "running",
            1.0,
            "대상 탐지 · 접근 착륙 완료 확인 중",
            phase=2,
        )

        self.node._on_ext(SimpleNamespace(vtol_state=3, landed_state=1))

        self.assertIsNone(self.node._waiting_for_landing_phase)
        self.assertEqual(self.node._pending_confirmation, "ok_again")
        self.node._publish.assert_called_with(
            "awaiting_confirmation",
            1.0,
            "대상 탐지 · 접근 착륙 완료 — 위치 확인 후 OK 또는 Again을 선택하세요",
            phase=2,
            prompt="ok_again",
        )

    def test_again_waits_when_failsafe_script_is_missing(self):
        self.node._running = 2
        self.node._pending_confirmation = "ok_again"
        self.node._status = {"state": "awaiting_confirmation"}

        self.node._on_phase_response(SimpleNamespace(data="again"))

        self.assertEqual(self.node._running, 2)
        self.assertEqual(self.node._pending_confirmation, "ok_again")
        self.node._publish.assert_called_once_with(
            "awaiting_confirmation",
            1.0,
            "failsafe.py 없음 — 파일 추가 후 Again을 다시 누르세요",
            phase=2,
            prompt="ok_again",
        )

    def test_successful_failsafe_relaunches_phase_two(self):
        phase = self.node._phases[2]
        self.node._running = 2
        self.node._run_phase = Mock()

        with patch.object(
            orchestrator.subprocess,
            "Popen",
            return_value=SuccessfulProcess(),
        ):
            self.node._run_retry_script(phase)

        self.node._run_phase.assert_called_once_with(phase)
        self.node._publish.assert_called_once_with(
            "running",
            0.0,
            "대상 탐지 · 접근 시작점 복귀 완료 — 착륙 재시도 시작",
            phase=2,
        )


if __name__ == "__main__":
    unittest.main()
