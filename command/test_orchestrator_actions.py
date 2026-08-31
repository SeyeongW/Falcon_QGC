"""Unit tests for camera and gripper process orchestration."""

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


COMMAND_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(COMMAND_DIR))

import orchestrator  # noqa: E402


class FakeProcess:
    def __init__(self, returncode=None, output=""):
        self.returncode = returncode
        self.output = output
        self.terminated = False

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = -15

    def kill(self):
        self.returncode = -9

    def communicate(self):
        return self.output, None


class OrchestratorActionsTest(unittest.TestCase):
    def setUp(self):
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.command_dir = Path(self._temporary_directory.name)
        self.node = orchestrator.PhaseOrchestrator.__new__(
            orchestrator.PhaseOrchestrator
        )
        self.node._camera_proc = None
        self.node._camera_stopping = False
        self.node._gripper_proc = None
        self.node._gripper_busy = False
        self.node._gripper_state = "unknown"
        self.node._failsafe_proc = None
        self.node._running = None
        self.node._action_msg = ""
        self.node._republish = Mock()
        self.node.get_logger = Mock(return_value=Mock())

        self._command_dir_patch = patch.object(
            orchestrator,
            "COMMAND_DIR",
            str(self.command_dir),
        )
        self._command_dir_patch.start()

    def tearDown(self):
        self._command_dir_patch.stop()
        self._temporary_directory.cleanup()

    def _write_script(self, name):
        path = self.command_dir / name
        path.write_text("print('ok')\n", encoding="utf-8")
        return path

    def test_action_status_reports_script_availability(self):
        self._write_script(orchestrator.CAMERA_SCRIPT)
        self._write_script(orchestrator.GRIPPER_SCRIPTS["close"])

        status = self.node._action_status()

        self.assertTrue(status["camera_available"])
        self.assertFalse(status["camera_running"])
        self.assertFalse(status["gripper_open_available"])
        self.assertTrue(status["gripper_close_available"])
        self.assertFalse(status["failsafe_available"])

    def test_camera_start_and_stop_tracks_process(self):
        self._write_script(orchestrator.CAMERA_SCRIPT)
        process = FakeProcess()

        with patch.object(orchestrator.subprocess, "Popen", return_value=process):
            self.node._start_camera()

        self.assertIs(self.node._camera_proc, process)
        self.assertEqual(self.node._action_msg, "Cam ON")
        self.assertTrue(self.node._action_status()["camera_running"])

        self.node._stop_camera()

        self.assertTrue(process.terminated)
        self.assertEqual(self.node._action_msg, "Cam OFF 처리 중")

    def test_missing_gripper_script_is_rejected(self):
        self.node._start_gripper("open")

        self.assertFalse(self.node._gripper_busy)
        self.assertEqual(self.node._action_msg, "gripper_open.py 없음")

    def test_gripper_action_starts_without_waiting_for_completion(self):
        self._write_script(orchestrator.GRIPPER_SCRIPTS["open"])
        process = FakeProcess()

        with (
            patch.object(orchestrator.subprocess, "Popen", return_value=process),
            patch.object(orchestrator.threading, "Thread") as thread,
        ):
            self.node._start_gripper("open")

        self.assertIs(self.node._gripper_proc, process)
        self.assertTrue(self.node._gripper_busy)
        self.assertEqual(self.node._gripper_state, "open")
        self.assertEqual(self.node._action_msg, "Gripper Open 실행 중")
        thread.return_value.start.assert_called_once()

    def test_new_gripper_action_supersedes_running_action(self):
        self._write_script(orchestrator.GRIPPER_SCRIPTS["open"])
        self._write_script(orchestrator.GRIPPER_SCRIPTS["close"])
        open_process = FakeProcess()
        close_process = FakeProcess()

        with (
            patch.object(
                orchestrator.subprocess,
                "Popen",
                side_effect=[open_process, close_process],
            ),
            patch.object(orchestrator.threading, "Thread"),
        ):
            self.node._start_gripper("open")
            self.node._start_gripper("close")

        self.assertTrue(open_process.terminated)
        self.assertIs(self.node._gripper_proc, close_process)
        self.assertTrue(self.node._gripper_busy)
        self.assertEqual(self.node._gripper_state, "close")
        self.assertEqual(self.node._action_msg, "Gripper Close 실행 중")

    def test_gripper_stop_terminates_running_close(self):
        process = FakeProcess()
        self.node._gripper_proc = process
        self.node._gripper_busy = True
        self.node._gripper_state = "close"

        with patch.object(orchestrator.threading, "Thread") as thread:
            self.node._stop_gripper()

        self.assertTrue(process.terminated)
        self.assertIsNone(self.node._gripper_proc)
        self.assertFalse(self.node._gripper_busy)
        self.assertEqual(self.node._gripper_state, "stopped")
        self.assertEqual(self.node._action_msg, "Gripper Close 종료")
        thread.return_value.start.assert_called_once()

    def test_gripper_stop_when_idle_is_safe(self):
        self.node._stop_gripper()

        self.assertIsNone(self.node._gripper_proc)
        self.assertFalse(self.node._gripper_busy)
        self.assertEqual(self.node._gripper_state, "stopped")
        self.assertEqual(self.node._action_msg, "Gripper가 이미 정지 상태입니다")

    def test_gripper_stop_command_is_routed(self):
        message = Mock(data="gripper:stop")

        with patch.object(self.node, "_stop_gripper") as stop_gripper:
            self.node._on_run_action(message)

        stop_gripper.assert_called_once_with()

    def test_failsafe_is_only_available_during_phase_two_or_four(self):
        self._write_script(orchestrator.FAILSAFE_SCRIPT)

        self.node._start_failsafe()

        self.assertIsNone(self.node._failsafe_proc)
        self.assertIn("Phase 2 또는 Phase 4", self.node._action_msg)

    def test_failsafe_button_command_runs_script_during_phase_two(self):
        self._write_script(orchestrator.FAILSAFE_SCRIPT)
        self.node._running = 2
        process = FakeProcess()

        with (
            patch.object(orchestrator.subprocess, "Popen", return_value=process),
            patch.object(orchestrator.threading, "Thread") as thread,
        ):
            self.node._on_run_action(Mock(data="failsafe:run"))

        self.assertIs(self.node._failsafe_proc, process)
        self.assertEqual(self.node._action_msg, "Failsafe 실행 중")
        thread.return_value.start.assert_called_once_with()

    def test_successful_current_gripper_action_updates_state(self):
        process = FakeProcess(returncode=0)
        self.node._gripper_proc = process
        self.node._gripper_busy = True

        self.node._monitor_gripper("open", process)

        self.assertFalse(self.node._gripper_busy)
        self.assertIsNone(self.node._gripper_proc)
        self.assertEqual(self.node._action_msg, "Gripper Open 완료")

    def test_superseded_gripper_exit_does_not_overwrite_new_action(self):
        old_process = FakeProcess(returncode=-15)
        current_process = FakeProcess()
        self.node._gripper_proc = current_process
        self.node._gripper_busy = True
        self.node._gripper_state = "close"
        self.node._action_msg = "Gripper Close 실행 중"

        self.node._monitor_gripper("open", old_process)

        self.assertIs(self.node._gripper_proc, current_process)
        self.assertTrue(self.node._gripper_busy)
        self.assertEqual(self.node._gripper_state, "close")
        self.assertEqual(self.node._action_msg, "Gripper Close 실행 중")


if __name__ == "__main__":
    unittest.main()
