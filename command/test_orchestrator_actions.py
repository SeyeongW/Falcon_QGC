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

    def test_successful_gripper_action_updates_state(self):
        script = self._write_script(orchestrator.GRIPPER_SCRIPTS["open"])
        process = FakeProcess(returncode=0)
        self.node._gripper_busy = True

        with patch.object(orchestrator.subprocess, "Popen", return_value=process):
            self.node._run_gripper("open", str(script))

        self.assertFalse(self.node._gripper_busy)
        self.assertEqual(self.node._gripper_state, "open")
        self.assertEqual(self.node._action_msg, "Gripper Open 완료")


if __name__ == "__main__":
    unittest.main()
