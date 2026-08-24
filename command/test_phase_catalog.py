"""Unit tests for the onboard phase catalog loader."""

import json
import tempfile
import unittest
from pathlib import Path

from command.common.phase_catalog import (
    CatalogError,
    catalog_payload,
    load_phase_catalog,
)


class PhaseCatalogTest(unittest.TestCase):
    def setUp(self):
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.command_dir = Path(self._temporary_directory.name)

    def tearDown(self):
        self._temporary_directory.cleanup()

    def _write_script(self, name):
        path = self.command_dir / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("print('ok')\n", encoding="utf-8")
        return path

    def _write_manifest(self, phases, version=1):
        document = {"version": version, "phases": phases}
        (self.command_dir / "phases.json").write_text(
            json.dumps(document, ensure_ascii=False), encoding="utf-8"
        )

    def test_discovers_unlisted_numbered_phases_in_id_order(self):
        self._write_script("phase10.py")
        self._write_script("phase2.py")
        self._write_script("helper.py")

        phases = load_phase_catalog(self.command_dir)

        self.assertEqual(list(phases), [2, 10])
        self.assertEqual(phases[2].title, "Phase 2")
        self.assertEqual(phases[10].script, "phase10.py")

    def test_new_numbered_phase_is_visible_on_next_load(self):
        self._write_script("phase0.py")
        self.assertEqual(list(load_phase_catalog(self.command_dir)), [0])

        self._write_script("phase4.py")

        self.assertEqual(list(load_phase_catalog(self.command_dir)), [0, 4])

    def test_manifest_overrides_metadata_and_allows_named_local_script(self):
        self._write_script("phase0.py")
        self._write_script("demos/gripper_demo.py")
        self._write_manifest(
            [
                {
                    "id": 0,
                    "title": "사전 점검",
                    "description": "기체 상태 확인",
                    "script": "phase0.py",
                },
                {
                    "id": 100,
                    "title": "그리퍼 시연",
                    "description": "그리퍼 단독 동작",
                    "script": "demos/gripper_demo.py",
                },
            ]
        )

        phases = load_phase_catalog(self.command_dir)
        payload = catalog_payload(phases)

        self.assertEqual(list(phases), [0, 100])
        self.assertEqual(phases[100].title, "그리퍼 시연")
        self.assertEqual(payload["phases"][1]["id"], 100)
        self.assertTrue(payload["phases"][1]["independent"])
        self.assertNotIn(str(self.command_dir), json.dumps(payload))

    def test_rejects_duplicate_ids(self):
        self._write_script("phase0.py")
        entry = {
            "id": 0,
            "title": "Phase 0",
            "description": "test",
            "script": "phase0.py",
        }
        self._write_manifest([entry, entry])

        with self.assertRaisesRegex(CatalogError, "중복 phase id"):
            load_phase_catalog(self.command_dir)

    def test_rejects_script_outside_command_directory(self):
        outside_script = self.command_dir.parent / "outside_phase.py"
        outside_script.write_text("print('unsafe')\n", encoding="utf-8")
        self.addCleanup(outside_script.unlink, missing_ok=True)
        self._write_manifest(
            [
                {
                    "id": 7,
                    "title": "Unsafe",
                    "description": "must fail",
                    "script": "../outside_phase.py",
                }
            ]
        )

        with self.assertRaisesRegex(CatalogError, "command 폴더 밖"):
            load_phase_catalog(self.command_dir)

    def test_rejects_missing_script(self):
        self._write_manifest(
            [
                {
                    "id": 4,
                    "title": "Missing",
                    "description": "must fail",
                    "script": "phase4.py",
                }
            ]
        )

        with self.assertRaisesRegex(CatalogError, "phase script가 없습니다"):
            load_phase_catalog(self.command_dir)


if __name__ == "__main__":
    unittest.main()
