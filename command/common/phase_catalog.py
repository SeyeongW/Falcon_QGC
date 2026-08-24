"""Load and validate the phase scripts exposed by the onboard orchestrator."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path


CATALOG_FILE_NAME = "phases.json"
CATALOG_VERSION = 1
_NUMBERED_PHASE_RE = re.compile(r"^phase(\d+)\.py$")
_VALID_CONFIRMATIONS = {"none", "ok", "ok_again"}
_VALID_ON_OK_ACTIONS = {"complete", "start_mission"}
_VALID_CONFIRM_AFTER = {"process_exit", "landed"}
_VALID_READY_ACTIONS = {"none", "land"}


class CatalogError(ValueError):
    """The phase catalog cannot be used safely."""


@dataclass(frozen=True)
class PhaseDefinition:
    """Validated phase metadata and its resolved local script path."""

    phase_id: int
    title: str
    description: str
    script: str
    script_path: Path
    confirmation: str
    on_ok: str
    available: bool
    confirm_after: str
    ready_action: str
    retry_script: str | None
    retry_script_path: Path | None

    def public_dict(self):
        """Return the metadata sent to FGC; never expose an absolute MC path."""
        return {
            "id": self.phase_id,
            "title": self.title,
            "desc": self.description,
            "script": self.script,
            "independent": True,
            "confirmation": self.confirmation,
            "on_ok": self.on_ok,
            "available": self.available,
            "confirm_after": self.confirm_after,
            "ready_action": self.ready_action,
            "retry_script": self.retry_script or "",
            "retry_available": bool(
                self.retry_script_path is not None
                and self.retry_script_path.is_file()
            ),
        }


def _read_manifest(command_dir):
    manifest_path = command_dir / CATALOG_FILE_NAME
    if not manifest_path.exists():
        return []

    try:
        with manifest_path.open(encoding="utf-8") as stream:
            document = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise CatalogError(f"{CATALOG_FILE_NAME} 읽기 실패: {exc}") from exc

    if not isinstance(document, dict):
        raise CatalogError(f"{CATALOG_FILE_NAME} 최상위 값은 객체여야 합니다")
    if document.get("version", CATALOG_VERSION) != CATALOG_VERSION:
        raise CatalogError(
            f"지원하지 않는 catalog version: {document.get('version')}"
        )

    entries = document.get("phases", [])
    if not isinstance(entries, list):
        raise CatalogError("phases 값은 배열이어야 합니다")
    return entries


def _validate_script(command_dir, script, allow_missing=False):
    if not isinstance(script, str) or not script.strip():
        raise CatalogError("script는 비어 있지 않은 문자열이어야 합니다")

    script = script.strip()
    if Path(script).suffix != ".py":
        raise CatalogError(f"Python phase 파일만 실행할 수 있습니다: {script}")

    command_root = command_dir.resolve()
    script_path = (command_root / script).resolve()
    try:
        script_path.relative_to(command_root)
    except ValueError as exc:
        raise CatalogError(f"command 폴더 밖의 script는 사용할 수 없습니다: {script}") from exc

    if not allow_missing and not script_path.is_file():
        raise CatalogError(f"phase script가 없습니다: {script}")

    return script, script_path


def _definition_from_entry(command_dir, entry):
    if not isinstance(entry, dict):
        raise CatalogError("각 phase 항목은 객체여야 합니다")

    phase_id = entry.get("id")
    if isinstance(phase_id, bool) or not isinstance(phase_id, int) or phase_id < 0:
        raise CatalogError(f"phase id는 0 이상의 정수여야 합니다: {phase_id}")

    pending = entry.get("pending", False)
    if not isinstance(pending, bool):
        raise CatalogError(f"Phase {phase_id} pending은 bool이어야 합니다")

    script, script_path = _validate_script(
        command_dir,
        entry.get("script"),
        allow_missing=pending,
    )
    title = entry.get("title", f"Phase {phase_id}")
    description = entry.get("description", script)
    confirmation = entry.get("confirmation", "none")
    on_ok = entry.get("on_ok", "complete")
    confirm_after = entry.get("confirm_after", "process_exit")
    ready_action = entry.get("ready_action", "none")
    retry_script = entry.get("retry_script")
    if not isinstance(title, str) or not title.strip():
        raise CatalogError(f"Phase {phase_id} title이 올바르지 않습니다")
    if not isinstance(description, str):
        raise CatalogError(f"Phase {phase_id} description이 올바르지 않습니다")
    if confirmation not in _VALID_CONFIRMATIONS:
        raise CatalogError(
            f"Phase {phase_id} confirmation이 올바르지 않습니다: {confirmation}"
        )
    if on_ok not in _VALID_ON_OK_ACTIONS:
        raise CatalogError(
            f"Phase {phase_id} on_ok가 올바르지 않습니다: {on_ok}"
        )
    if confirmation == "none" and on_ok != "complete":
        raise CatalogError(
            f"Phase {phase_id} on_ok={on_ok}에는 confirmation이 필요합니다"
        )
    if confirm_after not in _VALID_CONFIRM_AFTER:
        raise CatalogError(
            f"Phase {phase_id} confirm_after가 올바르지 않습니다: {confirm_after}"
        )
    if ready_action not in _VALID_READY_ACTIONS:
        raise CatalogError(
            f"Phase {phase_id} ready_action이 올바르지 않습니다: {ready_action}"
        )
    if confirm_after == "landed" and confirmation == "none":
        raise CatalogError(
            f"Phase {phase_id} landed 확인에는 confirmation이 필요합니다"
        )
    if retry_script is not None and confirmation != "ok_again":
        raise CatalogError(
            f"Phase {phase_id} retry_script에는 confirmation=ok_again이 필요합니다"
        )

    retry_script_path = None
    if retry_script is not None:
        retry_script, retry_script_path = _validate_script(
            command_dir,
            retry_script,
            allow_missing=True,
        )

    return PhaseDefinition(
        phase_id=phase_id,
        title=title.strip(),
        description=description.strip(),
        script=script,
        script_path=script_path,
        confirmation=confirmation,
        on_ok=on_ok,
        available=script_path.is_file(),
        confirm_after=confirm_after,
        ready_action=ready_action,
        retry_script=retry_script,
        retry_script_path=retry_script_path,
    )


def load_phase_catalog(command_dir):
    """Return validated phases keyed by id.

    ``phases.json`` supplies operator-facing metadata and may point to any Python
    script below ``command_dir``. Numbered ``phaseN.py`` files not present in the
    manifest are added automatically with a generic title, so copying a new
    numbered phase to the MC is enough for it to appear in FGC.
    """
    command_dir = Path(command_dir)
    phases = {}

    for entry in _read_manifest(command_dir):
        definition = _definition_from_entry(command_dir, entry)
        if definition.phase_id in phases:
            raise CatalogError(f"중복 phase id: {definition.phase_id}")
        phases[definition.phase_id] = definition

    try:
        children = list(command_dir.iterdir())
    except OSError as exc:
        raise CatalogError(f"command 폴더 읽기 실패: {exc}") from exc

    for path in children:
        match = _NUMBERED_PHASE_RE.fullmatch(path.name)
        if match is None:
            continue
        phase_id = int(match.group(1))
        if phase_id in phases:
            continue
        script, script_path = _validate_script(command_dir, path.name)
        phases[phase_id] = PhaseDefinition(
            phase_id=phase_id,
            title=f"Phase {phase_id}",
            description=f"{script} 실행",
            script=script,
            script_path=script_path,
            confirmation="none",
            on_ok="complete",
            available=True,
            confirm_after="process_exit",
            ready_action="none",
            retry_script=None,
            retry_script_path=None,
        )

    return dict(sorted(phases.items()))


def catalog_payload(phases):
    """Build the versioned JSON-compatible payload published to FGC."""
    return {
        "version": CATALOG_VERSION,
        "phases": [definition.public_dict() for definition in phases.values()],
    }
