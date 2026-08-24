# Mission phase script contract

The onboard `orchestrator.py` starts every phase script with the command
directory as its working directory. A script must exit with `0` only after it
has reached the handoff point configured in `phases.json`; any non-zero exit is
reported as a phase failure.

## Phase 1 and Phase 3

- Read the hover coordinate from the matching `common/phaseN_mission.py` file.
- Keep OFFBOARD setpoints active until the aircraft is safely holding at that
  coordinate, then hand off to a stable hold mode and exit with `0`.
- Do not start the uploaded mission in the phase script. The orchestrator shows
  the operator an OK prompt and requests `AUTO.MISSION` after OK.
- The uploaded mission must be visible on `/mavros/mission/waypoints`. The
  orchestrator completes the phase when `/mavros/mission/reached` reports its
  last `MAV_CMD_NAV_WAYPOINT` sequence.

Phase 3 is registered as pending. Copying a valid `phase3.py` into this
directory makes it available automatically; no QGC rebuild is required.

## Phase 2 and Phase 4

- Publish `True` on `/mission/ready_for_land` only after the vision controller
  has reached a stable landing alignment.
- Keep OFFBOARD control active until the orchestrator changes the vehicle to
  `AUTO.LAND`. Exit with `0` after detecting that safe mode handoff.
- The orchestrator waits for MAVROS `landed_state == ON_GROUND` before showing
  the operator the OK/Again dialog.
- A retry script must return the aircraft to the matching phase start hover and
  exit with `0`. The orchestrator then launches the phase script again.

Phase 2 uses `failsafe.py`; Phase 4 uses `failsafe2.py`. Missing retry scripts
are reported in the panel without discarding the pending OK/Again decision.

Phase 4 is also registered as pending. Copying `phase4.py` into this directory
automatically exposes it in the panel. Copying `failsafe2.py` enables its Again
recovery path without a QGC rebuild.
