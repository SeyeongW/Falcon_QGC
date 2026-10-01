# Mission phase script contract

The onboard `orchestrator.py` starts every phase script with the command
directory as its working directory. A script must exit with `0` only after it
has reached the handoff point configured in `phases.json`; any non-zero exit is
reported as a phase failure.

## Phase 1 and Phase 3

- These phases use `start_action: start_mission`; clicking the phase requests
  `AUTO.MISSION` immediately without launching an OFFBOARD phase script.
- The uploaded mission must be visible on `/mavros/mission/waypoints`. The
  orchestrator shows the operator an OK prompt when
  `/mavros/mission/reached` reports its last `MAV_CMD_NAV_WAYPOINT` sequence.
- At that final Waypoint the orchestrator requests `AUTO.LOITER`, so the
  aircraft holds position while the operator checks the result.
- Pressing OK completes the phase. It does not start another mission.

The retained `script` fields are catalog metadata/fallback paths and are not
executed while `start_action` is `start_mission`.

## Phase 2 and Phase 4

- The phase script owns Position-mode vision descent and alignment. The GCS
  labels these phases `Vision Based Land`; it does not switch the script to
  OFFBOARD.
- Finish the vision descent at the stable 3 m alignment, leave the aircraft in
  Position hover, and exit with `0`.
- A clean script exit makes the GCS ask
  `정렬이 완료 됐습니다. Land 하시겠습니까?`. No separate ROS completion
  topic is required.
- `NO` sends no mode command and keeps the pending decision available in the
  panel footer while the aircraft remains in Position hover.
- `OK` makes the orchestrator request `AUTO.LAND`. The phase is recorded
  complete only after MAVROS reports that the vehicle actually entered
  `AUTO.LAND`.

The independent Failsafe button is enabled only while Phase 2 or Phase 4 is
active and `failsafe.py` exists. It launches `failsafe.py` as a separate local
process. The team-provided phase and failsafe scripts must coordinate control
ownership so they do not publish conflicting commands.

Phase 4 is also registered as pending. Copying `phase4.py` into this directory
automatically exposes it in the panel without a QGC rebuild.

## Mission upload

The GCS Mission upload button uses QGroundControl's live
`PlanMasterController`. The operator selects a `.plan`, `.waypoints`, or `.txt`
file from the file dialog; QGC loads it and sends it through the normal vehicle
mission upload path. The orchestrator learns the new final navigation Waypoint
from `/mavros/mission/waypoints`, which is then used by Phase 1 or Phase 3.
