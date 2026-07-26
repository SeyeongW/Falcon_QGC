# Handoff notes — `wang` branch (mission-phase 3D/AR work)

> Temporary working memo for continuing this work from a local terminal.
> **Delete this file before marking PR #18 ready for review.**

## What this branch is
Branch `wang` builds on `dev/daehyeon` and adds three related fly-view features.
Draft PR: **#18** (`wang` → `dev/daehyeon`).

Commits on top of `dev/daehyeon` (`3a60358`):
1. `aee1569` — Roll/pitch telemetry binding on the 3D route aircraft
   (`custom/res/Custom/Widgets/MissionRoute3DView.qml`): aircraft node
   `eulerRotation` now uses pitch (X) and roll (Z) from `activeVehicle`, not just
   heading.
2. `1047812` — AR waypoint/path overlay
   (`custom/res/Custom/Widgets/ARWaypointOverlay.qml`, mounted in
   `RosVideoPanel.qml`, registered in `custom/CMakeLists.txt`). Transparent
   `View3D` mirrors the camera pose; waypoints projected with `mapFrom3DScene`,
   drawn as a 2D HUD (marker + seq + distance) plus a path line.
3. `50a1e8c` — Mission-phase adaptive layout (`src/FlyView/FlyView.qml`,
   `custom/src/FlyViewCustomLayer.qml`, `custom/res/Custom/Widgets/MissionPhasePanel.qml`).
   Pane split ratios follow the mission phase; phase relayed from the ROS-only
   `MissionPhasePanel.activePhase` → `FlyViewCustomLayer.missionPhase` →
   `FlyView._applyPhaseLayout()`. `Behavior` animates the transition; a manual
   divider drag pins it (MANUAL) and an AUTO/MANUAL toggle re-enables it.
   Per-phase targets live in `_phaseLayoutTargets()`.

## The CI blocker (why PR #18 is red)
CMake configure fails on every build job:
```
Cannot find source file:
  custom/res/3D/Falcon/generated/meshes/aircraft_Body_mesh.mesh
```
- Referenced by `custom/CMakeLists.txt` (lines ~76, ~120) and
  `custom/res/3D/Falcon/generated/Falcon_aircraft.qml:72`.
- The file is **git-ignored** (`.gitignore` line for
  `custom/res/3D/Falcon/generated/meshes/aircraft_Body_mesh.mesh`) and never
  committed. Pre-existing on `dev/daehyeon`, independent of the 3 commits above.
- The actual asset is **~134 MB** → exceeds GitHub's 100 MB per-file limit, so a
  plain `git add -f` + push is rejected. That's why it was ignored.
- Source model `falcon_aircraft.glb` (~136 MB) is the balsam input; not needed by
  the build, should not be committed as-is.

## Fixing the blocker (do this locally, where the file lives)
**Recommended — decimate, then normal commit:**
```bash
gltfpack -i falcon_aircraft.glb -o falcon_slim.glb -si 0.2   # shrink 10-50x
balsam --generateMeshes falcon_slim.glb                       # regenerate small .mesh
# overwrite custom/res/3D/Falcon/generated/meshes/aircraft_Body_mesh.mesh
# remove the .gitignore entry for that file, then:
git add -f custom/res/3D/Falcon/generated/meshes/*.mesh
git commit -m "Add decimated Falcon body mesh"
git push origin wang
```
**Alternative — Git LFS (keeps the 134 MB, heavier):** requires the CI checkout to
fetch LFS (`lfs: true`); otherwise the build still sees an LFS pointer and fails.

## Follow-ups after the build is green
- Verify attitude axis/sign in SITL (3D model + AR camera); flip an `eulerRotation`
  sign if roll/pitch is inverted.
- Calibrate AR: set `cameraFov` to the real camera; add letterbox correction and
  gimbal-angle wiring (currently the overlay fills the video surface).
- Tune per-phase split ratios in `_phaseLayoutTargets()`.
- Optional: vehicle-derived phase fallback (flightMode + mission item) so the
  adaptive layout works without the ROS orchestrator.
