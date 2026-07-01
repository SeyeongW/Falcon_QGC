# VTOL-GCS — Development Progress

Custom QGroundControl build (`VTOL-GCS`) for a VTOL pickup mission:
traverse waypoints → recognize object → grip with gripper → drop at a point →
return home → land. Flight stack **PX4 + MAVROS**; mission/offboard logic in **ROS2**.

This file is the living progress log — keep it updated as work lands.

## Architecture

- **Telemetry, control-surface (servo) data, map/waypoints** → received natively over
  **MAVLink** from PX4 (no ROS dependency inside QGC).
- **Recognition video** → vision node streams **RTSP/UDP**, reuse QGC's video stack.
- **Recognition results / gripper state** → **MAVLink custom messages** (planned).
- Built as the **`custom/` overlay** (QGC custom build), app name `VTOL-GCS`.

## Status

| Phase | Item | Status |
|-------|------|--------|
| 1 | `custom/` baseline overlay, branded `VTOL-GCS` | ✅ done |
| 2 | Control-surface widget (quadplane, VTOL mode-adaptive) | ✅ done |
| 2a | 3 surfaces (aileron/elevator/rudder), real 3D up/down tilt | ✅ done |
| 3 | Live binding to `SERVO_OUTPUT_RAW` + VTOL mode (no C++) | ✅ done |
| – | Fake-signal generator `custom/tools/fake_mavlink.py` | ✅ done, verified |
| 4 | Real QGC build (Qt 6.10) + run with fake signal | ✅ builds & runs; connects to fake vehicle |
| 5 | RTSP recognition-video panel | ⬜ todo |
| 6 | Waypoint coordinate-entry panel (type coords + count) | ⬜ todo |
| 7 | MAVLink custom messages for recognition/gripper | ⬜ todo |

## Servo channel convention

`custom/tools/fake_mavlink.py` and the panel binding in
`custom/src/FlyViewCustomLayer.qml` must agree:

| Channel | Use | Encoding |
|---------|-----|----------|
| ch1 | aileron (roll) | 1000–2000 µs, 1500 neutral → [-1,1] |
| ch2 | elevator (pitch) | 1000–2000 µs, 1500 neutral |
| ch3 | pusher motor | 1000–2000 µs → [0,1] |
| ch4 | rudder (yaw) | 1000–2000 µs, 1500 neutral |
| ch5–8 | lift motors | 1000–2000 µs → [0,1] |

## How to preview / test (no full build, system Qt 6.2)

```bash
bash ~/falcon_preview.sh   # control-surface widget only (auto-toggles mode)
bash ~/falcon_gcs.sh       # mock GCS demo (map + telemetry + panel, simulated)
```

## Build real QGC (Qt 6.10) and run with fake telemetry

Requires cmake ≥3.25 and Qt 6.10 (system apt Qt 6.2 is preview-only).

```bash
pip install cmake                              # cmake ≥3.25 (no sudo)
sudo python3 tools/setup/install_dependencies  # apt build deps (sudo)
python3 tools/setup/install_qt.py              # fetch Qt 6.10 (several GB)
# configure + build (custom/ is auto-detected); then run, and in another shell:
python3 custom/tools/fake_mavlink.py           # fake PX4 VTOL → QGC UDP 14550
```

Actual working build (Ubuntu 22.04, system Python 3.10):

```bash
pip install --user cmake tomli                 # cmake ≥3.25 + tomli for py3.10
python3 tools/setup/install_qt.py install --version 6.10.3 --host linux \
  --target desktop --arch linux_gcc_64 \
  --modules "qtgraphs qtlocation qtpositioning qtspeech qtmultimedia qtserialport qtimageformats qtshadertools qtconnectivity qtquick3d qtsensors qtscxml qtwebsockets qthttpserver"
~/.local/bin/cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/.qt/6.10.3/gcc_64" -DQGC_BUILD_TESTING=OFF -DQGC_BUILD_INSTALLER=OFF
~/.local/bin/cmake --build build -j"$(nproc)"  # -> build/Release/VTOL-GCS
```

Run app + fake telemetry together: `bash ~/falcon_run.sh`
(sets `QT_QPA_PLATFORM=xcb`, starts fake_mavlink, launches `build/Release/VTOL-GCS`).

## Git remotes

- `origin` = `github.com/SeyeongW/Falcon_QGC` (team working repo, branch `main`)
- `upstream` = `github.com/mavlink/qgroundcontrol` (pull QGC updates: `git fetch upstream && git merge upstream/master`)
