# VTOL-GCS — Development Progress

Custom QGroundControl build (`VTOL-GCS`) for a VTOL pickup mission:
traverse waypoints → recognize object → grip with gripper → drop at a point →
return home → land. Target flight stack **PX4 SITL + MAVROS** (OFFBOARD); the current
sim tooling is temporarily **ArduPilot/ArduCopter** (GUIDED, Gazebo Harmonic). Mission/
offboard logic in **ROS2**.

This file is the living progress log — keep it updated as work lands.

## Architecture

- **Telemetry, control-surface (servo) data, map/waypoints** → received natively over
  **MAVLink** from PX4 SITL (no ROS dependency for this path).
- **Recognition video + state-machine state + recognition/gripper results** → received
  over **ROS2 topics** via an **rclcpp bridge linked into QGC** (decided 2026-07-08).
  Requires ROS sourced at build/run time; Linux-only (breaks Windows/Android builds),
  gated behind a `QGC_ENABLE_ROS` CMake option.
  - Video: `sensor_msgs/Image` → `QImage` → `RosVideoView` (QQuickPaintedItem).
  - This **supersedes** the earlier plan (RTSP/UDP video + MAVLink custom msgs for
    recognition/gripper) — since rclcpp is now in QGC, everything ROS-side is a topic.
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
| 5 | ROS build plumbing (`QGC_ENABLE_ROS`) + `RosBridge` skeleton (image topic discovery/subscribe/QImage/FPS) | ✅ done — QGC links rclcpp (Humble), verified end-to-end |
| 6 | Video panel, **rqt-style topic selector** (enumerate `sensor_msgs/Image`, switch subscription on pick) → `RosVideoView` | ✅ built & type-checked; run `FAKE_CAM=1 bash ~/falcon_run.sh` to view |
| 7 | Mission-state panel: **user-editable step list** (WP1/WP2/TRANSITION/GRIP/DROP…), highlight from **ROS FSM topic**, + **gripper** indicator | ⬜ todo |
| 8 | Waypoint entry → **real mission upload to PX4** by reusing QGC `PlanMasterController`/`MissionController`/`MissionManager` | ⬜ todo |

## Final panel spec (decided 2026-07-08)

- **Video panel** — rqt_image_view style. Dropdown lists live `sensor_msgs/Image` topics
  (rclcpp `get_topic_names_and_types()` → QML ComboBox model); selecting one recreates the
  subscription. Frame → `QImage` → `RosVideoView` (QQuickPaintedItem). Reuse QGC QML chrome
  (`QGCComboBox`, toolbar-indicator patterns).
- **Mission-state panel** — user edits an ordered step list (add/remove/reorder/label:
  WP1, WP2, TRANSITION, GRIP, DROP, RTL, LAND…). Current step highlighted by matching a
  **ROS FSM topic** (assume `std_msgs/String` unless team has a custom msg). Includes a
  **gripper** state indicator (open/closed/holding) from a ROS topic.
- **Waypoint setup** — coordinate entry drops points on the QGC map AND uploads a real
  mission to PX4 over MAVLink by **reusing QGC's existing mission stack** (do not
  re-implement the mission protocol).

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

## ROS2 bridge (`QGC_ENABLE_ROS`)

`custom/src/Ros/RosBridge.{h,cc}` links rclcpp into QGC and is registered with QML
as the `RosBridge` singleton (`import Custom.Ros`) from `CustomPlugin::createQml­
ApplicationEngine`, all gated behind the `QGC_ENABLE_ROS` CMake option (default OFF;
Linux-only, requires a sourced ROS2 env at both build and run time). The node is
spun on the GUI thread via `spin_some` (QTimer), so callbacks/properties stay on the
Qt thread — no cross-thread marshaling. Phase 5 surface:

- `imageTopics` — live list of `sensor_msgs/Image` topics (re-scanned every 2 s)
- `imageTopic` (RW) / `setImageTopic()` — switch the active subscription (rqt style)
- `imageFps`, `frameReceived(QImage)` — decoded frames (rgb8/bgr8/rgba8/bgra8/mono8/mono16)

Verified end-to-end against Qt 6.10.3 + rclcpp Humble with a self-contained harness
(discovery → subscribe → QImage RGB888 → 10 fps). Fake camera for manual testing:
`python3 custom/tools/fake_image.py /fake/image 10` (rgb8 640x480), or `FAKE_CAM=1
bash ~/falcon_run.sh`.

To build with ROS enabled, add `-DQGC_ENABLE_ROS=ON` to the configure line **from a
shell that has sourced** `source /opt/ros/humble/setup.bash`.

## Real SITL scenario (replaces the fake sender)

`custom/tools/real_mission.py` (rclpy) drives a **real SITL over MAVROS** so QGC shows
genuine data (map/GPS, arming, live `/mavros/rc/out`). FSM: wait for FCU link → wait
local position → set flight mode → ARM (retries until pre-arm passes, incl. the GCS/QGC
datalink — hence QGC must be connected) → takeoff → square waypoints → RTL. It is
**stack-selectable** (`-p stack:=px4` for the eventual PX4 target; default `ardupilot`):
ardupilot→GUIDED/RTL, px4→OFFBOARD/AUTO.RTL. Run alongside your usual bringup:

```bash
# 1) SITL (MAVLink 14550 for QGC)
sim_vehicle.py -v ArduCopter -f JSON --console --map --out=udp:127.0.0.1:14551   # ArduPilot now
# make px4_sitl gz_x500                                                          # PX4 later
# 2) Gazebo world (ArduPilot)
./gazebo/run_sim.sh                        # (in PX4-ROS2/)
# 3) MAVROS (its own link; QGC keeps 14550)
ros2 launch mavros apm.launch fcu_url:=udp://:14551@     # ArduPilot
# ros2 launch mavros px4.launch fcu_url:=udp://:14540@   # PX4
# 4) QGC (ROS-sourced so its bridge sees /mavros/*) and the mission:
bash ~/falcon_run.sh
python3 custom/tools/real_mission.py                 # add: --ros-args -p stack:=px4  (PX4)
```

Note: the SITL vehicle is an ArduCopter **Iris** (4 motors, no control surfaces), so
the control-surface widget won't match it — that widget is for the real VTOL airframe.

## Laptop-GPS fallback for the map (no vehicle connected)

QGC already centers the map on the ground-station position (`gcsPosition`) when no
vehicle GPS is present. This laptop has no GPS hardware, so we use **geoclue wifi**.
Two host-level fixes are needed, plus one custom source:

- geoclue's default wifi backend (Mozilla Location Service) is dead → we repoint it
  at **BeaconDB** and allowlist the app. One-time, as root:
  ```bash
  sudo bash custom/tools/setup_geoclue.sh
  ```
- QGC only accepts a GCS fix with horizontal accuracy ≤ 100 m; wifi fixes are
  coarser. `CustomPlugin::createPositionSource` returns `CustomGeoclueSource`
  (`custom/src/Position/`), which wraps geoclue, keeps the coordinate, and clamps
  the reported accuracy so the fix is accepted (and keeps wifi enabled).

Coarse (city/street level). If BeaconDB has no data for the local APs it may still
return nothing — a manual/fixed home coordinate is the fallback if so.

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
source /opt/ros/humble/setup.bash               # required for -DQGC_ENABLE_ROS=ON
~/.local/bin/cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/.qt/6.10.3/gcc_64" -DQGC_BUILD_TESTING=OFF -DQGC_BUILD_INSTALLER=OFF \
  -DQGC_ENABLE_ROS=ON
~/.local/bin/cmake --build build -j"$(nproc)"  # -> build/Release/VTOL-GCS
```

Run app + fake telemetry together: `bash ~/falcon_run.sh`
(sets `QT_QPA_PLATFORM=xcb`, starts fake_mavlink, launches `build/Release/VTOL-GCS`).

## Git remotes

- `origin` = `github.com/SeyeongW/Falcon_QGC` (team working repo, branch `main`)
- `upstream` = `github.com/mavlink/qgroundcontrol` (pull QGC updates: `git fetch upstream && git merge upstream/master`)
