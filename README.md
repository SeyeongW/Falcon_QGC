
<p align="center">
  <img src="https://raw.githubusercontent.com/Dronecode/UX-Design/35d8148a8a0559cd4bcf50bfa2c94614983cce91/QGC/Branding/Deliverables/QGC_RGB_Logo_Horizontal_Positive_PREFERRED/QGC_RGB_Logo_Horizontal_Positive_PREFERRED.svg" alt="QGroundControl Logo" width="500">
</p>

<p align="center">
  <a href="https://github.com/mavlink/QGroundControl/releases">
    <img src="https://img.shields.io/github/v/release/mavlink/QGroundControl" alt="Latest Release">
  </a>
</p>

*QGroundControl* (QGC) is a highly intuitive and powerful Ground Control Station (GCS) designed for UAVs. Whether you're a first-time pilot or an experienced professional, QGC provides a seamless user experience for flight control and mission planning, making it the go-to solution for any *MAVLink-enabled drone*.

---

## 🦅 FGC (Falcon GCS) 빌드

이 저장소는 VTOL 픽업 임무용 커스텀 빌드(**FGC**)입니다. `custom/` 오버레이가 ROS2 브리지, 임무 페이즈 콘솔, AI 비전 패널, 3D 항로 뷰를 추가합니다. 실행 파일 이름은 `QGroundControl`이 아니라 **`FGC`** 입니다.

### 사전 준비

```bash
# ROS2 환경 (ROS 브리지를 켜고 빌드하려면 필수)
source /opt/ros/humble/setup.bash

# Qt는 저장소 안에 있습니다: .qt/6.10.3/gcc_64
```

> **주의** — 시스템 cmake(`/usr/bin/cmake`)와 pip cmake가 섞이면 매 실행마다 전체 리컨피그가 발생합니다. 아래 명령은 저장소의 `qt-cmake` 래퍼를 쓰므로 문제없습니다.

---

### 1. 개발 빌드 — 이 컴퓨터에서 바로 실행

일상적인 개발·디버깅용입니다. 패키징을 건너뛰므로 훨씬 빠릅니다.

```bash
./.qt/6.10.3/gcc_64/bin/qt-cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DQGC_BUILD_TESTING=OFF \
    -DQGC_ENABLE_ROS=ON

cmake --build build --parallel
```

실행:

```bash
source /opt/ros/humble/setup.bash
./build/Release/FGC
```

`source` 를 빠뜨리면 ROS 비디오 패널과 임무 페이즈 패널이 DDS 플러그인을 로드하지 못해 비활성 상태로 뜹니다. 빌드 시 RPATH에 `/opt/ros/humble/lib`가 박히므로 `LD_LIBRARY_PATH` 설정은 필요 없습니다.

디버그 빌드가 필요하면 `-DCMAKE_BUILD_TYPE=Debug` 로 바꾸면 `build/Debug/FGC` 가 생성됩니다.

---

### 2. AppImage 빌드 — 배포용

같은 빌드 트리에서 설치 단계를 실행하면 AppImage가 만들어집니다. `appimagetool` 등 필요한 도구는 자동으로 내려받습니다.

```bash
source /opt/ros/humble/setup.bash

./.qt/6.10.3/gcc_64/bin/qt-cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DQGC_BUILD_TESTING=OFF \
    -DQGC_ENABLE_ROS=ON \
    -DQGC_CREATE_APPIMAGE=ON

cmake --build build --parallel
cmake --install build --config Release
```

결과물: **`build/FGC-x86_64.AppImage`**

```bash
chmod +x build/FGC-x86_64.AppImage
./build/FGC-x86_64.AppImage
```

AppImage는 `deploy/linux/AppRun` 이 호스트의 `/opt/ros/$ROS_DISTRO/setup.bash` 를 자동으로 source 하므로 별도 준비 없이 실행됩니다.

| 환경변수 | 효과 |
|---|---|
| `QGC_NO_ROS=1` | ROS 환경 source 를 건너뜁니다 |
| `ROS_DISTRO=jazzy` | humble 외의 배포판을 지정합니다 |

> ROS 라이브러리는 **번들되지 않습니다.** 실행 호스트에 동일한 ROS2 배포판이 설치돼 있어야 ROS 기능이 동작합니다. 없으면 해당 기능만 비활성화되고 앱 자체는 정상 실행됩니다.

---

### 3. SITL 붙여서 실행

```bash
# 터미널 1 — PX4 SITL + Gazebo + MAVROS + 카메라/짐벌 브리지
~/start.sh

# 터미널 2 — FGC
source /opt/ros/humble/setup.bash
./build/Release/FGC
```

- 기체 연결은 **UDP 14550 자동연결**이라 링크를 수동으로 추가할 필요가 없습니다.
- 영상은 비디오 페인 상단의 **영상 소스** 칸에서 `ROS 토픽` / `RTSP·UDP` 를 고릅니다. ROS 토픽은 자동으로 `/fgc/cam`(압축 피드)을 선택합니다.

---

### 정리 / 재빌드

```bash
./tools/clean.py --cache     # CMake 캐시만 삭제
rm -rf build                 # 전체 삭제 (의존성은 .cache/CPM 에서 재사용됨)
```

---

### 🌟 *Why Choose QGroundControl?*

- *🚀 Ease of Use*: A beginner-friendly interface designed for smooth operation without sacrificing advanced features for pros.
- *✈️ Comprehensive Flight Control*: Full flight control and mission management for *PX4* and *ArduPilot* powered UAVs.
- *🛠️ Mission Planning*: Easily plan complex missions with a simple drag-and-drop interface.

🔍 For a deeper dive into using QGC, check out the [User Manual](https://docs.qgroundcontrol.com/en/) – although thanks to QGC's intuitive UI, you may not even need it!

---

### 🚁 *Key Features*

- 🕹️ *Full Flight Control*: Supports all *MAVLink drones*.
- ⚙️ *Vehicle Setup*: Tailored configuration for *PX4* and *ArduPilot* platforms.
- 🔧 *Fully Open Source*: Customize and extend the software to suit your needs.

---

### 💻 *Get Involved!*

QGroundControl is *open-source*, meaning you have the power to shape it! Whether you're fixing bugs, adding features, or customizing for your specific needs, QGC welcomes contributions from the community.

🛠️ Start building today with our [Developer Guide](https://dev.qgroundcontrol.com/en/) and [build instructions](https://dev.qgroundcontrol.com/en/getting_started/).

---

### 🔗 *Useful Links*

- 🌐 [Official Website](http://qgroundcontrol.com)
- 📘 [User Manual](https://docs.qgroundcontrol.com/en/)
- 🛠️ [Developer Guide](https://dev.qgroundcontrol.com/en/)
- 💬 [Discussion & Support](https://docs.qgroundcontrol.com/en/Support/Support.html)
- 🤝 [Contributing](.github/CONTRIBUTING.md) ([Dev Guide](https://dev.qgroundcontrol.com/en/contribute/))
- 📜 [License Information](https://github.com/mavlink/qgroundcontrol/blob/master/.github/COPYING.md)

---

With QGroundControl, you're in full command of your UAV, ready to take your missions to the next level.

---

### Stargazers over time

[![Stargazers over time](https://starchart.cc/mavlink/qgroundcontrol.svg?variant=adaptive)](https://starchart.cc/mavlink/qgroundcontrol)
