# FGC — Custom VTOL-GCS Build

QGroundControl 기반 커스텀 지상관제국(FGC). PX4/MAVROS/ROS2 연동, VTOL 임무용 UI 오버레이를 포함합니다.

## 빌드 방법

> **전제**: Qt 6.10.3 (gcc_64)와 ROS2(humble)가 이미 설치되어 있다고 가정합니다.
> 빌드 도구로 [`just`](https://github.com/casey/just)를 사용합니다 (`cargo install rust-just` 또는 `pipx install rust-just`; Ubuntu `apt` 버전은 너무 오래됨).

### 1. 저장소 클론

```bash
git clone <repo-url> qgroundcontrol
cd qgroundcontrol
```

### 2. ROS2 환경 source (필수)

이 빌드는 `QGC_ENABLE_ROS=ON`이 기본이라, configure 전에 반드시 ROS2 환경을 잡아야 합니다.
안 하면 CMake configure 단계에서 실패합니다.

```bash
source /opt/ros/humble/setup.bash
```

### 3. Qt 경로 설정

`justfile`은 기본적으로 `~/Qt/6.10.3/gcc_64`에서 Qt를 찾습니다.
Qt를 다른 곳에 설치했다면 `QT_DIR` 환경변수로 지정하세요.

```bash
# 기본 경로에 설치했다면 생략 가능
export QT_DIR=/path/to/Qt/6.10.3/gcc_64
```

### 4. Configure & Build

```bash
just configure   # git submodule 초기화 + CMake 구성 (최초 1회, 또는 CMake 변경 시)
just build       # 빌드
```

### 5. 실행

앱 이름이 `FGC`이므로 실행 바이너리도 `FGC`입니다.

```bash
./build/Debug/FGC
```

## 자주 겪는 문제

- **`qt-cmake: not found`** — `QT_DIR`이 실제 Qt 설치 경로를 가리키는지 확인하세요.
- **임무 페이즈 패널이 안 보임** — ROS2 환경을 source 하지 않고 빌드하면 `QGC_ENABLE_ROS`가 꺼져
  ROS 연동 UI(미션 페이즈 패널 등)가 숨겨집니다. 2번 단계 후 `just configure`부터 다시 하세요.
- **CMake 변경이 반영 안 됨** — CMake 캐시가 남아 있을 수 있습니다. `just rebuild`로 clean 후 재구성하세요.

## Release / AppImage

배포용 Release 빌드 및 AppImage 패키징:

```bash
source /opt/ros/humble/setup.bash
just release                              # Release 구성 + 빌드
cmake --build build --target qgc-package  # AppImage 패키징
```

## 커스터마이징 참고

- 브랜딩/기능 오버라이드: [`cmake/CustomOverrides.cmake`](cmake/CustomOverrides.cmake)
  (앱 이름 `QGC_APP_NAME`, stable build 여부 `QGC_STABLE_BUILD` 등)
- 커스텀 UI: [`res/Custom/`](res/Custom/), [`src/FlyViewCustomLayer.qml`](src/FlyViewCustomLayer.qml)
- 커스텀 빌드 일반 개념: [QGC Dev Guide](https://dev.qgroundcontrol.com/en/custom_build/custom_build.html)
