"""
phase2_offboard_precision_landing.py

목적
----
조난자/착륙 표적을 이용한 Phase 2 비전 유도 + OFFBOARD 정밀 착륙 제어기.

핵심 설계
---------
    1) VTOL이 ARMED, MULTICOPTER 상태에서 시작한다.
    2) /mission/target_info 기반 Vision PID/PI로 OFFBOARD 진입 후 3 m까지 접근한다.
    3) 3 m에서 기존 COARSE PID -> FINE PI -> DEADBAND 정렬 기능을 그대로 사용한다.
    4) 엄격한 위치/헤딩/고도/속도 조건이 서로 다른 Vision frame에서 연속으로 만족되면
       /mission/ready_for_land = True를 latch한다.
    5) READY 순간 Local X/Y/Yaw를 저장하고 READY_WAIT_GCS 상태로 전환한다.
       - Vision 정렬 제어는 종료한다.
       - 저장한 Local X/Y/Yaw를 유지한다.
       - FINAL_ALIGN_ALT_M(기본 3 m)을 계속 유지하며 GCS 승인을 기다린다.
    6) GCS에서 팝업 OK를 누르면 /mission/land_confirm = True를 수신한다.
    7) 같은 OFFBOARD 안에서 OFFBOARD_LANDING 상태로 전환한다.
       - 저장한 Local X/Y/Yaw를 유지하면서 Z축만 하강시킨다.
       - 착륙 중 Vision XY/Yaw 정렬 제어는 사용하지 않는다.
       - Z축은 Optical Flow 지면거리 우선으로 단계적으로 하강속도를 낮춘다.
    8) /mavros/extended_state의 landed_state가 일정 시간 ON_GROUND이면 touchdown 완료로 판정한다.

주의
----
- /mission/ready_for_land=True는 착륙 시작 명령이 아니라 GCS 팝업 표시용 준비 완료 신호다.
- 실제 하강 시작은 GCS가 /mission/land_confirm=True를 보낸 뒤에만 수행한다.
- AUTO.LAND 모드로 전환하지 않고 OFFBOARD 상태를 계속 유지한다.
- 실제 비행 전에는 SITL에서 제어 부호, Optical Flow 거리, touchdown 판정을 먼저 검증한다.

좌표계 가정
-----------
robo_jinheui.py 기준:
    영상 위쪽    = 기체 전방
    영상 오른쪽  = 기체 오른쪽

/mavros/local_position/pose, /velocity_local은 ROS ENU로 가정:
    x = East, y = North, z = Up
"""

import math
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, OpticalFlowRad, State
from mavros_msgs.srv import SetMode
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32MultiArray


# =============================================================================
# 사용자 조정 파라미터
# =============================================================================
#
# 임무 및 제어 튜닝에 필요한 값은 모두 이 영역에 모아두었다.
# 실제 비행 전에 시뮬레이션에서 보수적인 값부터 조정하는 것을 권장한다.
# =============================================================================


# -----------------------------------------------------------------------------
# 1. 제어 주기 / OFFBOARD
# -----------------------------------------------------------------------------

CONTROL_RATE_HZ = 20.0

# PX4는 OFFBOARD 진입 전에 일정 시간 연속 setpoint 송신이 필요하다.
OFFBOARD_PRESTREAM_SEC = 2.0

# OFFBOARD 진입이 확인될 때까지 일정 주기로 모드 요청을 반복한다.
OFFBOARD_REQUEST_INTERVAL_SEC = 1.0
OFFBOARD_ENTRY_TIMEOUT_SEC = 15.0


# -----------------------------------------------------------------------------
# 2. 실기 영상인식 인터페이스 / 제어 방향 부호
# -----------------------------------------------------------------------------
#
# robo_jinheui.py의 /mission/target_info와 직접 호환한다.
#
# 핵심 인덱스:
#   0  : detected
#   3  : error_x_px
#   4  : error_y_px
#   5  : error_x_norm
#   6  : error_y_norm
#   7  : bearing_x_rad
#   8  : bearing_y_rad
#   17 : orientation_valid (= heading_valid)
#   18 : heading_error_rad
#
# -------------------- 실기 제어 부호 --------------------
#
# 실제 카메라 영상 방향/짐벌 장착 방향이 시뮬레이션과 다를 경우
# 아래 세 값만 +1.0 <-> -1.0으로 바꾸면 된다.
#
# X_CONTROL_SIGN
#   +1.0 : 표적이 영상 오른쪽에 있으면 기체를 오른쪽으로 이동
#   -1.0 : 반대로 이동
#
# Y_CONTROL_SIGN
#   -1.0 : 표적이 영상 아래에 있으면 기체를 뒤로 이동
#   +1.0 : 반대로 이동
#
# YAW_CONTROL_SIGN
#   heading_error의 부호와 실제 기체 yaw 반응을 맞춘다.
#
# 실기 첫 시험은 반드시 낮은 속도에서
# "오차가 줄어드는 방향으로 움직이는지" 확인한 뒤 사용한다.
X_CONTROL_SIGN = +1.0
Y_CONTROL_SIGN = -1.0
YAW_CONTROL_SIGN = -1.0


# -------------------- 타겟 장축 대비 최종 기체 헤딩 --------------------
#
# robo_jinheui.py의 heading_error_rad는
# "영상 위쪽(기체 전방)=0°" 기준 타겟 장축의 상대각이다.
#
# 0.0°:
#   기체 전방과 타겟 장축을 평행하게 정렬
#
# 90.0°:
#   기체 전방과 타겟 장축을 직각으로 정렬
#
# PCA 장축은 180° 대칭이므로 +90°와 -90°는 동일한 '직각 정렬' 의미를 가진다.
# 따라서 정확히 어느 쪽 90° 방향(좌/우)을 구분하는 용도로는 사용할 수 없다.
DESIRED_HEADING_OFFSET_DEG = 0.0


# -----------------------------------------------------------------------------
# 3. 최종 정렬 고도
# -----------------------------------------------------------------------------

# 최종 정렬/호버링 목표 지면고도 [m].
# Optical Flow가 유효할 때는 센서의 distance가 이 값이 되도록 제어한다.
FINAL_ALIGN_ALT_M = 3.0

# 착륙 지점 지면이 이륙 지점(local z=0)보다 얼마나 높거나 낮은지 [m].
# 예) 착륙장이 이륙장보다 2 m 높음  -> +2.0
#     착륙장이 이륙장보다 1 m 낮음  -> -1.0
# Optical Flow가 유효할 때는 이 오프셋을 사용하지 않고 실제 distance=3 m를 사용한다.
LANDING_GROUND_OFFSET_M = 0.0

# Local Z 기준으로 Optical Flow 저고도 고도제어를 활성화할 기준.
# 실제 local target = FINAL_ALIGN_ALT_M + LANDING_GROUND_OFFSET_M
OPTICAL_FLOW_ENABLE_TOL_M = 0.20

# 최종 정렬 고도 허용오차 [m].
FINAL_ALIGN_ENTER_TOL_M = 0.20

# Optical Flow 데이터 유효성 기준.
OPTICAL_FLOW_MSG_TIMEOUT_SEC = 0.50
OPTICAL_FLOW_MIN_DISTANCE_M = 0.05
OPTICAL_FLOW_MAX_DISTANCE_M = 20.0
OPTICAL_FLOW_MIN_QUALITY = 1

# 수직 접근 및 3 m 고도 유지를 위한 P 제어기.
# Optical Flow 사용 시:
#   vz_cmd = ALTITUDE_KP * (FINAL_ALIGN_ALT_M - optical_flow_distance)
# fallback 시:
#   vz_cmd = ALTITUDE_KP * ((FINAL_ALIGN_ALT_M + LANDING_GROUND_OFFSET_M) - local_z)
ALTITUDE_KP = 0.65

# 최대 수직 속도 [m/s].
# ROS ENU 기준: +z는 상승, -z는 하강.
MAX_DESCEND_SPEED_MPS = 0.60
MAX_CLIMB_SPEED_MPS = 0.30

# -------------------------------------------------------------------------
# FOV 오차에 따른 하강속도 연속 조절
# -------------------------------------------------------------------------
#
# 기존처럼 FOV soft limit을 넘는 순간 하강속도를 0으로 만들면
#   하강 -> 정지 -> XY 보정 -> 다시 하강
# 형태가 반복되어 궤적이 계단식으로 보일 수 있다.
#
# 따라서 표적이 화면 중심에서 멀어질수록 하강속도를 연속적으로 줄인다.
#
# error_norm <= DESCENT_FULL_SPEED_FOV_NORM
#   -> 정상 하강속도 100%
#
# error_norm >= FOV_HARD_LIMIT_NORM
#   -> 최소 DESCENT_MIN_SCALE 비율로 계속 하강
#
# 중간 영역에서는 선형적으로 하강속도를 줄인다.
DESCENT_FULL_SPEED_FOV_NORM = 0.40
DESCENT_MIN_SCALE = 0.15


# -----------------------------------------------------------------------------
# 4. 카메라 오프셋 - 픽셀 기준
# -----------------------------------------------------------------------------
#
# 실제 시스템에서는 카메라 장착 오프셋을 '미터'로 환산하지 않고,
# 화면 중심으로부터 몇 pixel 떨어진 위치에 조난자가 와야 하는지 직접 지정한다.
#
# robo_jinheui.py에서 이미
#   error_x_px = 조난자 중심 x - 영상 중심 x
#   error_y_px = 조난자 중심 y - 영상 중심 y
# 를 계산하므로 영상 해상도를 다시 알 필요가 없다.
#
# 부호 규약:
#   +X pixel = 영상 오른쪽
#   +Y pixel = 영상 아래쪽
#
# 예:
#   LANDING_TARGET_OFFSET_Y_PX = 200.0
#   -> 최종 정렬 시 조난자 중심을 영상 중심보다 200 px 아래에 위치시킨다.
#
# 아래 값은 반드시 실제 기체/카메라 화면을 보고 조정할 것.
LANDING_TARGET_OFFSET_X_PX = 0.0
LANDING_TARGET_OFFSET_Y_PX = 200.0


# -----------------------------------------------------------------------------
# 5. 3 m 최종 Landing Zone - 픽셀 기준
# -----------------------------------------------------------------------------
#
# Landing Zone 자체는 너무 넓게 잡지 않는다.
# 현재 기본값:
#   가로 전체 폭 = 80 px
#   세로 전체 높이 = 80 px
#
# 조난자 중심이 아래 사각형 영역 안에 들어오면 position_ok=True.
LANDING_ZONE_HALF_WIDTH_PX = 40.0
LANDING_ZONE_HALF_HEIGHT_PX = 40.0

# 최종 허용영역은 영상 중심 기준:
#   X: -40 ~ +40 px
#   Y: +160 ~ +240 px
#
# 이 ±40 px 영역은 READY_FOR_LAND 위치 판정에 사용한다.
# 실제 XY 제어를 완전히 0으로 만드는 Deadband는 아래 FINAL_DEADBAND_*에서
# 별도로 더 작게 설정한다.

# 기존 Landing Zone(±40 px)은 최종 정렬 제어/모니터링에 유지한다.
# 실제 착륙 시작 허가는 더 엄격한 별도 조건을 사용한다.
READY_STRICT_X_PX = 25.0
READY_STRICT_Y_PX = 25.0
READY_STRICT_ALT_TOL_M = 0.10
READY_STRICT_HEADING_TOL_DEG = 8.0
READY_MAX_XY_SPEED_MPS = 0.10
READY_MAX_ABS_VZ_MPS = 0.08

# 동일한 stale sample을 여러 번 세지 않고, 서로 다른 Vision frame이
# 연속으로 이 횟수만큼 조건을 만족해야 READY를 latch한다.
READY_REQUIRED_FRAMES = 2

# GCS와 합의된 READY 토픽은 그대로 유지한다.
# True가 한 번 성립하면 착륙이 끝날 때까지 latch한다.
READY_RELEASE_MARGIN_PX = 20.0
READY_RELEASE_HEADING_TOL_DEG = 18.0
REQUIRE_VALID_HEADING_FOR_READY = True

# -------------------------------------------------------------------------
# GCS 승인 후 OFFBOARD 정밀 착륙 설정
# -------------------------------------------------------------------------
# READY=True가 된 뒤에도 즉시 하강하지 않는다.
# READY 순간 저장한 Local X/Y/Yaw와 FINAL_ALIGN_ALT_M을 유지한 채 대기하고,
# GCS가 LAND_CONFIRM_TOPIC에 Bool(True)를 보내면 그때 OFFBOARD_LANDING을 시작한다.

# READY 순간 저장한 Local X/Y/Yaw를 아래 P 제어로 끝까지 유지한다.
# OFFBOARD_LANDING 진입 이후 Vision은 제어에 사용하지 않는다.
LAND_LOCAL_XY_KP = 0.60
LAND_LOCAL_XY_MAX_SPEED_MPS = 0.20
LAND_LOCAL_YAW_KP = 1.00
LAND_LOCAL_YAW_MAX_RATE_DEG_S = 10.0

# 지면까지 높이에 따른 OFFBOARD 하강속도 [m/s]. ROS ENU이므로 실제 명령은 음수.
LAND_DESCEND_SPEED_MPS = 0.25
LAND_SLOW_ALT_M = 1.00
LAND_SLOW_DESCEND_SPEED_MPS = 0.12
LAND_CRAWL_ALT_M = 0.35
LAND_CRAWL_DESCEND_SPEED_MPS = 0.06

# landed_state가 ON_GROUND로 연속 유지되어야 touchdown 완료로 판정한다.
TOUCHDOWN_CONFIRM_SEC = 0.50


# -----------------------------------------------------------------------------
# 6. 3 m 이전 FOV 유지
# -----------------------------------------------------------------------------

# 3 m 이전의 최우선 목표는 조난자를 카메라 화면 밖으로 놓치지 않는 것이다.
#
# SOFT limit:
#   수평 보정 강도를 판단하는 참고 경계.
#
# HARD limit:
#   이 범위를 넘으면 최대 수평 보정 속도를 사용하고,
#   하강속도는 DESCENT_MIN_SCALE까지 감소한다.
FOV_SOFT_LIMIT_NORM = 0.65
FOV_HARD_LIMIT_NORM = 0.85

# 3 m 이전에는 작은 deadband를 둔다.
# 접근 단계에서는 픽셀 단위의 완벽한 중앙 정렬이 필요하지 않다.
APPROACH_DEADBAND_NORM = 0.04


# -----------------------------------------------------------------------------
# 7. 수평 PID - 접근 단계 (> 3 m)
# -----------------------------------------------------------------------------
#
# PID 입력은 robo_jinheui.py의 bearing_x/bearing_y 각도 오차 [rad]이다.
# PID 출력은 기체 Body frame 기준 수평 속도 명령 [m/s]이다.
#
# X 제어:
#   영상 좌/우 오차 -> 기체 우/좌 이동
#
# Y 제어:
#   영상 상/하 오차 -> 기체 전/후 이동
#
# 아래 값들은 초기 보수값이며 실제 비행시험을 통해 튜닝해야 한다.
APPROACH_X_KP = 0.80
APPROACH_X_KI = 0.02
APPROACH_X_KD = 0.08

APPROACH_Y_KP = 0.80
APPROACH_Y_KI = 0.02
APPROACH_Y_KD = 0.08

APPROACH_PID_I_LIMIT = 0.20
APPROACH_MAX_XY_SPEED_MPS = 0.70


# -----------------------------------------------------------------------------
# 8. 3 m 최종 수평 정렬: PID -> 저게인 PI -> Deadband
# -----------------------------------------------------------------------------
#
# 목표 pixel:
#   X = 0 px
#   Y = +200 px
#
# 한 개의 PID만 계속 사용하면 목표점 부근에서 기체 관성 + 영상 노이즈 때문에
# 앞뒤/좌우 왕복이 생길 수 있다.
#
# 따라서 3 m에서는 다음 3단계로 나눈다.
#
#   [1] COARSE PID
#       목표에서 멀리 있을 때 사용.
#       비교적 빠르게 목표 영역으로 접근한다.
#
#   [2] FINE PI
#       목표점 기준 ±80 px 영역 안에 들어오면 D항을 제거한 저게인 PI로 전환.
#       최대 속도도 낮춰서 목표점으로 천천히 수렴한다.
#
#   [3] DEADBAND
#       목표점 기준 ±25 px 안에 들어오면 XY 명령을 0으로 만든다.
#       픽셀 노이즈를 끝까지 추종하지 않도록 한다.
#
# READY 판정용 Landing Zone(±40 px)은 위 4번 영역을 그대로 사용한다.
# 즉 READY 판정 영역과 제어 Deadband는 서로 다른 개념이다.


# ----- [1] COARSE PID : 목표에서 멀 때 -----
# 입력 단위 = pixel
FINAL_X_KP = 0.0040
FINAL_X_KI = 0.0002
FINAL_X_KD = 0.0010

FINAL_Y_KP = 0.0040
FINAL_Y_KI = 0.0002
FINAL_Y_KD = 0.0010

FINAL_PID_I_LIMIT = 80.0
FINAL_MAX_XY_SPEED_MPS = 0.30


# ----- [2] FINE PI : 목표 주변에서 부드럽게 수렴 -----
#
# 목표점 기준 이 범위 안에 들어오면 COARSE PID -> FINE PI로 전환한다.
FINE_ALIGN_ENTER_X_PX = 80.0
FINE_ALIGN_ENTER_Y_PX = 80.0

# 경계에서 PID/PI가 계속 왔다 갔다 하지 않도록 해제 범위를 조금 더 크게 둔다.
# FINE PI에 들어온 뒤 이 범위를 벗어나야 다시 COARSE PID로 돌아간다.
FINE_ALIGN_EXIT_X_PX = 100.0
FINE_ALIGN_EXIT_Y_PX = 100.0

# 저게인 PI 초기값.
# D항은 사용하지 않는다.
FINE_X_KP = 0.0015
FINE_X_KI = 0.00005

FINE_Y_KP = 0.0015
FINE_Y_KI = 0.00005

FINE_PI_I_LIMIT = 120.0

# FINE PI 영역에서는 최대 수평속도를 매우 낮게 제한한다.
FINE_MAX_XY_SPEED_MPS = 0.12


# ----- [3] 최종 Deadband -----
#
# 목표점 (0, +200 px) 기준으로 이 영역 안에서는 XY 속도 명령을 0으로 한다.
# READY Landing Zone(±40 px)보다 더 작은 영역이다.
FINAL_DEADBAND_X_PX = 25.0
FINAL_DEADBAND_Y_PX = 25.0


# -----------------------------------------------------------------------------
# 9. Yaw PI 제어
# -----------------------------------------------------------------------------

# robo_jinheui.py의 heading_error_rad는 PCA 기반 조난자 장축 추정값이다.
YAW_KP = 1.20
YAW_KI = 0.10
YAW_I_LIMIT_RAD_S = math.radians(8.0)

# 최대 yaw-rate 제한.
APPROACH_MAX_YAW_RATE_DEG_S = 20.0
FINAL_MAX_YAW_RATE_DEG_S = 12.0

# Yaw 제어 방향 부호는 파일 상단의
# YAW_CONTROL_SIGN 파라미터에서 조정한다.


# -----------------------------------------------------------------------------
# 10. PID D항 필터 / 표적 데이터 타임아웃
# -----------------------------------------------------------------------------

# D항 저역통과 필터:
#   d_filtered = alpha*d_raw + (1-alpha)*d_previous
# 값이 작을수록 D항이 더 부드럽게 반응한다.
PID_D_FILTER_ALPHA = 0.15

# -------------------------------------------------------------------------
# 영상 오차 저역통과 필터
# -------------------------------------------------------------------------
# 새 측정값의 반영 비율.
# 값이 작을수록 영상 중심 검출 노이즈에 덜 출렁이지만 반응이 느려진다.
# 0.20~0.30 정도부터 시험하는 것을 권장한다.
VISION_ERROR_FILTER_ALPHA = 0.22

# -------------------------------------------------------------------------
# 수평 속도 명령 변화율 제한 [m/s^2]
# -------------------------------------------------------------------------
# PID 출력이 프레임마다 급격히 바뀌더라도 실제 PX4에 보내는 속도 명령은
# 이 변화율 이상 급변하지 않게 한다. 접근 중 Roll/Pitch 출렁임 완화용이다.
APPROACH_XY_CMD_ACCEL_LIMIT_MPS2 = 0.65
FINAL_XY_CMD_ACCEL_LIMIT_MPS2 = 0.35

# FINE PI 단계에서는 속도 변화도 더 천천히 만든다.
FINE_XY_CMD_ACCEL_LIMIT_MPS2 = 0.20

# 3 m 근처에서는 하강속도를 더 줄여 수직 운동과 XY 보정의 결합을 완화한다.
APPROACH_SLOWDOWN_ALT_M = 5.0
NEAR_GROUND_MAX_DESCEND_SPEED_MPS = 0.25

# 이 시간 동안 새로운 /mission/target_info가 들어오지 않으면
# 표적 정보를 오래된 값으로 판단하고 수평/Yaw 제어를 정지한다.
TARGET_MSG_TIMEOUT_SEC = 0.40

# detected=False가 되면 즉시 하강과 수평 이동을 멈춘다.
HOLD_IF_TARGET_LOST = True


# -----------------------------------------------------------------------------
# 11. ROS2 토픽
# -----------------------------------------------------------------------------

TARGET_INFO_TOPIC = "/mission/target_info"
STATE_TOPIC = "/mavros/state"
EXTENDED_STATE_TOPIC = "/mavros/extended_state"
LOCAL_POSE_TOPIC = "/mavros/local_position/pose"
LOCAL_VELOCITY_TOPIC = "/mavros/local_position/velocity_local"
OPTICAL_FLOW_TOPIC = "/mavros/px4flow/raw/optical_flow_rad"

# MAVROS 속도 setpoint 인터페이스.
VELOCITY_SETPOINT_TOPIC = "/mavros/setpoint_velocity/cmd_vel"

# Phase2 -> GCS: 정렬 완료/착륙 준비 신호.
READY_FOR_LAND_TOPIC = "/mission/ready_for_land"

# GCS -> Phase2: 팝업에서 사용자가 Land/OK를 승인했음을 알리는 신호.
# GCS는 READY=True를 받은 뒤 사용자가 OK를 누르면 Bool(True)를 한 번 발행하면 된다.
LAND_CONFIRM_TOPIC = "/mission/land_confirm"


# -----------------------------------------------------------------------------
# 12. robo_jinheui.py의 target_info 인덱스
# -----------------------------------------------------------------------------
#
# robo_jinheui.py /mission/target_info Float32MultiArray 구성:
#   0  detected
#   1  center_x_px
#   2  center_y_px
#   3  error_x_px
#   4  error_y_px
#   5  error_x_norm
#   6  error_y_norm
#   7  bearing_x_rad
#   8  bearing_y_rad
#   ...
#   17 orientation_valid (= heading_valid)
#   18 heading_error_rad
#   19 heading_error_deg
# =============================================================================

IDX_DETECTED = 0
IDX_ERROR_X_PX = 3
IDX_ERROR_Y_PX = 4
IDX_ERROR_X_NORM = 5
IDX_ERROR_Y_NORM = 6
IDX_BEARING_X_RAD = 7
IDX_BEARING_Y_RAD = 8
IDX_ORIENTATION_VALID = 17
IDX_HEADING_ERROR_RAD = 18


MAV_VTOL_STATE_MC = 3
MAV_LANDED_STATE_ON_GROUND = 1


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def wrap_axis_angle_rad(angle_rad: float) -> float:
    """
    PCA 장축은 180° 대칭이므로 각도 오차를 [-90°, +90°] 범위로 정규화한다.

    예:
        +100° -> -80°
        -100° -> +80°

    이를 통해 0°/90° 목표 헤딩 모두 같은 방식으로 안정적으로 제어할 수 있다.
    """
    while angle_rad > math.pi / 2.0:
        angle_rad -= math.pi

    while angle_rad < -math.pi / 2.0:
        angle_rad += math.pi

    return angle_rad


def quaternion_to_yaw(q) -> float:
    """geometry_msgs Quaternion에서 ROS ENU yaw를 계산한다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class TargetSample:
    detected: bool = False
    error_x_px: float = 0.0
    error_y_px: float = 0.0
    error_x_norm: float = 0.0
    error_y_norm: float = 0.0
    bearing_x_rad: float = 0.0
    bearing_y_rad: float = 0.0
    orientation_valid: bool = False
    heading_error_rad: float = 0.0
    receive_monotonic: float = 0.0


class PID:
    """적분항 제한, D항 필터, 출력 제한이 포함된 PID 제어기."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float,
        output_limit: float,
        d_alpha: float,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.integral_limit = abs(float(integral_limit))
        self.output_limit = abs(float(output_limit))
        self.d_alpha = clamp(float(d_alpha), 0.0, 1.0)

        self.integral = 0.0
        self.prev_error = None
        self.prev_d = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_d = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 1e-6:
            return 0.0

        p = self.kp * error

        self.integral += error * dt
        self.integral = clamp(
            self.integral,
            -self.integral_limit,
            self.integral_limit,
        )
        i = self.ki * self.integral

        if self.prev_error is None:
            d_raw = 0.0
        else:
            d_raw = (error - self.prev_error) / dt

        self.prev_d = (
            self.d_alpha * d_raw
            + (1.0 - self.d_alpha) * self.prev_d
        )
        d = self.kd * self.prev_d

        self.prev_error = error

        output = p + i + d
        return clamp(output, -self.output_limit, self.output_limit)


class Phase2VisionAlignTo3m(Node):
    def __init__(self):
        super().__init__("phase2_vision_align_to_3m")

        self.state = None
        self.extended_state = None
        self.local_pose = None
        self.local_velocity = None
        self.target = TargetSample()
        self.vision_filter_initialized = False

        # Optical Flow 지면거리 상태.
        self.optical_flow_distance_m = None
        self.optical_flow_quality = 0
        self.optical_flow_receive_monotonic = 0.0

        # Local Z가 착륙장 오프셋을 반영한 3 m 목표 근처에 최초 진입하면 True.
        # 이후에는 Optical Flow가 유효할 때마다 지면거리 기준 3 m 제어를 우선한다.
        self.optical_flow_stage_enabled = False
        self.last_altitude_source = "LOCAL_Z"

        # PX4에 실제로 보낸 직전 수평 속도 명령.
        # PID 출력 변화가 급격해도 이 값을 기준으로 slew-rate를 제한한다.
        self.last_cmd_vx_east = 0.0
        self.last_cmd_vy_north = 0.0

        self.phase = "WAIT"
        self.ready_since = None  # legacy/manual-wait path compatibility
        self.ready_latched = False
        self.ready_frame_count = 0
        self.last_ready_vision_stamp = 0.0
        self.land_confirmed = False

        # OFFBOARD landing fallback reference (READY 확정 순간 저장).
        self.land_hold_x = None
        self.land_hold_y = None
        self.land_hold_yaw = None
        self.touchdown_since = None
        self.mission_complete = False

        self.last_control_monotonic = time.monotonic()
        self.last_log_monotonic = 0.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            State,
            STATE_TOPIC,
            self.state_callback,
            qos,
        )
        self.create_subscription(
            ExtendedState,
            EXTENDED_STATE_TOPIC,
            self.extended_state_callback,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            LOCAL_POSE_TOPIC,
            self.local_pose_callback,
            qos,
        )
        self.create_subscription(
            TwistStamped,
            LOCAL_VELOCITY_TOPIC,
            self.local_velocity_callback,
            qos,
        )
        self.create_subscription(
            OpticalFlowRad,
            OPTICAL_FLOW_TOPIC,
            self.optical_flow_callback,
            qos,
        )
        self.create_subscription(
            Float32MultiArray,
            TARGET_INFO_TOPIC,
            self.target_callback,
            10,
        )
        self.create_subscription(
            Bool,
            LAND_CONFIRM_TOPIC,
            self.land_confirm_callback,
            10,
        )

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            VELOCITY_SETPOINT_TOPIC,
            10,
        )
        self.ready_pub = self.create_publisher(
            Bool,
            READY_FOR_LAND_TOPIC,
            10,
        )

        self.set_mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode",
        )

        self.approach_x_pid = PID(
            APPROACH_X_KP,
            APPROACH_X_KI,
            APPROACH_X_KD,
            APPROACH_PID_I_LIMIT,
            APPROACH_MAX_XY_SPEED_MPS,
            PID_D_FILTER_ALPHA,
        )
        self.approach_y_pid = PID(
            APPROACH_Y_KP,
            APPROACH_Y_KI,
            APPROACH_Y_KD,
            APPROACH_PID_I_LIMIT,
            APPROACH_MAX_XY_SPEED_MPS,
            PID_D_FILTER_ALPHA,
        )

        self.final_x_pid = PID(
            FINAL_X_KP,
            FINAL_X_KI,
            FINAL_X_KD,
            FINAL_PID_I_LIMIT,
            FINAL_MAX_XY_SPEED_MPS,
            PID_D_FILTER_ALPHA,
        )
        self.final_y_pid = PID(
            FINAL_Y_KP,
            FINAL_Y_KI,
            FINAL_Y_KD,
            FINAL_PID_I_LIMIT,
            FINAL_MAX_XY_SPEED_MPS,
            PID_D_FILTER_ALPHA,
        )

        # 3 m 목표점 부근에서 사용하는 저게인 PI.
        # D항은 0으로 두어 영상 노이즈에 대한 민감도를 줄인다.
        self.fine_x_pi = PID(
            FINE_X_KP,
            FINE_X_KI,
            0.0,
            FINE_PI_I_LIMIT,
            FINE_MAX_XY_SPEED_MPS,
            PID_D_FILTER_ALPHA,
        )
        self.fine_y_pi = PID(
            FINE_Y_KP,
            FINE_Y_KI,
            0.0,
            FINE_PI_I_LIMIT,
            FINE_MAX_XY_SPEED_MPS,
            PID_D_FILTER_ALPHA,
        )

        # COARSE / FINE / DEADBAND 중 현재 최종 정렬 단계를 기록한다.
        self.final_xy_stage = "COARSE"

        # PI는 동일한 PID 클래스를 사용하되 kd=0으로 구현한다.
        self.yaw_pi = PID(
            YAW_KP,
            YAW_KI,
            0.0,
            integral_limit=YAW_I_LIMIT_RAD_S / max(YAW_KI, 1e-6),
            output_limit=math.radians(APPROACH_MAX_YAW_RATE_DEG_S),
            d_alpha=PID_D_FILTER_ALPHA,
        )

        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ,
            self.control_timer_callback,
        )

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def state_callback(self, msg: State):
        self.state = msg

    def extended_state_callback(self, msg: ExtendedState):
        self.extended_state = msg

    def local_pose_callback(self, msg: PoseStamped):
        self.local_pose = msg

    def local_velocity_callback(self, msg: TwistStamped):
        self.local_velocity = msg

    def optical_flow_callback(self, msg: OpticalFlowRad):
        # PX4FLOW/MAVROS OpticalFlowRad의 distance는 센서에서 지면까지의 거리 [m].
        self.optical_flow_distance_m = float(msg.distance)
        self.optical_flow_quality = int(msg.quality)
        self.optical_flow_receive_monotonic = time.monotonic()

    def land_confirm_callback(self, msg: Bool):
        """GCS 팝업에서 사용자가 Land/OK를 승인했을 때 착륙 시작을 latch한다."""
        if not bool(msg.data):
            return

        # READY가 확정되고 실제로 3 m 대기 상태일 때만 승인을 받아들인다.
        # READY 이전에 들어온 stale/오발행 True는 무시한다.
        if self.ready_latched and self.phase == "READY_WAIT_GCS":
            if not self.land_confirmed:
                self.get_logger().warn(
                    "[PHASE 2] GCS LAND CONFIRM received -> start OFFBOARD landing"
                )
            self.land_confirmed = True

    def target_callback(self, msg: Float32MultiArray):
        data = list(msg.data)

        if len(data) <= IDX_HEADING_ERROR_RAD:
            self.get_logger().warn(
                f"[PHASE 2] target_info too short: len={len(data)}"
            )
            return

        detected = bool(data[IDX_DETECTED] > 0.5)

        raw_error_x_px = float(data[IDX_ERROR_X_PX])
        raw_error_y_px = float(data[IDX_ERROR_Y_PX])
        raw_error_x_norm = float(data[IDX_ERROR_X_NORM])
        raw_error_y_norm = float(data[IDX_ERROR_Y_NORM])
        raw_bearing_x = float(data[IDX_BEARING_X_RAD])
        raw_bearing_y = float(data[IDX_BEARING_Y_RAD])

        # -------------------------------------------------------------
        # 영상 오차 1차 저역통과 필터
        #
        # 카메라 검출 중심이 프레임마다 몇 pixel씩 흔들리면 D항이 이를
        # 크게 증폭할 수 있다. 따라서 PID에 넣기 전에 위치/각도 오차를
        # 부드럽게 만든다.
        # -------------------------------------------------------------
        if not self.vision_filter_initialized or not detected:
            filt_error_x_px = raw_error_x_px
            filt_error_y_px = raw_error_y_px
            filt_error_x_norm = raw_error_x_norm
            filt_error_y_norm = raw_error_y_norm
            filt_bearing_x = raw_bearing_x
            filt_bearing_y = raw_bearing_y
            self.vision_filter_initialized = detected
        else:
            a = VISION_ERROR_FILTER_ALPHA

            filt_error_x_px = (
                a * raw_error_x_px
                + (1.0 - a) * self.target.error_x_px
            )
            filt_error_y_px = (
                a * raw_error_y_px
                + (1.0 - a) * self.target.error_y_px
            )
            filt_error_x_norm = (
                a * raw_error_x_norm
                + (1.0 - a) * self.target.error_x_norm
            )
            filt_error_y_norm = (
                a * raw_error_y_norm
                + (1.0 - a) * self.target.error_y_norm
            )
            filt_bearing_x = (
                a * raw_bearing_x
                + (1.0 - a) * self.target.bearing_x_rad
            )
            filt_bearing_y = (
                a * raw_bearing_y
                + (1.0 - a) * self.target.bearing_y_rad
            )

        self.target = TargetSample(
            detected=detected,
            error_x_px=filt_error_x_px,
            error_y_px=filt_error_y_px,
            error_x_norm=filt_error_x_norm,
            error_y_norm=filt_error_y_norm,
            bearing_x_rad=filt_bearing_x,
            bearing_y_rad=filt_bearing_y,
            orientation_valid=bool(data[IDX_ORIENTATION_VALID] > 0.5),
            heading_error_rad=float(data[IDX_HEADING_ERROR_RAD]),
            receive_monotonic=time.monotonic(),
        )

    # =========================================================================
    # BASIC HELPERS
    # =========================================================================

    def target_is_fresh(self) -> bool:
        if self.target.receive_monotonic <= 0.0:
            return False
        return (
            time.monotonic() - self.target.receive_monotonic
            <= TARGET_MSG_TIMEOUT_SEC
        )

    def target_is_usable(self) -> bool:
        return self.target_is_fresh() and self.target.detected

    def current_altitude_m(self):
        if self.local_pose is None:
            return None
        return float(self.local_pose.pose.position.z)

    def local_final_target_altitude_m(self) -> float:
        # Optical Flow가 없을 때 사용하는 Local Z fallback 목표.
        return float(FINAL_ALIGN_ALT_M + LANDING_GROUND_OFFSET_M)

    def optical_flow_is_fresh(self) -> bool:
        if self.optical_flow_receive_monotonic <= 0.0:
            return False
        return (
            time.monotonic() - self.optical_flow_receive_monotonic
            <= OPTICAL_FLOW_MSG_TIMEOUT_SEC
        )

    def optical_flow_is_usable(self) -> bool:
        if not self.optical_flow_is_fresh():
            return False

        if self.optical_flow_distance_m is None:
            return False

        distance = float(self.optical_flow_distance_m)

        if not math.isfinite(distance):
            return False

        if (
            distance < OPTICAL_FLOW_MIN_DISTANCE_M
            or distance > OPTICAL_FLOW_MAX_DISTANCE_M
        ):
            return False

        if self.optical_flow_quality < OPTICAL_FLOW_MIN_QUALITY:
            return False

        return True

    def altitude_control_state(self, local_alt_m: float):
        """
        반환:
            measured_alt_m : 현재 고도제어에 사용하는 측정값
            target_alt_m   : 해당 측정 기준 목표값
            source         : 'OPTICAL_FLOW' 또는 'LOCAL_Z'

        optical_flow_stage_enabled가 된 이후:
          - Optical Flow 정상 -> distance 기준 3 m
          - Optical Flow 비정상 -> Local Z 기준 3 m + 착륙장 오프셋
        """
        if self.optical_flow_stage_enabled and self.optical_flow_is_usable():
            return (
                float(self.optical_flow_distance_m),
                float(FINAL_ALIGN_ALT_M),
                "OPTICAL_FLOW",
            )

        return (
            float(local_alt_m),
            self.local_final_target_altitude_m(),
            "LOCAL_Z",
        )

    def current_yaw_rad(self):
        if self.local_pose is None:
            return None
        return quaternion_to_yaw(self.local_pose.pose.orientation)

    def reset_horizontal_controllers(self):
        self.approach_x_pid.reset()
        self.approach_y_pid.reset()
        self.final_x_pid.reset()
        self.final_y_pid.reset()
        self.fine_x_pi.reset()
        self.fine_y_pi.reset()
        self.final_xy_stage = "COARSE"

    def reset_yaw_controller(self):
        self.yaw_pi.reset()

    def desired_heading_error_rad(self) -> float:
        """
        현재 타겟 장축과 사용자가 원하는 최종 기체 헤딩 사이의 제어 오차.

        robo_jinheui.py의 heading_error_rad는 카메라/영상 기준 상대각이며,
        PCA 장축 특성상 180° 방향성은 구분하지 않는다.

        DESIRED_HEADING_OFFSET_DEG:
            0°  -> 타겟 장축과 평행
            90° -> 타겟 장축과 직각
        """
        desired_rad = math.radians(
            DESIRED_HEADING_OFFSET_DEG
        )

        return wrap_axis_angle_rad(
            self.target.heading_error_rad - desired_rad
        )

    # =========================================================================
    # 카메라 픽셀 오프셋 / Landing Zone
    # =========================================================================

    def desired_final_target_px(self):
        """
        최종 정렬 시 조난자가 위치해야 하는 영상 중심 기준 pixel 오프셋.

        robo_jinheui.py의 error_x_px/error_y_px 자체가 이미 영상 중심을 0으로 한
        상대 pixel 좌표이므로 별도의 영상 해상도 정보가 필요하지 않다.
        """
        return (
            float(LANDING_TARGET_OFFSET_X_PX),
            float(LANDING_TARGET_OFFSET_Y_PX),
        )

    def landing_zone_error_px(self):
        """
        현재 조난자 중심과 원하는 Landing Zone 중심 사이의 pixel 오차.
        """
        desired_x_px, desired_y_px = self.desired_final_target_px()

        dx_px = self.target.error_x_px - desired_x_px
        dy_px = self.target.error_y_px - desired_y_px

        return dx_px, dy_px

    def inside_landing_zone(self) -> bool:
        dx_px, dy_px = self.landing_zone_error_px()

        return (
            abs(dx_px) <= LANDING_ZONE_HALF_WIDTH_PX
            and abs(dy_px) <= LANDING_ZONE_HALF_HEIGHT_PX
        )

    def inside_ready_release_zone(self) -> bool:
        """
        READY 이후 상태 표시가 경계에서 계속 True/False로 떨리지 않도록
        Landing Zone보다 약간 큰 '해제 기준'만 별도로 둔다.

        실제 최종 정렬 목표 영역은 inside_landing_zone()의 크기 그대로다.
        """
        dx_px, dy_px = self.landing_zone_error_px()

        return (
            abs(dx_px)
            <= LANDING_ZONE_HALF_WIDTH_PX + READY_RELEASE_MARGIN_PX
            and abs(dy_px)
            <= LANDING_ZONE_HALF_HEIGHT_PX + READY_RELEASE_MARGIN_PX
        )

    def current_velocity_enu(self):
        if self.local_velocity is None:
            return None
        return (
            float(self.local_velocity.twist.linear.x),
            float(self.local_velocity.twist.linear.y),
            float(self.local_velocity.twist.linear.z),
        )

    def strict_ready_conditions(self, altitude: float, altitude_target: float):
        """엄격한 착륙 허가 조건과 세부 상태를 반환한다."""
        dx_px, dy_px = self.landing_zone_error_px()

        position_ok = (
            abs(dx_px) <= READY_STRICT_X_PX
            and abs(dy_px) <= READY_STRICT_Y_PX
        )

        if self.target.orientation_valid:
            heading_ok = (
                abs(math.degrees(self.desired_heading_error_rad()))
                <= READY_STRICT_HEADING_TOL_DEG
            )
        else:
            heading_ok = not REQUIRE_VALID_HEADING_FOR_READY

        altitude_ok = abs(altitude - altitude_target) <= READY_STRICT_ALT_TOL_M

        vel = self.current_velocity_enu()
        if vel is None:
            speed_ok = False
            xy_speed = float("inf")
            abs_vz = float("inf")
        else:
            vx, vy, vz = vel
            xy_speed = math.hypot(vx, vy)
            abs_vz = abs(vz)
            speed_ok = (
                xy_speed <= READY_MAX_XY_SPEED_MPS
                and abs_vz <= READY_MAX_ABS_VZ_MPS
            )

        return (
            position_ok and heading_ok and altitude_ok and speed_ok,
            position_ok,
            heading_ok,
            altitude_ok,
            speed_ok,
            xy_speed,
            abs_vz,
        )

    def capture_landing_reference(self):
        """READY 순간의 Local X/Y/Yaw를 Vision 소실 fallback 기준점으로 저장한다."""
        if self.local_pose is None:
            return False

        yaw = self.current_yaw_rad()
        if yaw is None:
            return False

        self.land_hold_x = float(self.local_pose.pose.position.x)
        self.land_hold_y = float(self.local_pose.pose.position.y)
        self.land_hold_yaw = float(yaw)
        return True

    def landing_height_agl_m(self, local_alt_m: float):
        """착륙 중 지면까지 높이. Optical Flow 우선, 없으면 Local Z fallback."""
        if self.optical_flow_is_usable():
            return max(0.0, float(self.optical_flow_distance_m)), "OPTICAL_FLOW"

        return max(0.0, float(local_alt_m - LANDING_GROUND_OFFSET_M)), "LOCAL_Z"

    def landing_vertical_speed_command(self, height_agl_m: float) -> float:
        """PX4 LAND와 유사하게 지면 접근 시 하강속도를 단계적으로 낮춘다."""
        if height_agl_m <= LAND_CRAWL_ALT_M:
            return -LAND_CRAWL_DESCEND_SPEED_MPS
        if height_agl_m <= LAND_SLOW_ALT_M:
            return -LAND_SLOW_DESCEND_SPEED_MPS
        return -LAND_DESCEND_SPEED_MPS

    def local_hold_xy_command(self):
        if (
            self.local_pose is None
            or self.land_hold_x is None
            or self.land_hold_y is None
        ):
            return 0.0, 0.0

        ex = self.land_hold_x - float(self.local_pose.pose.position.x)
        ey = self.land_hold_y - float(self.local_pose.pose.position.y)

        vx = clamp(
            LAND_LOCAL_XY_KP * ex,
            -LAND_LOCAL_XY_MAX_SPEED_MPS,
            LAND_LOCAL_XY_MAX_SPEED_MPS,
        )
        vy = clamp(
            LAND_LOCAL_XY_KP * ey,
            -LAND_LOCAL_XY_MAX_SPEED_MPS,
            LAND_LOCAL_XY_MAX_SPEED_MPS,
        )
        return vx, vy

    def local_hold_yaw_command(self, current_yaw: float) -> float:
        if self.land_hold_yaw is None:
            return 0.0

        err = wrap_pi(self.land_hold_yaw - current_yaw)
        limit = math.radians(LAND_LOCAL_YAW_MAX_RATE_DEG_S)
        return clamp(LAND_LOCAL_YAW_KP * err, -limit, limit)

    def touchdown_is_confirmed(self, now: float) -> bool:
        on_ground = (
            self.extended_state is not None
            and self.extended_state.landed_state == MAV_LANDED_STATE_ON_GROUND
        )

        if not on_ground:
            self.touchdown_since = None
            return False

        if self.touchdown_since is None:
            self.touchdown_since = now
            return False

        return (now - self.touchdown_since) >= TOUCHDOWN_CONFIRM_SEC

    # =========================================================================
    # CONTROL LAW
    # =========================================================================

    def body_velocity_to_local_enu(
        self,
        forward_mps: float,
        right_mps: float,
        yaw_rad: float,
    ):
        """
        기체 Body frame의 전방/오른쪽 속도를 ROS local ENU의 East/North 속도로 변환한다.

        yaw=0일 때:
            forward -> East
            right   -> South (-North)
        """
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)

        vx_east = forward_mps * c + right_mps * s
        vy_north = forward_mps * s - right_mps * c

        return vx_east, vy_north

    def altitude_velocity_command(
        self,
        measured_alt_m: float,
        target_alt_m: float,
        local_alt_m: float,
    ) -> float:
        error_z = target_alt_m - measured_alt_m

        vz = ALTITUDE_KP * error_z

        max_descend = MAX_DESCEND_SPEED_MPS

        # 속도 완화 판단은 local z가 아니라 실제 지면거리(Optical Flow)가
        # 사용 중이면 그 값을 우선한다.
        slowdown_alt_m = measured_alt_m if self.last_altitude_source == "OPTICAL_FLOW" else local_alt_m

        if slowdown_alt_m <= APPROACH_SLOWDOWN_ALT_M:
            max_descend = min(
                max_descend,
                NEAR_GROUND_MAX_DESCEND_SPEED_MPS,
            )

        return clamp(
            vz,
            -max_descend,
            MAX_CLIMB_SPEED_MPS,
        )

    def yaw_rate_command(self, final_align: bool, dt: float) -> float:
        if not self.target.orientation_valid:
            self.reset_yaw_controller()
            return 0.0

        # 타겟 장축 자체를 0°로 맞추는 것이 아니라,
        # DESIRED_HEADING_OFFSET_DEG에 지정한 목표 상대각으로 정렬한다.
        heading_error = self.desired_heading_error_rad()

        yaw_rate = YAW_CONTROL_SIGN * self.yaw_pi.update(
            heading_error,
            dt,
        )

        limit_deg_s = (
            FINAL_MAX_YAW_RATE_DEG_S
            if final_align
            else APPROACH_MAX_YAW_RATE_DEG_S
        )
        limit_rad_s = math.radians(limit_deg_s)

        return clamp(yaw_rate, -limit_rad_s, limit_rad_s)

    def approach_xy_command(self, dt: float):
        """
        3 m 이전 접근 단계:
        조난자를 카메라 중심 부근에 유지하되 픽셀 단위의 완벽한 중앙 정렬은 요구하지 않는다.
        """
        bx = self.target.bearing_x_rad
        by = self.target.bearing_y_rad

        # 접근 단계에서 불필요한 미세 움직임을 줄이기 위한 deadband.
        if abs(self.target.error_x_norm) <= APPROACH_DEADBAND_NORM:
            bx = 0.0
            self.approach_x_pid.reset()

        if abs(self.target.error_y_norm) <= APPROACH_DEADBAND_NORM:
            by = 0.0
            self.approach_y_pid.reset()

        # 실기 영상/짐벌 방향 차이는 상단 X/Y_CONTROL_SIGN으로 조정한다.
        right_cmd = (
            X_CONTROL_SIGN
            * self.approach_x_pid.update(bx, dt)
        )

        forward_cmd = (
            Y_CONTROL_SIGN
            * self.approach_y_pid.update(by, dt)
        )

        # HARD FOV 경계에 가까우면 해당 축을 최대 속도로 보정한다.
        # 필요한 방향으로 해당 축 제어 출력을 포화시킨다.
        if abs(self.target.error_x_norm) >= FOV_HARD_LIMIT_NORM:
            right_cmd = (
                X_CONTROL_SIGN
                * math.copysign(
                    APPROACH_MAX_XY_SPEED_MPS,
                    self.target.error_x_norm,
                )
            )

        if abs(self.target.error_y_norm) >= FOV_HARD_LIMIT_NORM:
            forward_cmd = (
                Y_CONTROL_SIGN
                * math.copysign(
                    APPROACH_MAX_XY_SPEED_MPS,
                    self.target.error_y_norm,
                )
            )

        return forward_cmd, right_cmd

    def final_xy_command(self, dt: float):
        """
        3 m 최종 정렬 수평 제어.

        단계:
            COARSE   : 기존 PID로 목표 주변까지 접근
            FINE     : 저게인 PI로 (0, +200 px)에 천천히 수렴
            DEADBAND : 목표점 ±25 px 안에서는 XY 명령 = 0

        READY 판정용 Landing Zone(±40 px)과 Deadband(±25 px)는 별개다.
        """
        error_x_px, error_y_px = self.landing_zone_error_px()

        abs_ex = abs(error_x_px)
        abs_ey = abs(error_y_px)

        # ------------------------------------------------------------
        # 1. 최종 Deadband
        # ------------------------------------------------------------
        inside_deadband = (
            abs_ex <= FINAL_DEADBAND_X_PX
            and abs_ey <= FINAL_DEADBAND_Y_PX
        )

        if inside_deadband:
            if self.final_xy_stage != "DEADBAND":
                # 영역 진입 시 기존 적분값을 모두 제거한다.
                self.final_x_pid.reset()
                self.final_y_pid.reset()
                self.fine_x_pi.reset()
                self.fine_y_pi.reset()

            self.final_xy_stage = "DEADBAND"
            return 0.0, 0.0

        # ------------------------------------------------------------
        # 2. COARSE <-> FINE 전환
        #
        # FINE에 한번 들어오면 EXIT 경계를 벗어나기 전까지 FINE을 유지한다.
        # 이를 통해 ±80 px 부근에서 PID/PI가 계속 바뀌는 것을 막는다.
        # ------------------------------------------------------------
        if self.final_xy_stage in ("FINE", "DEADBAND"):
            stay_fine = (
                abs_ex <= FINE_ALIGN_EXIT_X_PX
                and abs_ey <= FINE_ALIGN_EXIT_Y_PX
            )

            if stay_fine:
                if self.final_xy_stage == "DEADBAND":
                    # Deadband에서 다시 살짝 벗어난 경우
                    # 저게인 PI로 매우 부드럽게 복귀한다.
                    self.fine_x_pi.reset()
                    self.fine_y_pi.reset()

                self.final_xy_stage = "FINE"
            else:
                # FINE 영역을 크게 벗어나면 다시 COARSE PID.
                self.fine_x_pi.reset()
                self.fine_y_pi.reset()
                self.final_x_pid.reset()
                self.final_y_pid.reset()
                self.final_xy_stage = "COARSE"

        elif (
            abs_ex <= FINE_ALIGN_ENTER_X_PX
            and abs_ey <= FINE_ALIGN_ENTER_Y_PX
        ):
            # COARSE -> FINE 최초 진입.
            # 기존 PID의 적분/D 상태를 가져가지 않는다.
            self.final_x_pid.reset()
            self.final_y_pid.reset()
            self.fine_x_pi.reset()
            self.fine_y_pi.reset()
            self.final_xy_stage = "FINE"

        # ------------------------------------------------------------
        # 3. FINE PI
        # ------------------------------------------------------------
        if self.final_xy_stage == "FINE":
            right_cmd = (
                X_CONTROL_SIGN
                * self.fine_x_pi.update(error_x_px, dt)
            )
            forward_cmd = (
                Y_CONTROL_SIGN
                * self.fine_y_pi.update(error_y_px, dt)
            )

            return forward_cmd, right_cmd

        # ------------------------------------------------------------
        # 4. COARSE PID
        # ------------------------------------------------------------
        self.final_xy_stage = "COARSE"

        right_cmd = (
            X_CONTROL_SIGN
            * self.final_x_pid.update(error_x_px, dt)
        )
        forward_cmd = (
            Y_CONTROL_SIGN
            * self.final_y_pid.update(error_y_px, dt)
        )

        return forward_cmd, right_cmd

    def apply_xy_slew_rate(
        self,
        vx_east: float,
        vy_north: float,
        dt: float,
        final_align: bool,
    ):
        """
        수평 속도 명령의 프레임 간 변화율을 제한한다.

        카메라 PID 출력이 순간적으로 바뀌더라도 PX4에 전달되는 속도 명령이
        급변하지 않도록 하여 Roll/Pitch 출렁임을 줄인다.
        """
        if final_align:
            if self.final_xy_stage in ("FINE", "DEADBAND"):
                accel_limit = FINE_XY_CMD_ACCEL_LIMIT_MPS2
            else:
                accel_limit = FINAL_XY_CMD_ACCEL_LIMIT_MPS2
        else:
            accel_limit = APPROACH_XY_CMD_ACCEL_LIMIT_MPS2

        max_delta = accel_limit * max(dt, 1e-3)

        vx_limited = clamp(
            vx_east,
            self.last_cmd_vx_east - max_delta,
            self.last_cmd_vx_east + max_delta,
        )
        vy_limited = clamp(
            vy_north,
            self.last_cmd_vy_north - max_delta,
            self.last_cmd_vy_north + max_delta,
        )

        self.last_cmd_vx_east = vx_limited
        self.last_cmd_vy_north = vy_limited

        return vx_limited, vy_limited

    def control_ready_wait_gcs(
        self,
        dt: float,
        local_altitude: float,
        yaw: float,
        altitude: float,
        altitude_target: float,
    ):
        """
        READY=True 이후 GCS 승인을 기다리는 3 m 호버 상태.

        - Vision 제어를 더 이상 사용하지 않는다.
        - READY 순간 캡처한 Local X/Y/Yaw를 고정한다.
        - FINAL_ALIGN_ALT_M을 계속 유지한다.
        - /mission/land_confirm=True가 들어오면 다음 cycle부터 OFFBOARD_LANDING으로 전환한다.
        """
        vx_east, vy_north = self.local_hold_xy_command()
        vx_east, vy_north = self.apply_xy_slew_rate(
            vx_east, vy_north, dt, final_align=True
        )
        yaw_rate = self.local_hold_yaw_command(yaw)

        # Optical Flow가 유효하면 지면거리 FINAL_ALIGN_ALT_M 유지,
        # 아니면 기존 Local Z fallback 목표를 유지한다.
        vz_up = self.altitude_velocity_command(
            measured_alt_m=altitude,
            target_alt_m=altitude_target,
            local_alt_m=local_altitude,
        )

        # GCS 합의: 정렬이 해제되면 ready_for_land=False.
        # 대기 중 Vision은 제어에는 쓰지 않고 정렬 유효성 감시에만 사용한다.
        target_ok = self.target_is_usable()
        position_ok = target_ok and self.inside_ready_release_zone()

        if target_ok and self.target.orientation_valid:
            heading_ok = (
                abs(math.degrees(self.desired_heading_error_rad()))
                <= READY_RELEASE_HEADING_TOL_DEG
            )
        else:
            heading_ok = target_ok and (not REQUIRE_VALID_HEADING_FOR_READY)

        altitude_ok = abs(altitude - altitude_target) <= FINAL_ALIGN_ENTER_TOL_M

        vel = self.current_velocity_enu()
        if vel is None:
            speed_ok = False
        else:
            cur_vx, cur_vy, cur_vz = vel
            speed_ok = (
                math.hypot(cur_vx, cur_vy) <= READY_MAX_XY_SPEED_MPS + 0.05
                and abs(cur_vz) <= READY_MAX_ABS_VZ_MPS + 0.04
            )

        ready_still_valid = position_ok and heading_ok and altitude_ok and speed_ok

        if not ready_still_valid:
            self.get_logger().warn(
                "[PHASE 2] READY released while waiting GCS -> return FINAL_ALIGN"
            )
            self.ready_latched = False
            self.land_confirmed = False
            self.ready_frame_count = 0
            self.last_ready_vision_stamp = self.target.receive_monotonic
            self.phase = "FINAL_ALIGN"
            self.reset_horizontal_controllers()
            self.reset_yaw_controller()
            self.publish_ready(False)
            self.publish_zero_setpoint()
            return

        self.publish_ready(True)

        if self.land_confirmed:
            self.phase = "OFFBOARD_LANDING"
            self.touchdown_since = None
            self.reset_horizontal_controllers()
            self.reset_yaw_controller()
            # 승인 받은 동일 cycle에는 3m snapshot hold를 유지하고,
            # 다음 20 Hz cycle부터 수직 하강을 시작한다.
            self.publish_velocity_setpoint(vx_east, vy_north, 0.0, yaw_rate)
            return

        self.publish_velocity_setpoint(vx_east, vy_north, vz_up, yaw_rate)
        self.log_control(
            altitude=altitude,
            phase="READY_WAIT_GCS/SNAPSHOT_HOVER",
            target_ok=False,
            position_ok=True,
            heading_ok=True,
            ready_now=True,
            desired_px=self.desired_final_target_px(),
            v_cmd=(vx_east, vy_north, vz_up),
            yaw_rate=yaw_rate,
            descent_scale=0.0,
        )

    def control_offboard_landing(
        self,
        now: float,
        dt: float,
        local_altitude: float,
        yaw: float,
    ):
        """
        READY 순간에 저장한 Local X/Y/Yaw를 고정 기준점으로 사용해 착륙한다.

        중요한 점:
        - OFFBOARD_LANDING 진입 후에는 Vision XY/Yaw 제어를 다시 사용하지 않는다.
        - X/Y는 READY 순간 저장한 local 위치를 P 제어로 유지한다.
        - Yaw도 READY 순간 저장한 heading을 유지한다.
        - Z축만 Optical Flow/Local Z 기반 단계 하강한다.

        즉 FINAL_ALIGN_ALT_M에서 '정렬된 상태를 스냅샷'으로 저장한 뒤,
        그 스냅샷 위치/방향을 유지하면서 수직으로 내려오는 구조다.
        """
        if self.touchdown_is_confirmed(now):
            self.phase = "TOUCHDOWN"
            self.mission_complete = True
            self.publish_ready(False)
            self.publish_zero_setpoint()
            self.get_logger().warn(
                "[PHASE 2] TOUCHDOWN CONFIRMED -> OFFBOARD landing complete"
            )
            return

        # 착륙 중 Vision은 참고/로그에도 제어 입력으로 사용하지 않는다.
        # READY 순간 캡처한 Local X/Y/Yaw만 이용한다.
        vx_east, vy_north = self.local_hold_xy_command()
        vx_east, vy_north = self.apply_xy_slew_rate(
            vx_east, vy_north, dt, final_align=True
        )
        yaw_rate = self.local_hold_yaw_command(yaw)

        height_agl, height_source = self.landing_height_agl_m(local_altitude)
        self.last_altitude_source = height_source
        vz_up = self.landing_vertical_speed_command(height_agl)

        # READY는 착륙 시작 허가가 이미 확정됐다는 latch 의미로 touchdown 전까지 유지한다.
        self.publish_ready(True)
        self.publish_velocity_setpoint(vx_east, vy_north, vz_up, yaw_rate)

        self.log_control(
            altitude=height_agl,
            phase="OFFBOARD_LANDING/SNAPSHOT_HOLD",
            target_ok=False,
            position_ok=True,
            heading_ok=True,
            ready_now=True,
            desired_px=self.desired_final_target_px(),
            v_cmd=(vx_east, vy_north, vz_up),
            yaw_rate=yaw_rate,
            descent_scale=1.0,
        )

    # =========================================================================
    # SETPOINT / MODE
    # =========================================================================

    def publish_velocity_setpoint(
        self,
        vx_east: float,
        vy_north: float,
        vz_up: float,
        yaw_rate_rad_s: float,
    ):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.twist.linear.x = float(vx_east)
        msg.twist.linear.y = float(vy_north)
        msg.twist.linear.z = float(vz_up)

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(yaw_rate_rad_s)

        self.velocity_pub.publish(msg)

    def publish_zero_setpoint(self):
        self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)

    def publish_ready(self, ready: bool):
        msg = Bool()
        msg.data = bool(ready)
        self.ready_pub.publish(msg)

    def wait_service(self, timeout_sec=5.0) -> bool:
        return self.set_mode_client.wait_for_service(timeout_sec=timeout_sec)

    def request_mode(self, mode_name: str) -> bool:
        if not self.wait_service():
            self.get_logger().error(
                "[PHASE 2] /mavros/set_mode unavailable"
            )
            return False

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_name

        future = self.set_mode_client.call_async(req)
        start = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

            # 서비스 응답을 기다리는 동안에도 OFFBOARD setpoint 송신을 유지한다.
            self.publish_zero_setpoint()

            if future.done():
                try:
                    result = future.result()
                except Exception as exc:
                    self.get_logger().error(
                        f"[PHASE 2] set_mode exception: {exc}"
                    )
                    return False

                return bool(result is not None and result.mode_sent)

            if time.monotonic() - start > 5.0:
                self.get_logger().error(
                    f"[PHASE 2] set_mode {mode_name} timeout"
                )
                return False

        return False

    # =========================================================================
    # STARTUP
    # =========================================================================

    def wait_initial_data(self, timeout_sec=15.0) -> bool:
        self.get_logger().info(
            "[PHASE 2] Waiting for MAVROS state / local pose / cam target"
        )

        start = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            state_ok = self.state is not None and self.state.connected
            pose_ok = self.local_pose is not None
            vel_ok = self.local_velocity is not None
            ext_ok = self.extended_state is not None
            target_rx_ok = self.target.receive_monotonic > 0.0

            if state_ok and pose_ok and vel_ok and ext_ok and target_rx_ok:
                self.get_logger().info(
                    f"[PHASE 2] Initial data OK | "
                    f"mode={self.state.mode} | "
                    f"armed={self.state.armed} | "
                    f"alt={self.current_altitude_m():.2f} m | "
                    f"detected={self.target.detected}"
                )
                return True

            if time.monotonic() - start > timeout_sec:
                self.get_logger().error(
                    "[PHASE 2] Initial data timeout | "
                    f"state={state_ok} pose={pose_ok} vel={vel_ok} ext={ext_ok} "
                    f"target_rx={target_rx_ok}"
                )
                return False

        return False

    def enter_offboard(self) -> bool:
        self.get_logger().info(
            "[PHASE 2] Pre-streaming zero-velocity setpoints before OFFBOARD"
        )

        end = time.monotonic() + OFFBOARD_PRESTREAM_SEC

        while rclpy.ok() and time.monotonic() < end:
            self.publish_zero_setpoint()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(1.0 / CONTROL_RATE_HZ)

        start = time.monotonic()
        last_request = 0.0

        while rclpy.ok():
            self.publish_zero_setpoint()
            rclpy.spin_once(self, timeout_sec=0.02)

            if self.state is not None and self.state.mode == "OFFBOARD":
                self.get_logger().info(
                    "[PHASE 2] OFFBOARD confirmed"
                )
                return True

            now = time.monotonic()

            if now - last_request >= OFFBOARD_REQUEST_INTERVAL_SEC:
                self.get_logger().info(
                    "[PHASE 2] Request OFFBOARD"
                )
                self.request_mode("OFFBOARD")
                last_request = time.monotonic()

            if now - start > OFFBOARD_ENTRY_TIMEOUT_SEC:
                current = self.state.mode if self.state else "UNKNOWN"
                self.get_logger().error(
                    f"[PHASE 2] OFFBOARD entry timeout. current={current}"
                )
                return False

            time.sleep(1.0 / CONTROL_RATE_HZ)

        return False

    # =========================================================================
    # MAIN TIMER
    # =========================================================================

    def control_timer_callback(self):
        # run()이 초기화를 담당하며 phase가 활성화된 뒤에만 제어한다.
        active_phases = (
            "APPROACH",
            "FINAL_ALIGN",
            "READY_WAIT_GCS",
            "OFFBOARD_LANDING",
            "TOUCHDOWN",
        )
        if self.phase not in active_phases:
            return

        now = time.monotonic()
        dt = clamp(now - self.last_control_monotonic, 0.01, 0.20)
        self.last_control_monotonic = now

        if self.state is None or self.local_pose is None:
            return

        # 다른 모드가 실제로 선택된 뒤에는 새 모드와 setpoint 경쟁을 하지 않는다.
        if self.state.mode != "OFFBOARD":
            return

        if self.phase == "TOUCHDOWN":
            self.publish_ready(False)
            self.publish_zero_setpoint()
            return

        local_altitude = self.current_altitude_m()
        yaw = self.current_yaw_rad()
        if local_altitude is None or yaw is None:
            return

        # 착륙장 고도 오프셋을 반영한 Local Z 목표 근처에 최초 진입하면
        # Optical Flow 지면거리 사용을 활성화한다.
        local_final_target = self.local_final_target_altitude_m()
        if (
            not self.optical_flow_stage_enabled
            and abs(local_altitude - local_final_target)
            <= OPTICAL_FLOW_ENABLE_TOL_M
        ):
            self.optical_flow_stage_enabled = True
            if self.optical_flow_is_usable():
                self.get_logger().warn(
                    "[PHASE 2] Optical Flow altitude control ENABLED | "
                    f"local_z={local_altitude:.2f} m | "
                    f"distance={self.optical_flow_distance_m:.2f} m | "
                    f"quality={self.optical_flow_quality}"
                )
            else:
                self.get_logger().warn(
                    "[PHASE 2] Optical Flow stage ENABLED but data is not usable. "
                    "Using Local Z fallback."
                )

        # READY 이후 내부 OFFBOARD landing 상태는 별도 제어기로 처리한다.
        if self.phase == "OFFBOARD_LANDING":
            self.control_offboard_landing(now, dt, local_altitude, yaw)
            return

        altitude, altitude_target, altitude_source = self.altitude_control_state(
            local_altitude
        )
        if altitude_source != self.last_altitude_source:
            self.get_logger().warn(
                "[PHASE 2] Altitude source changed: "
                f"{self.last_altitude_source} -> {altitude_source}"
            )
            self.last_altitude_source = altitude_source

        # READY 확정 후에는 Vision과 무관하게 snapshot 위치/방향 + 최종 고도를 유지하며
        # GCS의 /mission/land_confirm=True를 기다린다.
        if self.phase == "READY_WAIT_GCS":
            self.control_ready_wait_gcs(
                dt, local_altitude, yaw, altitude, altitude_target
            )
            return

        target_ok = self.target_is_usable()
        if not target_ok:
            self.reset_horizontal_controllers()
            self.reset_yaw_controller()
            self.last_cmd_vx_east = 0.0
            self.last_cmd_vy_north = 0.0
            self.ready_since = None
            self.ready_frame_count = 0
            self.last_ready_vision_stamp = 0.0
            if not self.ready_latched:
                self.publish_ready(False)
            self.publish_zero_setpoint()
            self.log_control(
                altitude=altitude,
                phase=self.phase,
                target_ok=False,
                position_ok=False,
                heading_ok=False,
                ready_now=self.ready_latched,
                desired_px=(0.0, 0.0),
                v_cmd=(0.0, 0.0, 0.0),
                yaw_rate=0.0,
                descent_scale=0.0,
            )
            return

        # 3 m 최종 정렬 진입.
        if (
            self.phase == "APPROACH"
            and self.optical_flow_stage_enabled
            and abs(altitude - altitude_target) <= FINAL_ALIGN_ENTER_TOL_M
        ):
            self.phase = "FINAL_ALIGN"
            self.reset_horizontal_controllers()
            self.reset_yaw_controller()
            self.ready_since = None
            self.ready_frame_count = 0
            self.last_ready_vision_stamp = 0.0
            self.get_logger().warn(
                "[PHASE 2] Final altitude reached -> FINAL_ALIGN | "
                f"source={altitude_source} measured={altitude:.2f} m "
                f"target={altitude_target:.2f} m"
            )

        final_align = self.phase in ("FINAL_ALIGN", "READY_WAIT_GCS")

        if final_align:
            forward_cmd, right_cmd = self.final_xy_command(dt)
        else:
            forward_cmd, right_cmd = self.approach_xy_command(dt)

        vx_east, vy_north = self.body_velocity_to_local_enu(
            forward_cmd, right_cmd, yaw
        )
        vx_east, vy_north = self.apply_xy_slew_rate(
            vx_east, vy_north, dt, final_align
        )

        vz_up = self.altitude_velocity_command(
            measured_alt_m=altitude,
            target_alt_m=altitude_target,
            local_alt_m=local_altitude,
        )

        # 3 m 이전 FOV 오차에 따른 연속 하강속도 조절 기능 유지.
        descent_scale = 1.0
        if not final_align and vz_up < 0.0:
            fov_error = max(
                abs(self.target.error_x_norm),
                abs(self.target.error_y_norm),
            )
            if fov_error <= DESCENT_FULL_SPEED_FOV_NORM:
                descent_scale = 1.0
            elif fov_error >= FOV_HARD_LIMIT_NORM:
                descent_scale = DESCENT_MIN_SCALE
            else:
                ratio = (
                    (fov_error - DESCENT_FULL_SPEED_FOV_NORM)
                    / (FOV_HARD_LIMIT_NORM - DESCENT_FULL_SPEED_FOV_NORM)
                )
                descent_scale = 1.0 - ratio * (1.0 - DESCENT_MIN_SCALE)

            descent_scale = clamp(descent_scale, DESCENT_MIN_SCALE, 1.0)
            vz_up *= descent_scale

        yaw_rate = self.yaw_rate_command(final_align=final_align, dt=dt)

        position_ok = False
        heading_ok = False
        ready_now = self.ready_latched
        desired_px = self.desired_final_target_px()

        if final_align and not self.ready_latched:
            (
                conditions_ok,
                position_ok,
                heading_ok,
                altitude_ok,
                speed_ok,
                xy_speed,
                abs_vz,
            ) = self.strict_ready_conditions(altitude, altitude_target)

            # 같은 vision sample을 timer가 두 번 읽어도 frame count를 올리지 않는다.
            is_new_vision_frame = (
                self.target.receive_monotonic > self.last_ready_vision_stamp
            )

            if conditions_ok and is_new_vision_frame:
                self.ready_frame_count += 1
                self.last_ready_vision_stamp = self.target.receive_monotonic
            elif not conditions_ok:
                self.ready_frame_count = 0
                self.last_ready_vision_stamp = self.target.receive_monotonic

            if self.ready_frame_count >= READY_REQUIRED_FRAMES:
                self.ready_latched = True
                ready_now = True

                if not self.capture_landing_reference():
                    self.get_logger().error(
                        "[PHASE 2] READY detected but failed to capture Local X/Y/Yaw"
                    )
                    self.ready_latched = False
                    self.ready_frame_count = 0
                else:
                    self.publish_ready(True)
                    self.get_logger().warn(
                        "[PHASE 2] READY FOR LAND LATCHED | "
                        f"frames={self.ready_frame_count} | "
                        f"xy_speed={xy_speed:.3f} m/s | vz={abs_vz:.3f} m/s | "
                        f"hold=({self.land_hold_x:.2f}, {self.land_hold_y:.2f})"
                    )

                    # READY=True는 GCS 팝업 표시용 준비 완료 신호다.
                    # 여기서는 절대 하강을 시작하지 않고 snapshot 위치/방향/고도를 유지한다.
                    # 실제 하강은 GCS가 LAND_CONFIRM_TOPIC에 Bool(True)를 보낸 뒤 시작한다.
                    self.land_confirmed = False
                    self.phase = "READY_WAIT_GCS"
                    self.reset_horizontal_controllers()
                    self.reset_yaw_controller()
                    # READY가 된 동일 cycle에는 정지 setpoint를 넣고,
                    # 다음 cycle부터 snapshot hover controller가 3 m를 유지한다.
                    self.publish_zero_setpoint()
                    return

        elif self.ready_latched:
            # READY_WAIT_GCS는 위에서 별도 snapshot-hover controller가 처리한다.
            # 이 분기는 방어적으로 READY latch 상태만 유지한다.
            ready_now = True

        self.publish_ready(ready_now)
        self.publish_velocity_setpoint(vx_east, vy_north, vz_up, yaw_rate)
        self.log_control(
            altitude=altitude,
            phase=self.phase,
            target_ok=True,
            position_ok=position_ok,
            heading_ok=heading_ok,
            ready_now=ready_now,
            desired_px=desired_px,
            v_cmd=(vx_east, vy_north, vz_up),
            yaw_rate=yaw_rate,
            descent_scale=descent_scale,
        )

    def log_control(
        self,
        altitude,
        phase,
        target_ok,
        position_ok,
        heading_ok,
        ready_now,
        desired_px,
        v_cmd,
        yaw_rate,
        descent_scale,
    ):
        now = time.monotonic()

        if now - self.last_log_monotonic < 0.5:
            return

        self.last_log_monotonic = now

        raw_heading_deg = (
            math.degrees(self.target.heading_error_rad)
            if self.target.orientation_valid
            else float("nan")
        )

        control_heading_error_deg = (
            math.degrees(self.desired_heading_error_rad())
            if self.target.orientation_valid
            else float("nan")
        )

        self.get_logger().info(
            "[PHASE 2] "
            f"phase={phase} | "
            f"alt_ctrl={altitude:.2f}m | "
            f"alt_source={self.last_altitude_source} | "
            f"target={target_ok} | "
            f"norm=({self.target.error_x_norm:+.3f},"
            f"{self.target.error_y_norm:+.3f}) | "
            f"px=({self.target.error_x_px:+.1f},"
            f"{self.target.error_y_px:+.1f}) | "
            f"desired_px=({desired_px[0]:+.1f},"
            f"{desired_px[1]:+.1f}) | "
            f"heading_raw={raw_heading_deg:+.1f}deg | "
            f"heading_target={DESIRED_HEADING_OFFSET_DEG:+.1f}deg | "
            f"heading_ctrl_err={control_heading_error_deg:+.1f}deg | "
            f"pos_ok={position_ok} heading_ok={heading_ok} "
            f"ready={ready_now} | "
            f"xy_stage={self.final_xy_stage if phase != 'APPROACH' else 'APPROACH'} | "
            f"v_enu=({v_cmd[0]:+.2f},"
            f"{v_cmd[1]:+.2f},"
            f"{v_cmd[2]:+.2f})m/s | "
            f"descent_scale={descent_scale:.2f} | "
            f"yaw_rate={math.degrees(yaw_rate):+.1f}deg/s"
        )

    # =========================================================================
    # RUN
    # =========================================================================

    def run(self) -> bool:
        self.get_logger().info(
            "[PHASE 2] REAL FLIGHT: vision approach -> snapshot READY -> 3m GCS wait -> OFFBOARD vertical landing"
        )
        self.get_logger().info(
            "[PHASE 2] Control signs: "
            f"X={X_CONTROL_SIGN:+.0f}, Y={Y_CONTROL_SIGN:+.0f}, "
            f"YAW={YAW_CONTROL_SIGN:+.0f}"
        )
        self.get_logger().info(
            "[PHASE 2] READY strict: "
            f"px=±({READY_STRICT_X_PX:.0f},{READY_STRICT_Y_PX:.0f}) | "
            f"alt=±{READY_STRICT_ALT_TOL_M:.2f}m | "
            f"heading=±{READY_STRICT_HEADING_TOL_DEG:.1f}deg | "
            f"Vxy<={READY_MAX_XY_SPEED_MPS:.2f} | "
            f"|Vz|<={READY_MAX_ABS_VZ_MPS:.2f} | "
            f"frames={READY_REQUIRED_FRAMES}"
        )
        self.get_logger().info(
            "[PHASE 2] GCS landing confirmation topic: "
            f"{LAND_CONFIRM_TOPIC}"
        )

        if not self.wait_initial_data():
            return False

        if self.state is None or not self.state.armed:
            self.get_logger().error(
                "[PHASE 2] Vehicle must already be ARMED from Phase 1."
            )
            return False

        if (
            self.extended_state is None
            or self.extended_state.vtol_state != MAV_VTOL_STATE_MC
        ):
            current = (
                self.extended_state.vtol_state
                if self.extended_state is not None
                else -1
            )
            self.get_logger().error(
                f"[PHASE 2] Vehicle must be MULTICOPTER. vtol_state={current}"
            )
            return False

        if not self.enter_offboard():
            return False

        self.phase = "APPROACH"
        self.last_control_monotonic = time.monotonic()
        self.get_logger().info(
            "[PHASE 2] APPROACH active: FOV retention has priority over descent"
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.mission_complete:
                self.publish_ready(False)
                self.get_logger().warn(
                    "[PHASE 2] Mission complete: touchdown confirmed"
                )
                return True

            if self.state is None:
                continue

            if self.state.mode != "OFFBOARD":
                # 내부 OFFBOARD landing을 쓰는 도중 외부에서 AUTO.LAND 등을 걸면
                # 즉시 setpoint 경쟁을 중단하고 제어권을 넘긴다.
                if self.ready_latched:
                    self.get_logger().warn(
                        f"[PHASE 2] External mode handoff after READY: "
                        f"{self.state.mode}. OFFBOARD output released."
                    )
                    self.phase = "HANDOFF_COMPLETE"
                    self.publish_ready(False)
                    return True

                self.get_logger().error(
                    f"[PHASE 2] OFFBOARD lost before READY_FOR_LAND. "
                    f"current={self.state.mode}"
                )
                self.phase = "ABORTED"
                self.publish_ready(False)
                return False

        return False

def main():
    rclpy.init()
    node = Phase2VisionAlignTo3m()

    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn(
            "[PHASE 2] Interrupted by user"
        )
        ok = False
    finally:
        # 내부 OFFBOARD landing이든 외부 mode handoff든 종료 시 READY는 False로 정리한다.
        if rclpy.ok():
            try:
                node.publish_ready(False)
                node.publish_zero_setpoint()
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
