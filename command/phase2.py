"""
phase2_visual_align_to_3m.py

목적
----
조난자/착륙 표적을 이용한 Phase 2 비전 유도 제어기.

이 노드는 LAND 모드를 직접 인가하지 않는다.

동작 순서
---------
    1) VTOL이 ARMED, MULTICOPTER 상태이며 보통 AUTO.LOITER 상태에서 시작한다.
    2) cam.py가 발행하는 /mission/target_info를 구독한다.
    3) 속도/yaw-rate setpoint를 미리 송신한 뒤 OFFBOARD 모드로 진입한다.
    4) 고도 3 m 이전:
         - 조난자가 카메라 FOV 밖으로 나가지 않도록 유지한다.
         - 수평 위치는 PID로 보정한다.
         - 조난자 방향 정보가 유효하면 Yaw는 PI로 정렬한다.
         - 3 m를 향해 하강한다.
    5) 고도 3 m:
         - 고도를 유지한다.
         - 기체 코 쪽 카메라 오프셋을 pixel 기준 Landing Zone으로 보상한다.
         - 조난자를 비교적 좁은 Landing Zone 안으로 유도한다.
         - Yaw는 너무 엄격하지 않은 허용각 안으로 정렬한다.
    6) 위치 + 방향 조건을 짧은 시간 동안 만족하면:
         - READY_FOR_LAND = True를 발행한다.
         - LAND는 수행하지 않는다.
         - 3 m 고도와 정렬 상태를 계속 유지한다.
    7) 이후 GCS에서 LAND 등의 모드를 직접 인가한다.
       OFFBOARD가 해제되면 본 노드는 제어권을 넘기고 종료한다.

제어 구조
---------
외부 제어기에서 생성하는 명령:
    - 비전 PID 기반 수평 속도 명령
    - 3 m 접근/유지를 위한 수직 속도 명령
    - 비전 PI 기반 yaw-rate 명령

PX4 내부 멀티콥터 제어기가 수평 속도 명령을 실제 Roll/Pitch 명령으로 변환한다.
따라서 이 노드는 Raw Roll/Pitch/Thrust를 직접 명령하지 않으면서도,
최종 XY 정렬 과정에서 Roll/Pitch를 자연스럽게 사용하게 된다.

좌표계 가정
-----------
cam.py 기준:
    영상 위쪽    = 기체 전방
    영상 오른쪽  = 기체 오른쪽

따라서:
    표적이 영상 오른쪽에 있음 -> 기체를 오른쪽으로 이동
    표적이 영상 아래쪽에 있음 -> 표적이 기체 뒤쪽에 있으므로 기체를 뒤로 이동

/mavros/local_position/pose는 ROS ENU 좌표계로 가정:
    x = East, y = North, z = Up

짐벌은 거의 수직 아래를 바라보고 있다고 가정한다.
"""

import math
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
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
# 2. 최종 정렬 고도
# -----------------------------------------------------------------------------

# 최종 정렬 고도 [m]. /mavros/local_position/pose의 z 값을 사용한다.
# local z=0이 착륙 지면과 거의 일치한다고 가정한다.
FINAL_ALIGN_ALT_M = 3.0

# 현재 고도가 3 m 근처에 들어오면 FINAL_ALIGN 단계로 전환한다.
FINAL_ALIGN_ENTER_TOL_M = 0.20

# 수직 접근 및 3 m 고도 유지를 위한 P 제어기:
#   vz_cmd = ALTITUDE_KP * (FINAL_ALIGN_ALT_M - current_z)
ALTITUDE_KP = 0.65

# 최대 수직 속도 [m/s].
# ROS ENU 기준: +z는 상승, -z는 하강.
MAX_DESCEND_SPEED_MPS = 2.00
MAX_CLIMB_SPEED_MPS = 0.80

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
# 3. 카메라 오프셋 - 픽셀 기준
# -----------------------------------------------------------------------------
#
# 실제 시스템에서는 카메라 장착 오프셋을 '미터'로 환산하지 않고,
# 화면 중심으로부터 몇 pixel 떨어진 위치에 조난자가 와야 하는지 직접 지정한다.
#
# cam.py에서 이미
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
#   -> 최종 정렬 시 조난자 중심을 영상 중심보다 95 px 아래에 위치시킨다.
#
# 아래 값은 반드시 실제 기체/카메라 화면을 보고 조정할 것.
LANDING_TARGET_OFFSET_X_PX = 0.0
LANDING_TARGET_OFFSET_Y_PX = 200.0


# -----------------------------------------------------------------------------
# 4. 3 m 최종 Landing Zone - 픽셀 기준
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
# 조난자 중심이 이 영역 안에 들어오면
# FINAL_STOP_XY_INSIDE_LANDING_ZONE=True에 의해
# XY PID 출력을 0으로 만들어 더 이상 목표점 (0, +200)을
# 정밀하게 쫓지 않는다. 3 m에서의 앞뒤/좌우 출렁임 억제 목적이다.

# Yaw는 PCA/영상 노이즈 때문에 완벽한 0도를 요구하지 않는다.
READY_HEADING_TOL_DEG = 12.0

# 위치 + Yaw + 3 m 조건은 짧게만 유지해도 READY로 판정한다.
READY_STABLE_TIME_SEC = 0.70

# READY가 한번 성립한 뒤 경계에서 True/False가 계속 흔들리는 것을 막는
# 히스테리시스 여유값이다.
# 주의: 이 값은 Landing Zone 자체를 넓히는 값이 아니다.
READY_RELEASE_MARGIN_PX = 20.0
READY_RELEASE_HEADING_TOL_DEG = 18.0

# True이면 heading 추정이 유효하지 않을 경우 READY를 허용하지 않는다.
REQUIRE_VALID_HEADING_FOR_READY = True


# -----------------------------------------------------------------------------
# 5. 3 m 이전 FOV 유지
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
# 6. 수평 PID - 접근 단계 (> 3 m)
# -----------------------------------------------------------------------------
#
# PID 입력은 cam.py의 bearing_x/bearing_y 각도 오차 [rad]이다.
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
# 7. 3 m 최종 수평 정렬: PID -> 저게인 PI -> Deadband
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
# 8. Yaw PI 제어
# -----------------------------------------------------------------------------

# cam.py의 heading_error_rad는 PCA 기반 조난자 장축 추정값이다.
YAW_KP = 1.20
YAW_KI = 0.10
YAW_I_LIMIT_RAD_S = math.radians(8.0)

# 최대 yaw-rate 제한.
APPROACH_MAX_YAW_RATE_DEG_S = 20.0
FINAL_MAX_YAW_RATE_DEG_S = 12.0

# cam.py heading_error와 기체 yaw-rate 사이의 부호 매핑.
#
# cam.py에서 양의 heading error는 영상 오른쪽 방향이다.
# ROS ENU에서 양의 yaw는 반시계 방향이다.
# 따라서 현재 영상 좌표 규약에서는 초기값 -1.0을 사용한다.
#
# 시뮬레이션에서 반대 방향으로 회전하면 +1.0으로 바꾼다.
YAW_CONTROL_SIGN = -1.0


# -----------------------------------------------------------------------------
# 9. PID D항 필터 / 표적 데이터 타임아웃
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
# 10. ROS2 토픽
# -----------------------------------------------------------------------------

TARGET_INFO_TOPIC = "/mission/target_info"
STATE_TOPIC = "/mavros/state"
EXTENDED_STATE_TOPIC = "/mavros/extended_state"
LOCAL_POSE_TOPIC = "/mavros/local_position/pose"

# MAVROS 속도 setpoint 인터페이스.
VELOCITY_SETPOINT_TOPIC = "/mavros/setpoint_velocity/cmd_vel"

# GCS 또는 모니터링용 READY 신호.
READY_FOR_LAND_TOPIC = "/mission/ready_for_land"


# -----------------------------------------------------------------------------
# 11. cam.py의 target_info 인덱스
# -----------------------------------------------------------------------------
#
# /mission/target_info Float32MultiArray 구성:
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
#   17 orientation_valid
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


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
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
        self.target = TargetSample()
        self.vision_filter_initialized = False

        # PX4에 실제로 보낸 직전 수평 속도 명령.
        # PID 출력 변화가 급격해도 이 값을 기준으로 slew-rate를 제한한다.
        self.last_cmd_vx_east = 0.0
        self.last_cmd_vy_north = 0.0

        self.phase = "WAIT"
        self.ready_since = None
        self.ready_latched = False

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
            Float32MultiArray,
            TARGET_INFO_TOPIC,
            self.target_callback,
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

    # =========================================================================
    # 카메라 픽셀 오프셋 / Landing Zone
    # =========================================================================

    def desired_final_target_px(self):
        """
        최종 정렬 시 조난자가 위치해야 하는 영상 중심 기준 pixel 오프셋.

        cam.py의 error_x_px/error_y_px 자체가 이미 영상 중심을 0으로 한
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

    def altitude_velocity_command(self, current_alt_m: float) -> float:
        error_z = FINAL_ALIGN_ALT_M - current_alt_m

        vz = ALTITUDE_KP * error_z

        max_descend = MAX_DESCEND_SPEED_MPS

        if current_alt_m <= APPROACH_SLOWDOWN_ALT_M:
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

        heading_error = wrap_pi(self.target.heading_error_rad)

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

        # 표적이 영상 오른쪽이면 기체를 오른쪽으로 이동한다.
        right_cmd = self.approach_x_pid.update(bx, dt)

        # 표적이 영상 중심 아래쪽이면 기체 기준 뒤쪽이므로 기체를 뒤로 이동한다.
        forward_cmd = -self.approach_y_pid.update(by, dt)

        # HARD FOV 경계에 가까우면 해당 축을 최대 속도로 보정한다.
        # 필요한 방향으로 해당 축 제어 출력을 포화시킨다.
        if abs(self.target.error_x_norm) >= FOV_HARD_LIMIT_NORM:
            right_cmd = math.copysign(
                APPROACH_MAX_XY_SPEED_MPS,
                self.target.error_x_norm,
            )

        if abs(self.target.error_y_norm) >= FOV_HARD_LIMIT_NORM:
            forward_cmd = -math.copysign(
                APPROACH_MAX_XY_SPEED_MPS,
                self.target.error_y_norm,
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
            # 표적이 목표보다 오른쪽(+x)이면 기체를 오른쪽으로 이동.
            right_cmd = self.fine_x_pi.update(error_x_px, dt)

            # 표적이 목표보다 아래(+y)이면 기체 기준 뒤쪽이므로 뒤로 이동.
            forward_cmd = -self.fine_y_pi.update(error_y_px, dt)

            return forward_cmd, right_cmd

        # ------------------------------------------------------------
        # 4. COARSE PID
        # ------------------------------------------------------------
        self.final_xy_stage = "COARSE"

        right_cmd = self.final_x_pid.update(error_x_px, dt)
        forward_cmd = -self.final_y_pid.update(error_y_px, dt)

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
            ext_ok = self.extended_state is not None
            target_rx_ok = self.target.receive_monotonic > 0.0

            if state_ok and pose_ok and ext_ok and target_rx_ok:
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
                    f"state={state_ok} pose={pose_ok} ext={ext_ok} "
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
        # run()이 초기화를 담당하며, phase가 활성화된 뒤에만 제어를 수행한다.
        if self.phase not in ("APPROACH", "FINAL_ALIGN", "READY_WAIT_GCS"):
            return

        now = time.monotonic()
        dt = now - self.last_control_monotonic
        self.last_control_monotonic = now
        dt = clamp(dt, 0.01, 0.20)

        if self.state is None or self.local_pose is None:
            return

        # GCS/수동 모드 변경은 제어권 인계로 간주하며 새 모드와 경쟁하지 않는다.
        if self.state.mode != "OFFBOARD":
            self.publish_ready(False)
            return

        altitude = self.current_altitude_m()
        yaw = self.current_yaw_rad()

        if altitude is None or yaw is None:
            return

        target_ok = self.target_is_usable()

        # ---------------------------------------------------------------------
        # 표적 소실/데이터 stale 시 HOLD. 표적 없이 계속 하강하지 않는다.
        # ---------------------------------------------------------------------
        if not target_ok:
            self.reset_horizontal_controllers()
            self.reset_yaw_controller()
            self.last_cmd_vx_east = 0.0
            self.last_cmd_vy_north = 0.0
            self.ready_since = None
            self.publish_ready(False)

            # 0 속도 명령으로 PX4 속도제어기에 현재 위치 유지를 요청한다.
            self.publish_zero_setpoint()

            self.log_control(
                altitude=altitude,
                phase=self.phase,
                target_ok=False,
                position_ok=False,
                heading_ok=False,
                ready_now=False,
                desired_px=(0.0, 0.0),
                v_cmd=(0.0, 0.0, 0.0),
                yaw_rate=0.0,
                descent_scale=0.0,
            )
            return

        # ---------------------------------------------------------------------
        # 3 m 부근에서 APPROACH -> FINAL_ALIGN로 전환한다.
        # ---------------------------------------------------------------------
        if (
            self.phase == "APPROACH"
            and abs(altitude - FINAL_ALIGN_ALT_M)
            <= FINAL_ALIGN_ENTER_TOL_M
        ):
            self.phase = "FINAL_ALIGN"
            self.reset_horizontal_controllers()
            self.reset_yaw_controller()
            self.ready_since = None

            self.get_logger().warn(
                "[PHASE 2] ================================================"
            )
            self.get_logger().warn(
                "[PHASE 2] 3 m reached -> FINAL_ALIGN"
            )
            self.get_logger().warn(
                "[PHASE 2] Camera-offset Landing Zone control ACTIVE"
            )
            self.get_logger().warn(
                "[PHASE 2] LAND command will NOT be sent by this node"
            )
            self.get_logger().warn(
                "[PHASE 2] ================================================"
            )

        final_align = self.phase in ("FINAL_ALIGN", "READY_WAIT_GCS")

        # ---------------------------------------------------------------------
        # 수평 PID 제어.
        # ---------------------------------------------------------------------
        if final_align:
            forward_cmd, right_cmd = self.final_xy_command(dt)
        else:
            forward_cmd, right_cmd = self.approach_xy_command(dt)

        vx_east, vy_north = self.body_velocity_to_local_enu(
            forward_cmd,
            right_cmd,
            yaw,
        )

        # PID 출력이 급격히 변해도 실제 PX4에 전달되는 속도 명령은
        # 부드럽게 변하도록 제한한다.
        vx_east, vy_north = self.apply_xy_slew_rate(
            vx_east,
            vy_north,
            dt,
            final_align,
        )

        # ---------------------------------------------------------------------
        # 수직 접근 및 3 m 고도 유지.
        # ---------------------------------------------------------------------
        vz_up = self.altitude_velocity_command(altitude)

        # -----------------------------------------------------------------
        # 3 m 이전: FOV 오차에 따라 하강속도를 연속적으로 조절한다.
        #
        # 기존의 ON/OFF 방식(vz=0)은 제거한다.
        # XY 보정과 하강을 동시에 유지해서 대각선 형태의 연속적인 접근을 만든다.
        # -----------------------------------------------------------------
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
                    /
                    (FOV_HARD_LIMIT_NORM - DESCENT_FULL_SPEED_FOV_NORM)
                )

                descent_scale = (
                    1.0
                    - ratio * (1.0 - DESCENT_MIN_SCALE)
                )

            descent_scale = clamp(
                descent_scale,
                DESCENT_MIN_SCALE,
                1.0,
            )

            vz_up *= descent_scale

        # ---------------------------------------------------------------------
        # Yaw PI 제어.
        # ---------------------------------------------------------------------
        yaw_rate = self.yaw_rate_command(
            final_align=final_align,
            dt=dt,
        )

        # ---------------------------------------------------------------------
        # 3 m에서 READY 조건 판정.
        # ---------------------------------------------------------------------
        position_ok = False
        heading_ok = False
        ready_now = False

        desired_px = self.desired_final_target_px()

        if final_align:
            position_ok = self.inside_landing_zone()

            if self.target.orientation_valid:
                heading_ok = (
                    abs(math.degrees(self.target.heading_error_rad))
                    <= READY_HEADING_TOL_DEG
                )
            else:
                heading_ok = not REQUIRE_VALID_HEADING_FOR_READY

            altitude_ok = (
                abs(altitude - FINAL_ALIGN_ALT_M)
                <= FINAL_ALIGN_ENTER_TOL_M
            )

            conditions_ok = (
                position_ok
                and heading_ok
                and altitude_ok
            )

            if conditions_ok:
                if self.ready_since is None:
                    self.ready_since = now

                stable_time = now - self.ready_since

                if stable_time >= READY_STABLE_TIME_SEC:
                    ready_now = True
                    self.ready_latched = True

                    if self.phase != "READY_WAIT_GCS":
                        self.phase = "READY_WAIT_GCS"
                        self.get_logger().warn(
                            "[PHASE 2] ================================================"
                        )
                        self.get_logger().warn(
                            "[PHASE 2] READY FOR LAND"
                        )
                        self.get_logger().warn(
                            "[PHASE 2] Position / heading / 3 m condition satisfied"
                        )
                        self.get_logger().warn(
                            "[PHASE 2] Holding alignment. Waiting for GCS mode change."
                        )
                        self.get_logger().warn(
                            "[PHASE 2] THIS NODE DOES NOT COMMAND LAND."
                        )
                        self.get_logger().warn(
                            "[PHASE 2] ================================================"
                        )
            else:
                self.ready_since = None

            # READY가 한번 성립한 후에는 작은 영상 노이즈로 즉시 False가
            # 되지 않도록 더 넓은 '해제 기준'을 사용한다.
            if self.ready_latched:
                release_position_ok = self.inside_ready_release_zone()

                if self.target.orientation_valid:
                    release_heading_ok = (
                        abs(math.degrees(self.target.heading_error_rad))
                        <= READY_RELEASE_HEADING_TOL_DEG
                    )
                else:
                    release_heading_ok = not REQUIRE_VALID_HEADING_FOR_READY

                ready_now = bool(
                    release_position_ok
                    and release_heading_ok
                    and altitude_ok
                )

        self.publish_ready(ready_now)

        # READY 이후에도 폐루프 정렬 제어를 계속 유지한다.
        self.publish_velocity_setpoint(
            vx_east,
            vy_north,
            vz_up,
            yaw_rate,
        )

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

        heading_deg = (
            math.degrees(self.target.heading_error_rad)
            if self.target.orientation_valid
            else float("nan")
        )

        self.get_logger().info(
            "[PHASE 2] "
            f"phase={phase} | "
            f"alt={altitude:.2f}m | "
            f"target={target_ok} | "
            f"norm=({self.target.error_x_norm:+.3f},"
            f"{self.target.error_y_norm:+.3f}) | "
            f"px=({self.target.error_x_px:+.1f},"
            f"{self.target.error_y_px:+.1f}) | "
            f"desired_px=({desired_px[0]:+.1f},"
            f"{desired_px[1]:+.1f}) | "
            f"heading={heading_deg:+.1f}deg | "
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
            "[PHASE 2] Vision approach -> 3 m -> final align -> GCS handoff"
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

        # 실제 제어는 timer callback이 담당하고 run()은 spin을 유지한다.
        # GCS에서 LAND 또는 다른 모드로 변경하면 본 노드의 OFFBOARD 제어권이 종료된다.
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.state is None:
                continue

            if self.state.mode != "OFFBOARD":
                if self.ready_latched:
                    self.get_logger().warn(
                        f"[PHASE 2] GCS/FCU mode handoff detected: "
                        f"{self.state.mode}. Phase 2 control released."
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
        # 여기서는 LAND 명령을 보내지 않는다.
        # GCS에서 모드를 변경한 이후 단계는 PX4/GCS가 담당한다.
        node.publish_ready(False)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
