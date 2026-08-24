import math

from mavros_msgs.msg import Waypoint


MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_VTOL_LAND = 85
MAV_CMD_DO_VTOL_TRANSITION = 3000

MAV_VTOL_STATE_MC = 3
MAV_VTOL_STATE_FW = 4

MAV_FRAME_GLOBAL_REL_ALT = 3
MAV_FRAME_MISSION = 2


# ============================================================
# 대회 GPS 좌표 직접 입력부
# ============================================================
# 형식:
#   "wp1": (lat, lon, alt)
#
# alt는 상대고도[m]로 사용.
# 실제 대회장에서는 반드시 실제 GPS 좌표로 교체.
# ============================================================

MISSION_GPS = {
    # QGC 2번 → WP1
    "wp1": (47.3983727, 8.5461613, 30),

    # QGC 3번 → WP2
    "wp2": (47.3993855, 8.5470882, 30),

    # QGC 4번 → WP3
    "wp3": (47.4004467, 8.5493682, 30),

    # QGC 5번 → WP4
    "wp4": (47.4004124, 8.5438027, 30),

    # QGC 6번 → WP5
    "wp5": (47.3992492, 8.5461776, 30),

    # QGC 7번 → REP
    "rep": (47.3983302, 8.5482113, 10),
}

def make_wp(
    command,
    lat,
    lon,
    alt,
    frame=MAV_FRAME_GLOBAL_REL_ALT,
    is_current=False,
    autocontinue=True,
    param1=0.0,
    param2=0.0,
    param3=0.0,
    param4=0.0,
):
    wp = Waypoint()

    wp.frame = int(frame)
    wp.command = int(command)
    wp.is_current = bool(is_current)
    wp.autocontinue = bool(autocontinue)

    wp.param1 = float(param1)
    wp.param2 = float(param2)
    wp.param3 = float(param3)
    wp.param4 = float(param4)

    wp.x_lat = float(lat)
    wp.y_long = float(lon)
    wp.z_alt = float(alt)

    return wp


def validate_gps_point(name, point):
    if point is None:
        raise ValueError(f"{name} is None")

    if len(point) != 3:
        raise ValueError(f"{name} must be (lat, lon, alt)")

    lat, lon, alt = point

    if not (-90.0 <= float(lat) <= 90.0):
        raise ValueError(f"{name} latitude out of range: {lat}")

    if not (-180.0 <= float(lon) <= 180.0):
        raise ValueError(f"{name} longitude out of range: {lon}")

    if float(alt) < 0.0:
        raise ValueError(f"{name} altitude must be >= 0: {alt}")

    return float(lat), float(lon), float(alt)


def get_mission_gps_points(mission_gps=None, override_alt_m=None):
    if mission_gps is None:
        mission_gps = MISSION_GPS

    required_keys = ["wp1", "wp2", "wp3", "wp4", "wp5", "rep"]

    points = {}

    for key in required_keys:
        if key not in mission_gps:
            raise KeyError(f"Missing mission GPS key: {key}")

        lat, lon, alt = validate_gps_point(key, mission_gps[key])

        if override_alt_m is not None:
            alt = float(override_alt_m)

        points[key] = (lat, lon, alt)

    return points


def build_phase1_wp2_to_rep_mission(
    home_lat=None,
    home_lon=None,
    mission_alt_m=30.0,
    mission_gps=None,
):
    """
    Mission B:
        seq0 : WP2
        seq1 : WP3
        seq2 : WP4
        seq3 : WP5
        seq4 : DO_VTOL_TRANSITION to Multicopter
        seq5 : REP
        seq6 : VTOL_LAND dummy

    주의:
        FW 전환은 이 mission 안에서 하지 않는다.
        phase1.py에서 Enter 후 CommandLong으로 직접 FW 전환한다.
    """

    points = get_mission_gps_points(
        mission_gps=mission_gps,
        override_alt_m=mission_alt_m,
    )

    wp2_lat, wp2_lon, wp2_alt = points["wp2"]
    wp3_lat, wp3_lon, wp3_alt = points["wp3"]
    wp4_lat, wp4_lon, wp4_alt = points["wp4"]
    wp5_lat, wp5_lon, wp5_alt = points["wp5"]
    rep_lat, rep_lon, rep_alt = points["rep"]

    waypoints = []

    fw_acceptance_radius_m = 35.0

    # seq0~seq3. WP2 → WP3 → WP4 → WP5 고정익 비행
    for idx, (lat, lon, alt) in enumerate(
        [
            (wp2_lat, wp2_lon, wp2_alt),
            (wp3_lat, wp3_lon, wp3_alt),
            (wp4_lat, wp4_lon, wp4_alt),
            (wp5_lat, wp5_lon, wp5_alt),
        ]
    ):
        waypoints.append(
            make_wp(
                command=MAV_CMD_NAV_WAYPOINT,
                lat=lat,
                lon=lon,
                alt=alt,
                frame=MAV_FRAME_GLOBAL_REL_ALT,
                is_current=(idx == 0),
                autocontinue=True,
                param2=fw_acceptance_radius_m,
            )
        )

    # seq4. WP5 이후 고정익 → 회전익 천이
    waypoints.append(
        make_wp(
            command=MAV_CMD_DO_VTOL_TRANSITION,
            lat=wp5_lat,
            lon=wp5_lon,
            alt=wp5_alt,
            frame=MAV_FRAME_MISSION,
            autocontinue=True,
            param1=MAV_VTOL_STATE_MC,
        )
    )

    # seq5. REP 진입 및 호버링
    waypoints.append(
        make_wp(
            command=MAV_CMD_NAV_WAYPOINT,
            lat=rep_lat,
            lon=rep_lon,
            alt=rep_alt,
            frame=MAV_FRAME_GLOBAL_REL_ALT,
            autocontinue=True,
            param1=5.0,
            param2=8.0,
        )
    )

    # seq6. Mission validity용 dummy LAND
    waypoints.append(
        make_wp(
            command=MAV_CMD_NAV_VTOL_LAND,
            lat=rep_lat,
            lon=rep_lon,
            alt=0.0,
            frame=MAV_FRAME_GLOBAL_REL_ALT,
            autocontinue=True,
        )
    )

    mission_info = {
        "name": "phase1_wp2_to_rep",
        "start": (home_lat, home_lon) if home_lat is not None and home_lon is not None else None,

        "wp2": (wp2_lat, wp2_lon),
        "wp3": (wp3_lat, wp3_lon),
        "wp4": (wp4_lat, wp4_lon),
        "wp5": (wp5_lat, wp5_lon),
        "rep": (rep_lat, rep_lon),

        "wp2_alt_m": wp2_alt,
        "wp3_alt_m": wp3_alt,
        "wp4_alt_m": wp4_alt,
        "wp5_alt_m": wp5_alt,
        "rep_alt_m": rep_alt,

        "fw_wp": (wp2_lat, wp2_lon),
        "fw_wps": [
            (wp2_lat, wp2_lon),
            (wp3_lat, wp3_lon),
            (wp4_lat, wp4_lon),
            (wp5_lat, wp5_lon),
        ],
        "mc_wp": (rep_lat, rep_lon),

        "alt_m": mission_alt_m,
        "count": len(waypoints),

        # Mission B 기준 seq index
        "seq_wp2": 0,
        "seq_wp3": 1,
        "seq_wp4": 2,
        "seq_wp5": 3,
        "seq_transition_mc": 4,
        "seq_rep": 5,
        "seq_land_dummy": 6,
    }

    return waypoints, mission_info
