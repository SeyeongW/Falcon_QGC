import math

from mavros_msgs.msg import Waypoint


MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_VTOL_TAKEOFF = 84
MAV_CMD_NAV_VTOL_LAND = 85
MAV_CMD_DO_VTOL_TRANSITION = 3000

MAV_VTOL_STATE_MC = 3
MAV_VTOL_STATE_FW = 4

MAV_FRAME_GLOBAL_REL_ALT = 3
MAV_FRAME_MISSION = 2


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    earth_radius_m = 6378137.0

    d_lat = north_m / earth_radius_m
    d_lon = east_m / (earth_radius_m * math.cos(math.radians(lat_deg)))

    new_lat = lat_deg + math.degrees(d_lat)
    new_lon = lon_deg + math.degrees(d_lon)

    return new_lat, new_lon


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


def build_pentagon_offsets(center_north_m, center_east_m, side_length_m=200.0):
    """
    정오각형 waypoint offset 생성.

    side_length_m:
        오각형 한 변 길이.
        여기서는 약 200m 간격의 5각형 경로를 만든다.

    반환:
        [(north_m, east_m), ...] 5개
    """

    # 정오각형의 외접반지름 R = s / (2 sin(pi / 5))
    radius_m = side_length_m / (2.0 * math.sin(math.pi / 5.0))

    points = []

    # 첫 점을 북쪽 위쪽에 두고 시계방향으로 회전
    # 좌표계: north/east offset
    start_angle_rad = math.radians(90.0)

    for i in range(5):
        angle = start_angle_rad - i * 2.0 * math.pi / 5.0

        north = center_north_m + radius_m * math.sin(angle)
        east = center_east_m + radius_m * math.cos(angle)

        points.append((north, east))

    return points


def build_phase1_vtol_transit_mission(
    home_lat,
    home_lon,
    mission_alt_m=30.0,
    takeoff_north_m=30.0,
    pentagon_center_north_m=350.0,
    pentagon_center_east_m=0.0,
    pentagon_side_m=200.0,
):
    """
    Phase 1 AUTO.MISSION 생성.

    Mission:
    0. VTOL_TAKEOFF
    1. DO_VTOL_TRANSITION to Fixed-wing
    2. Pentagon waypoint 1
    3. Pentagon waypoint 2
    4. Pentagon waypoint 3
    5. Pentagon waypoint 4
    6. Pentagon waypoint 5
    7. DO_VTOL_TRANSITION to Multicopter
    8. VTOL_LAND

    실제 Phase 2로 넘길 때는 VTOL_LAND까지 가지 않는다.
    phase1.py에서 MC 역천이를 확인한 뒤 AUTO.LOITER로 전환한다.
    """

    takeoff_lat, takeoff_lon = offset_latlon(
        home_lat,
        home_lon,
        north_m=takeoff_north_m,
        east_m=0.0,
    )

    pentagon_offsets = build_pentagon_offsets(
        center_north_m=pentagon_center_north_m,
        center_east_m=pentagon_center_east_m,
        side_length_m=pentagon_side_m,
    )

    pentagon_latlon = [
        offset_latlon(
            home_lat,
            home_lon,
            north_m=north_m,
            east_m=east_m,
        )
        for north_m, east_m in pentagon_offsets
    ]

    last_lat, last_lon = pentagon_latlon[-1]

    waypoints = []

    # 0. VTOL 이륙
    waypoints.append(
        make_wp(
            command=MAV_CMD_NAV_VTOL_TAKEOFF,
            lat=takeoff_lat,
            lon=takeoff_lon,
            alt=mission_alt_m,
            frame=MAV_FRAME_GLOBAL_REL_ALT,
            is_current=True,
        )
    )

    # 1. 고정익 천이
    # DO_VTOL_TRANSITION is a MISSION-frame DO command: x/y/z (param5/6/7) must
    # be 0 (PX4 rejects "param5 invalid" if a lat/lon is put here).
    waypoints.append(
        make_wp(
            command=MAV_CMD_DO_VTOL_TRANSITION,
            lat=0.0,
            lon=0.0,
            alt=0.0,
            frame=MAV_FRAME_MISSION,
            param1=MAV_VTOL_STATE_FW,
        )
    )

    # 2~6. 고정익 5각형 waypoint 5개
    for lat, lon in pentagon_latlon:
        waypoints.append(
            make_wp(
                command=MAV_CMD_NAV_WAYPOINT,
                lat=lat,
                lon=lon,
                alt=mission_alt_m,
                frame=MAV_FRAME_GLOBAL_REL_ALT,
                param2=35.0,  # acceptance radius
            )
        )

    # 7. 마지막 오각형 waypoint 근처에서 멀티콥터 역천이
    # MISSION-frame DO command -> x/y/z must be 0 (see note above).
    waypoints.append(
        make_wp(
            command=MAV_CMD_DO_VTOL_TRANSITION,
            lat=0.0,
            lon=0.0,
            alt=0.0,
            frame=MAV_FRAME_MISSION,
            param1=MAV_VTOL_STATE_MC,
        )
    )

    # 8. mission validity용 VTOL_LAND
    # 실제로는 phase1.py가 역천이 확인 후 AUTO.LOITER로 전환하므로 여기까지 가지 않는다.
    waypoints.append(
        make_wp(
            command=MAV_CMD_NAV_VTOL_LAND,
            lat=last_lat,
            lon=last_lon,
            alt=0.0,
            frame=MAV_FRAME_GLOBAL_REL_ALT,
        )
    )

    mission_info = {
        "start": (home_lat, home_lon),
        "takeoff": (takeoff_lat, takeoff_lon),
        "fw_wp": pentagon_latlon[0],
        "fw_wps": pentagon_latlon,
        "mc_wp": (last_lat, last_lon),
        "alt_m": mission_alt_m,
        "pentagon_side_m": pentagon_side_m,
        "count": len(waypoints),
    }

    return waypoints, mission_info
