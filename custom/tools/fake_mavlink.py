#!/usr/bin/env python3
"""Fake PX4 VTOL telemetry → QGroundControl over UDP.

Lets you test the custom VTOL-GCS (map, telemetry, control-surface panel)
without a real Pixhawk/PX4. It flies a small waypoint mission in a loop and
emits the MAVLink messages QGC needs:

  HEARTBEAT (PX4 VTOL)      -> vehicle shows up as an armed VTOL
  GLOBAL_POSITION_INT / GPS -> vehicle position on the real map
  ATTITUDE                  -> roll/pitch/yaw (attitude indicator)
  EXTENDED_SYS_STATE        -> VTOL hover/forward mode (drives the panel)
  SERVO_OUTPUT_RAW          -> control-surface + motor signals (drives the panel)
  SYS_STATUS / VFR_HUD      -> battery / speed / altitude

Servo channel convention (match this in the QML panel binding):
  ch1 aileron   ch2 elevator   ch3 pusher   ch4 rudder   ch5-8 lift motors
  surfaces: 1000..2000us, 1500 = neutral.   motors: 1000 off .. 2000 full.

Usage:
  python3 fake_mavlink.py                       # -> 127.0.0.1:14550 (QGC default)
  python3 fake_mavlink.py --target 127.0.0.1:14550 --home 47.3977 8.5456
"""
import argparse
import math
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mav

EARTH = 111320.0  # meters per degree latitude (approx)


def parse_args():
    p = argparse.ArgumentParser(description="Fake PX4 VTOL telemetry to QGC")
    p.add_argument("--target", default="127.0.0.1:14550",
                   help="host:port QGC listens on (default 127.0.0.1:14550)")
    p.add_argument("--home", nargs=2, type=float, default=[47.397742, 8.545594],
                   metavar=("LAT", "LON"), help="home position (default PX4 SITL Zurich)")
    p.add_argument("--speed", type=float, default=0.012,
                   help="mission progress per second (default 0.012 ~ 80s loop)")
    return p.parse_args()


# Waypoints as (north, east) meter offsets from home — a zigzag mission.
WAYPOINTS_M = [(0, 0), (120, 80), (40, 200), (180, 300), (90, 420)]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm180(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def surface_pwm(value):
    """-1..1 -> 1000..2000us, 1500 neutral."""
    return int(1500 + clamp(value, -1, 1) * 500)


def motor_pwm(throttle):
    """0..1 -> 1000..2000us."""
    return int(1000 + clamp(throttle, 0, 1) * 1000)


# Minimal PX4-ish parameter set. QGC downloads the full list on connect; without
# a response it warns "vehicle did not respond to request for parameters" and
# limits the UI. Serving a self-consistent set (advertised count == count sent)
# clears that. Values are plausible but not a real airframe. [name, value, type]
INT = mav.MAV_PARAM_TYPE_INT32
REAL = mav.MAV_PARAM_TYPE_REAL32
PARAMS = [
    ["SYS_AUTOSTART", 4001, INT],
    ["SYS_MC_EST_GROUP", 2, INT],
    ["MAV_TYPE", 22, INT],
    ["MAV_SYS_ID", 1, INT],
    ["MAV_COMP_ID", 1, INT],
    ["BAT1_N_CELLS", 4, INT],
    ["COM_RC_IN_MODE", 1, INT],
    ["NAV_RCL_ACT", 2, INT],
    ["NAV_DLL_ACT", 0, INT],
    ["COM_FLTMODE1", 0, INT],
    ["CBRK_SUPPLY_CHK", 0, INT],
    ["EKF2_AID_MASK", 1, INT],
    ["MPC_XY_VEL_MAX", 12.0, REAL],
    ["MIS_TAKEOFF_ALT", 60.0, REAL],
]
_PARAM_INDEX = {name: i for i, (name, _, _) in enumerate(PARAMS)}


def _param_name(raw):
    """PARAM_* param_id may arrive as str or NUL-padded bytes."""
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", "ignore")
    return raw.rstrip("\x00")


def send_param(conn, i):
    name, value, ptype = PARAMS[i]
    conn.mav.param_value_send(name.encode("ascii"), float(value), ptype, len(PARAMS), i)


def handle_incoming(conn):
    """Answer parameter requests so QGC can complete its param download."""
    msg = conn.recv_match(blocking=False)
    while msg is not None:
        mtype = msg.get_type()
        if mtype == "PARAM_REQUEST_LIST":
            for i in range(len(PARAMS)):
                send_param(conn, i)
        elif mtype == "PARAM_REQUEST_READ":
            idx = msg.param_index
            if 0 <= idx < len(PARAMS):
                send_param(conn, idx)
            else:
                i = _PARAM_INDEX.get(_param_name(msg.param_id))
                if i is not None:
                    send_param(conn, i)
        elif mtype == "PARAM_SET":
            i = _PARAM_INDEX.get(_param_name(msg.param_id))
            if i is not None:
                PARAMS[i][1] = msg.param_value
                send_param(conn, i)
        msg = conn.recv_match(blocking=False)


def main():
    args = parse_args()
    host, port = args.target.split(":")
    home_lat, home_lon = args.home

    conn = mavutil.mavlink_connection(f"udpout:{host}:{port}", source_system=1, source_component=1)
    print(f"Sending fake PX4 VTOL telemetry to {host}:{port} (Ctrl+C to stop)")

    # Precompute segment lengths for constant-speed travel.
    segs, total = [], 0.0
    for i in range(len(WAYPOINTS_M) - 1):
        dn = WAYPOINTS_M[i + 1][0] - WAYPOINTS_M[i][0]
        de = WAYPOINTS_M[i + 1][1] - WAYPOINTS_M[i][1]
        d = math.hypot(dn, de)
        segs.append(d)
        total += d

    boot = time.time()
    prog = 0.0
    prev_heading = 0.0
    bank = 0.0
    last = {"hb": 0.0, "slow": 0.0, "mid": 0.0}

    PX4_AUTO_MISSION = (4 << 16) | (4 << 24)  # PX4 custom_mode: AUTO / MISSION

    while True:
        now = time.time()
        dt = 0.02
        t_ms = int((now - boot) * 1000)

        # Respond to parameter (and other) requests from QGC.
        handle_incoming(conn)

        # --- advance along the mission ---
        prog = (prog + args.speed * dt) % 1.0
        d = prog * total
        i = 0
        while i < len(segs) - 1 and d > segs[i]:
            d -= segs[i]
            i += 1
        local = d / segs[i] if segs[i] > 0 else 0.0
        an, ae = WAYPOINTS_M[i]
        bn, be = WAYPOINTS_M[i + 1]
        north = an + (bn - an) * local
        east = ae + (be - ae) * local

        heading = math.degrees(math.atan2(be - ae, bn - an))  # 0 = north
        last_seg = len(segs) - 1
        fwd = not (i == 0 or i == last_seg)  # hover on first/last leg

        # bank into the upcoming turn
        target_bank = 0.0
        if i < last_seg and local > 0.55:
            cn, ce = WAYPOINTS_M[i + 2]
            next_h = math.degrees(math.atan2(ce - be, cn - bn))
            target_bank = clamp(norm180(next_h - heading) / 45.0, -1, 1)
        bank += (target_bank - bank) * 0.06
        prev_heading = heading

        # --- geo position ---
        lat = home_lat + north / EARTH
        lon = home_lon + east / (EARTH * math.cos(math.radians(home_lat)))
        alt = 60.0 if fwd else (60.0 * local if i == 0 else 60.0 * (1 - local))
        gspeed = 17.0 if fwd else 2.5

        # --- attitude ---
        roll = math.radians(bank * 35.0)
        pitch = math.radians(2.0 if fwd else 0.0)
        yaw = math.radians(heading)

        # --- control surfaces / motors (the convention above) ---
        aileron = surface_pwm(bank)
        elevator = surface_pwm(0.12 * math.sin(now * 3) if fwd else 0.0)
        rudder = surface_pwm(bank * 0.7)
        pusher = motor_pwm(0.6 if fwd else 0.0)
        lift = motor_pwm(0.0 if fwd else 0.58)
        servos = [aileron, elevator, pusher, rudder, lift, lift, lift, lift]

        # ===== send =====
        # 1 Hz: heartbeat, sys_status, extended_sys_state
        if now - last["hb"] >= 1.0:
            last["hb"] = now
            conn.mav.heartbeat_send(
                mav.MAV_TYPE_VTOL_RESERVED4, mav.MAV_AUTOPILOT_PX4,
                mav.MAV_MODE_FLAG_SAFETY_ARMED | mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                PX4_AUTO_MISSION, mav.MAV_STATE_ACTIVE)
            conn.mav.sys_status_send(
                0, 0, 0, 500, 16800, 5000, int(72 - prog * 10),
                0, 0, 0, 0, 0, 0)
            conn.mav.extended_sys_state_send(
                mav.MAV_VTOL_STATE_FW if fwd else mav.MAV_VTOL_STATE_MC,
                mav.MAV_LANDED_STATE_IN_AIR)

        # 5 Hz: global position, gps raw, vfr_hud
        if now - last["mid"] >= 0.2:
            last["mid"] = now
            conn.mav.global_position_int_send(
                t_ms, int(lat * 1e7), int(lon * 1e7), int(alt * 1000),
                int(alt * 1000), 0, 0, 0, int(heading * 100))
            conn.mav.gps_raw_int_send(
                t_ms * 1000, 3, int(lat * 1e7), int(lon * 1e7),
                int(alt * 1000), 80, 80, int(gspeed * 100),
                int(heading * 100), 12)
            conn.mav.vfr_hud_send(
                gspeed, gspeed, int(heading), int(60 if fwd else 30),
                alt, 1.0 if fwd else 0.0)

        # 20 Hz: attitude, servo outputs
        conn.mav.attitude_send(t_ms, roll, pitch, yaw, 0, 0, 0)
        conn.mav.servo_output_raw_send(t_ms * 1000, 0, *servos)

        time.sleep(dt)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
