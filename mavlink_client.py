#!/usr/bin/env python3
"""
MAVLink client for tracker-so.py — routes through MAVProxy.

Start MAVProxy on the RPi before running tracker-so.py:
    mavproxy.py --master=/dev/ttyACM0 --baud=115200 \
                --out=udpout:127.0.0.1:14551 \
                --out=udp:<GCS_IP>:14550

Usage:
    import mavlink_client
    mavlink_client.connect()                 # call once at startup
    mavlink_client.send_vision_error(p, y, is_tracking)  # non-blocking, called every frame
"""
import math
import time
import struct
import threading
import subprocess
import atexit

_connection    = None
_enabled       = False
_ser           = None
_launched      = False
_mavproxy_proc = None
DEBUG           = True  # set True to print pitch/yaw values every frame
SHOW_TELEMETRY  = True  # set True to print incoming ATTITUDE in the console

# "ardupilot" or "px4" — set via set_autopilot() before connect(). Selects
# MAV_CMD_DO_SET_MODE encoding, whether OFFBOARD/GUIDED needs a primed
# setpoint stream first, and how SET_ATTITUDE_TARGET's yaw is composed.
_autopilot = "ardupilot"

# PX4 custom_mode packs main_mode into bits 16-23 (sub_mode in 24-31).
# Values from PX4's mavlink/mavlink_main.h custom mode enum.
_PX4_MAIN_MODE_MANUAL   = 1
_PX4_MAIN_MODE_OFFBOARD = 6


def set_autopilot(kind: str):
    """Select "ardupilot" or "px4" mode-switch/attitude-target behavior.
    Call before connect()."""
    global _autopilot
    kind = kind.lower()
    if kind not in ("ardupilot", "px4"):
        raise ValueError(f"unknown autopilot kind: {kind!r} (expected 'ardupilot' or 'px4')")
    _autopilot = kind

# send_attitude_target() clamp: bounds how far the camera error is ever
# allowed to rotate the target away from the FC's current attitude.
MAX_ANGLE = math.radians(25)

# ArduPlane's GUIDED SET_ATTITUDE_TARGET takes an absolute, horizon-referenced
# roll/pitch/yaw demand (confirmed on the bench), so "hold current attitude"
# requires composing the camera error onto the FC's latest reported attitude
# before sending. _attitude_timestamp lets send_attitude_target() refuse to
# compose onto a stale reading rather than send a wrong absolute target.
_attitude_lock      = threading.Lock()
_current_attitude    = None  # (roll, pitch, yaw) radians, or None until first ATTITUDE msg
_attitude_timestamp  = 0.0
MAX_ATTITUDE_AGE     = 0.3   # seconds; skip send if the cached attitude is older than this


# ---------------------------------------------------------------------------
# Launch state
# ---------------------------------------------------------------------------

_launch_lock = threading.Lock()
_last_launch_change = 0.0

def set_launch(value: bool):
    global _launched, _last_launch_change
    with _launch_lock:           # Flask is threaded — lock prevents two threads
        if _launched == value:   # racing past the debounce simultaneously
            return
        now = time.time()
        if now - _last_launch_change < 0.3:
            print(f"[Launch] debounced rapid toggle to {value}")
            return
        _last_launch_change = now
        _launched = value
    print(f"[Launch] {'LAUNCHED' if value else 'RESET'}")


# ---------------------------------------------------------------------------
# Send (synchronous — called directly from main loop, no queue)
# ---------------------------------------------------------------------------

def send_vision_error(pitch_err, yaw_err, is_tracking=False):
    """Send MAVLink debug messages synchronously from the main loop.
    Both is_tracking and _launched are read at the same instant, eliminating
    the race condition that existed when a sender thread read _launched later.
    """
    if is_tracking:
        x, y, z = float(pitch_err), float(yaw_err), 1.0
    else:
        x, y, z = 0.0, 0.0, 0.0

    launch_val = 1.0 if _launched else -1.0

    if _enabled:
        try:
            _connection.mav.debug_vect_send(
                b"vision_err",
                int(time.time() * 1e6),
                x, y, z
            )
            _connection.mav.named_value_float_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                b"launch",
                launch_val
            )
            if DEBUG:
                print(f"[MAVLink] x={x:.4f}, y={y:.4f}, z={z:.4f}, launch={launch_val:.0f}")
        except Exception as e:
            print(f"[MAVLink] send failed: {e}")
    if _ser is not None:
        packet = struct.pack('<BBff', 0xAA, 0x55, x, y)
        try:
            _ser.write(packet)
            if DEBUG:
                print(f"[Serial] x={x:.4f}, y={y:.4f}")
        except Exception as e:
            print(f"[Serial] Send failed: {e}")


# ---------------------------------------------------------------------------
# Connect
# ---------------------------------------------------------------------------

def _find_fc_port(configured):
    import os, glob
    if os.path.exists(configured):
        return configured
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ports = sorted(glob.glob(pattern))
        if ports:
            print(f"[MAVProxy] {configured} not found, auto-detected {ports[0]}")
            return ports[0]
    print(f"[MAVProxy] WARNING: no FC port found, falling back to {configured}")
    return configured


def start_mavproxy(pixhawk_port="/dev/ttyACM0", pixhawk_baud=115200,
                   gcs_port=14550, local_port=14551,
                   extra_outputs=None):
    """
    Launch MAVProxy as a background subprocess.
    Automatically killed when the Python process exits.

    extra_outputs: list of IP strings that each get a dedicated unicast
                   --out=udpout:<ip>:<gcs_port> added to the MAVProxy command.
    """
    global _mavproxy_proc
    pixhawk_port = _find_fc_port(pixhawk_port)
    cmd = [
        "/home/mahat/mav_venv/bin/mavproxy.py",
        f"--master={pixhawk_port}",
        f"--baud={pixhawk_baud}",
        f"--out=udpout:127.0.0.1:{local_port}",
    ]
    for ip in (extra_outputs or []):
        cmd.append(f"--out=udpout:{ip}:{gcs_port}")
        print(f"[MAVProxy] Extra unicast output → {ip}:{gcs_port}")
    cmd.append("--daemon")
    print(f"[MAVProxy] Starting: {' '.join(cmd)}")
    _mavproxy_proc = subprocess.Popen(cmd)
    atexit.register(_stop_mavproxy)
    time.sleep(2)  # give MAVProxy time to connect to Pixhawk


def _stop_mavproxy():
    if _mavproxy_proc and _mavproxy_proc.poll() is None:
        print("[MAVProxy] Stopping...")
        _mavproxy_proc.terminate()
        try:
            _mavproxy_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            print("[MAVProxy] Force-killing...")
            _mavproxy_proc.kill()


def _request_attitude_stream(rate_hz=20):
    """Ask the FC to push ATTITUDE at rate_hz. send_attitude_target() composes
    onto the latest ATTITUDE reading, so the default (~2-4 Hz) stream rate is
    too stale for a per-frame control loop — this speeds it up explicitly."""
    from pymavlink import mavutil
    _connection.mav.command_long_send(
        _connection.target_system,
        _connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        int(1e6 / rate_hz),  # microseconds between messages
        0, 0, 0, 0, 0
    )


_PX4_RCL_EXCEPT_OFFBOARD = 4  # COM_RCL_EXCEPT bitmask bit: exempt Offboard from RC-loss failsafe

# EKF2_NOAID_TOUT ceiling: PX4 flags position invalid once dead-reckoning (pure
# IMU, no GPS/vision/flow aiding) has run longer than this many microseconds.
# We have no GPS at all and only ever send attitude+thrust setpoints (never
# need position), so push this as high as PX4 will accept to keep position
# "valid" for the whole flight rather than have it expire mid-flight.
_PX4_NOAID_TOUT_US = 2_000_000_000  # ~33 minutes


# PX4 params this rig always wants fixed to a specific value on every connect.
# Blind writes, not read-modify-write: MAVProxy does its own full ~1150-param
# bulk fetch right after connecting, which floods/delays PARAM_VALUE replies
# on this same link enough that waiting on a read reliably timed out on the
# bench (tried up to a 20s budget). None of these need the old value anyway —
# this rig has a fixed role (no RC, no GPS, attitude-only offboard), so
# there's nothing else that would have set competing bits/values worth
# preserving. Confirmation is logged asynchronously via PARAM_VALUE in
# _telemetry_reader instead of blocking connect() on a reply.
_PX4_FIXED_PARAMS = {
    # Bitmask bit 2 = exempt Offboard from the RC-loss failsafe. Without this,
    # PX4 treats "no RC ever received" as RC-lost from boot and silently
    # reverts/blocks OFFBOARD entry — DO_SET_MODE still ACKs ACCEPTED, but
    # HEARTBEAT ground-truth shows the FC stuck in its old mode. Confirmed on
    # the bench (no RC transmitter connected).
    'COM_RCL_EXCEPT': 4,
    # This airframe has no GPS receiver at all, ever, and mavlink_client only
    # sends attitude+thrust setpoints (never position/velocity), so a GPS
    # position estimate is neither available nor needed. Left at the default
    # (requiring GPS), the EKF never validates a position and PX4's generic
    # position-invalid failsafe force-switches to LAND a few seconds after
    # arming regardless of flight mode — confirmed on the bench: OFFBOARD
    # entry silently reverted, HEARTBEAT showed AUTO/LAND ~9s after arm, with
    # the land-detector oscillating Takeoff/Landing continuously afterward.
    'EKF2_GPS_CTRL': 0,
    # Ceiling on how long PX4 trusts pure-IMU dead reckoning (no GPS/vision/
    # flow aiding) before flagging position invalid again. Push it to the
    # max PX4 will accept so that failsafe never re-trips mid-flight.
    'EKF2_NOAID_TOUT': 2_000_000_000,  # microseconds, ~33 minutes
}


def _apply_px4_fixed_params():
    from pymavlink import mavutil
    for name, value in _PX4_FIXED_PARAMS.items():
        packed = struct.unpack('<f', struct.pack('<i', value))[0]
        _connection.mav.param_set_send(
            _connection.target_system, _connection.target_component,
            name.encode(), packed, mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        print(f"[MAVLink] {name} -> {value} (sent, see async confirmation)")


def _lock_onto_autopilot(conn, timeout=5.0):
    """wait_heartbeat() alone does NOT set target_system/target_component —
    that's a common pymavlink footgun. target_system auto-populates only as
    a side effect (pymavlink locks onto the srcSystem of the *first* HEARTBEAT
    that looks vehicle-like), and only if that first HEARTBEAT wasn't, say,
    MAVProxy's own GCS-type heartbeat on the same link. target_component is
    NEVER auto-populated by pymavlink — it silently stays 0 forever unless we
    set it explicitly. Confirmed on the bench: a one-shot wait_heartbeat() at
    connect time left target_system=0, target_component=0, so our PX4 param
    writes were being addressed to component 0 the whole time — PARAM_SET
    apparently doesn't get the same broadcast tolerance PX4 gives COMMAND_LONG
    (which is why arm/mode commands worked despite this bug and param writes
    didn't). Loop until we see a HEARTBEAT specifically from the autopilot
    component (compid 1), then set target_system/target_component from it."""
    from pymavlink import mavutil
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=deadline - time.time())
        if msg and msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            conn.target_system = msg.get_srcSystem()
            conn.target_component = msg.get_srcComponent()
            return True
    return False


def connect(url="udpin:0.0.0.0:14551", fallback_url=None):
    """
    Connect to MAVProxy via UDP and start the telemetry reader thread.
    MAVProxy must be running with --out=udpout:127.0.0.1:14551.
    If the primary connection gets no heartbeat (e.g. no USB), falls back to
    fallback_url (e.g. udpout:GCS_IP:14550) so debug_vect still reaches the GCS.
    """
    global _connection, _enabled
    from pymavlink import mavutil
    try:
        _connection = mavutil.mavlink_connection(url)
        if not _lock_onto_autopilot(_connection, timeout=15.0):
            raise RuntimeError("no HEARTBEAT from the autopilot component (compid 1)")
        if _autopilot == "px4":
            _apply_px4_fixed_params()
        _enabled = True
        _request_attitude_stream()
        set_guided_mode()
        print(f"[MAVLink] Connected via MAVProxy ({url}), heartbeat received.")
    except Exception as e:
        print(f"[WARNING] MAVLink primary connection failed: {e}")
        if fallback_url:
            try:
                _connection = mavutil.mavlink_connection(fallback_url)
                _enabled = True
                _request_attitude_stream()
                print(f"[MAVLink] Fallback connected ({fallback_url}), sending debug_vect to GCS directly.")
            except Exception as e2:
                print(f"[WARNING] MAVLink fallback also failed: {e2}")
                _connection = None
                _enabled    = False
        else:
            _connection = None
            _enabled    = False


def connect_serial(port="/dev/serial0", baud=57600):
    """Open a raw serial port for sending pitch/yaw packets (non-MAVLink)."""
    global _ser
    try:
        import serial
        _ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=1,
            rtscts=False, dsrdtr=False, xonxoff=False,
        )
        print(f"[Serial] Connected to {port} at {baud} baud.")
    except Exception as e:
        print(f"[WARNING] Serial not connected: {e}")
        _ser = None


# ---------------------------------------------------------------------------
# Telemetry reader thread
# ---------------------------------------------------------------------------

_PX4_MAIN_MODE_NAMES = {
    1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO",
    6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE",
}
_last_hb_armed = None
_last_hb_mode  = None


def _telemetry_reader():
    from pymavlink import mavutil
    mav_result_names = mavutil.mavlink.enums['MAV_RESULT']
    while True:
        if not _enabled or _connection is None:
            time.sleep(0.5)
            continue
        try:
            msg = _connection.recv_match(type=['ATTITUDE', 'STATUSTEXT', 'COMMAND_ACK', 'HEARTBEAT', 'PARAM_VALUE'],
                                          blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == 'PARAM_VALUE':
                name = msg.param_id.rstrip('\x00')
                if name in _PX4_FIXED_PARAMS:
                    value = struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
                    print(f"[FC] PARAM_VALUE {name} = {value}")
            elif msg.get_type() == 'STATUSTEXT':
                print(f"[FC] {msg.text.strip()}")
            elif msg.get_type() == 'COMMAND_ACK':
                result = mav_result_names.get(msg.result)
                result_name = result.name if result else msg.result
                cmd = mavutil.mavlink.enums['MAV_CMD'].get(msg.command)
                cmd_name = cmd.name if cmd else msg.command
                print(f"[FC] COMMAND_ACK {cmd_name} -> {result_name}")
            elif msg.get_type() == 'HEARTBEAT':
                # Ground truth for the actual current mode — DO_SET_MODE's
                # COMMAND_ACK only means "command parsed", not "transition
                # accepted"; PX4 can silently reject the state change after
                # ACKing receipt. Only print on change to avoid flooding
                # (HEARTBEAT streams at ~1Hz regardless).
                if _connection is not None and msg.get_srcSystem() == _connection.target_system:
                    global _last_hb_armed, _last_hb_mode
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if _autopilot == "px4":
                        main_mode = (msg.custom_mode >> 16) & 0xFF
                        mode_name = _PX4_MAIN_MODE_NAMES.get(main_mode, f"main_mode={main_mode}")
                    else:
                        mode_name = f"custom_mode={msg.custom_mode}"
                    if (armed, mode_name) != (_last_hb_armed, _last_hb_mode):
                        print(f"[FC] HEARTBEAT armed={armed} mode={mode_name}")
                        _last_hb_armed, _last_hb_mode = armed, mode_name
            elif msg.get_type() == 'ATTITUDE':
                global _current_attitude, _attitude_timestamp
                with _attitude_lock:
                    _current_attitude = (msg.roll, msg.pitch, msg.yaw)
                    _attitude_timestamp = time.time()
                if SHOW_TELEMETRY:
                    print(f"[Telem] roll={math.degrees(msg.roll):+.1f}°  "
                          f"pitch={math.degrees(msg.pitch):+.1f}°  "
                          f"yaw={math.degrees(msg.yaw):+.1f}°")
        except Exception as e:
            print(f"[Telem] read error: {e}")
            time.sleep(0.5)

_telem_thread = threading.Thread(target=_telemetry_reader, daemon=True)
_telem_thread.start()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def _is_armed():
    """Return True if the FC heartbeat shows armed. Filters for FC sysid, not MAVProxy."""
    from pymavlink import mavutil
    deadline = time.time() + 3.0
    while time.time() < deadline:
        try:
            hb = _connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if hb and hb.get_srcSystem() == _connection.target_system:
                return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        except Exception:
            pass
    return False


def disarm():
    """Disarm the FC."""
    if not _enabled:
        return
    from pymavlink import mavutil
    # Flask is threaded — set_guided_mode()/arm() and disarm() share this lock
    # so an overlapping call (e.g. a stale UI auto-disarm racing a real
    # launch click) can't interleave mode/arm commands with this one.
    with _launch_lock:
        try:
            armed = _is_armed()
            print(f"[MAVLink] FC is {'ARMED' if armed else 'DISARMED'} — {'sending disarm' if armed else 'nothing to do'}")
            if not armed:
                global _launched
                _launched = False
                return
            if _autopilot == "px4":
                # PX4 MANUAL: main_mode=1, sub_mode=0 — "0" is not a valid PX4
                # main_mode (that's ArduPlane's MANUAL index, not PX4's).
                base_mode = (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                             | mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
                             | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                custom_mode = _PX4_MAIN_MODE_MANUAL  # raw, no <<16 — see set_guided_mode()
            else:
                base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                custom_mode = 0  # ArduPlane MANUAL — most permissive for disarm
            _connection.mav.command_long_send(
                _connection.target_system,
                _connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                base_mode,
                custom_mode,
                0, 0, 0, 0, 0
            )
            time.sleep(0.5)
            # Neutral all surfaces + zero throttle
            _connection.mav.rc_channels_override_send(
                _connection.target_system,
                _connection.target_component,
                1500, 1500, 1000, 1500,   # roll, pitch, throttle, yaw → neutral/min
                65535, 65535, 65535, 65535
            )
            time.sleep(1.0)
            _connection.mav.command_long_send(
                _connection.target_system,
                _connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,      # disarm
                21196,  # force
                0, 0, 0, 0, 0
            )
            time.sleep(0.5)
            still_armed = _is_armed()
            print(f"[MAVLink] DISARM {'succeeded' if not still_armed else 'FAILED — FC still armed'}")
            _launched = False
        except Exception as e:
            print(f"[MAVLink] disarm error: {e}")


def set_guided_mode():
    """Switch into GUIDED (ArduPilot) or OFFBOARD (PX4) mode, without arming.
    Called automatically at the end of connect() so the FC is already in the
    right mode well before Launch is pressed — arm() then only has to arm.
    Relies on the main loop streaming a neutral-hold attitude target
    continuously from connect time onward (not gated on "launched"), since
    PX4 exits OFFBOARD if the setpoint stream stops even briefly."""
    if not _enabled:
        print("[MAVLink] Not connected — skipping mode switch")
        return
    from pymavlink import mavutil
    with _launch_lock:
        try:
            if _autopilot == "px4":
                # PX4 rejects the switch into OFFBOARD unless a setpoint stream is
                # already flowing — prime it with a few no-op attitude targets
                # before requesting the mode change.
                for _ in range(10):
                    send_attitude_target(0.0, 0.0, thrust=0.0)
                    time.sleep(0.05)
                base_mode = (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                             | mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED
                             | mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
                             | mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED)
                # NOTE: no <<16 shift here. That packed encoding (main_mode in
                # bits 16-23) is only for the 32-bit custom_mode field of
                # HEARTBEAT / the legacy SET_MODE message. MAV_CMD_DO_SET_MODE
                # sent via COMMAND_LONG is different: PX4's mavlink_receiver
                # forwards param2 straight through, and Commander reads it as
                # (uint8_t)param2 — the raw main_mode number, unshifted. A
                # shifted value here truncates to 0 on the FC side and matches
                # no valid mode, so the switch silently never takes effect.
                custom_mode = _PX4_MAIN_MODE_OFFBOARD
                mode_label = "OFFBOARD"
                # A single DO_SET_MODE can land while the link is congested
                # (e.g. MAVProxy's post-connect param bulk fetch) right when
                # PX4 checks setpoint recency, and get silently ignored with
                # no STATUSTEXT. Retry a few times, interleaved with attitude
                # targets, so the setpoint stream stays fresh across attempts.
                for attempt in range(5):
                    _connection.mav.command_long_send(
                        _connection.target_system,
                        _connection.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        0,
                        base_mode,
                        custom_mode,
                        0, 0, 0, 0, 0
                    )
                    print(f"[MAVLink] {mode_label} mode command sent (attempt {attempt + 1}/5)")
                    send_attitude_target(0.0, 0.0, thrust=0.0)
                    time.sleep(0.2)
            else:
                base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                custom_mode = 15  # ArduPlane GUIDED
                mode_label = "GUIDED"
                _connection.mav.command_long_send(
                    _connection.target_system,
                    _connection.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    base_mode,
                    custom_mode,
                    0, 0, 0, 0, 0
                )
                print(f"[MAVLink] {mode_label} mode command sent")
        except Exception as e:
            print(f"[MAVLink] set_guided_mode error: {e}")


def arm():
    """Arm the FC. Assumes GUIDED/OFFBOARD mode was already set by
    set_guided_mode() (called automatically at connect time) — this now
    only arms, so Launch doesn't have to wait on the mode-switch retries."""
    if not _enabled:
        print("[MAVLink] Not connected — skipping arm")
        return
    from pymavlink import mavutil
    with _launch_lock:
        try:
            _connection.mav.command_long_send(
                _connection.target_system,
                _connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,      # arm
                21196,  # force
                0, 0, 0, 0, 0
            )
            print("[MAVLink] ARM command sent")
            global _launched
            _launched = True
        except Exception as e:
            print(f"[MAVLink] arm error: {e}")


def send_attitude_target(pitch_err, yaw_err, roll_err=0.0, thrust=0.5):
    """Send SET_ATTITUDE_TARGET every frame. pitch_err/yaw_err/roll_err are
    BODY-FRAME camera tracking errors (radians), clamped to MAX_ANGLE.

    ArduPlane's GUIDED handler treats this quaternion's axes completely
    differently from each other (confirmed against ArduPlane source,
    Attitude.cpp/GCS_MAVLink_Plane.cpp):
      - roll/pitch extracted from the quaternion become nav_roll_cd /
        nav_pitch_cd, ABSOLUTE angle targets fed into Plane's normal
        attitude PID (which computes its own error against current
        attitude) — so "camera centered" must compose onto the FC's
        current roll/pitch to mean "hold here".
      - yaw extracted from the quaternion is assigned STRAIGHT to
        commanded_rudder with no reference to current heading at all —
        it's an open-loop rudder deflection, not an attitude target.
        Composing it onto current heading turns the rudder command into
        ~absolute compass heading in centidegrees, saturating the rudder
        toward whatever direction happens to be "north" instead of
        responding to the body-frame camera error. So on ArduPilot yaw is
        sent RAW (just the clamped camera error, never composed).

    PX4 (fixed-wing, no ailerons — yaw done via dedicated rudder surfaces,
    CA_SV_CS0/CS2 TRQ_Y=1.0 per mav.parm): tried yaw as an absolute
    attitude-quaternion angle, then as an explicit body_yaw_rate mixed
    alongside the quaternion for roll/pitch — neither moved the rudder.
    Confirmed via QGC MAVLink Inspector: the ATTITUDE_TARGET echo showed
    our commanded yaw_rate replaced by a tiny (~0.01-0.03 rad/s) value
    that tracked roll (near 0, since roll is never commanded), not our
    input. That means PX4's FW_ATT_CONTROL, whenever the attitude
    quaternion is not ignored, owns the rate setpoint for ALL axes
    (deriving yaw's from bank-angle turn coordination) and silently
    overwrites/ignores any per-axis body rate we also provide — there is
    no real per-axis attitude/rate mix on PX4 FW.
    So PX4 now runs in full RATE mode instead: type_mask sets
    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE, so the quaternion (q) is
    unused (current attitude sent as a harmless placeholder) and ALL
    THREE axes are driven as explicit body rate setpoints, computed here
    as a simple proportional mapping (clamped angle error, in rad, used
    directly as rad/s) — same convention already used for yaw. This
    bypasses FW_ATT_CONTROL's angle loop (and its turn-coordination yaw
    logic) entirely, going straight to the rate controller/CA allocation
    for roll, pitch AND yaw. ArduPlane is untouched — still absolute
    quaternion for roll/pitch, raw yaw (see above), proven working.

    Skips the send if the cached current-attitude reading is stale (older
    than MAX_ATTITUDE_AGE) rather than command a wrong roll/pitch target.
    Call at ~10 Hz or faster."""
    if not _enabled:
        if DEBUG:
            print(f"[DEBUG] pitch_err={math.degrees(pitch_err):.2f}° yaw_err={math.degrees(yaw_err):.2f}°")
        return
    with _attitude_lock:
        current = _current_attitude
        age = time.time() - _attitude_timestamp
    if current is None or age > MAX_ATTITUDE_AGE:
        if DEBUG:
            print(f"[MAVLink] set_attitude_target skipped: attitude stale/missing (age={age:.2f}s)")
        return
    roll  = max(-MAX_ANGLE, min(MAX_ANGLE, roll_err))
    pitch = max(-MAX_ANGLE, min(MAX_ANGLE, pitch_err))
    yaw   = max(-MAX_ANGLE, min(MAX_ANGLE, yaw_err))
    from pymavlink.quaternion import QuaternionBase
    try:
        q_current = QuaternionBase([current[0], current[1], current[2]])
        if _autopilot == "px4":
            # Full rate mode: quaternion is ignored, all three axes are
            # explicit body rate setpoints (rad/s), bypassing FW_ATT_CONTROL's
            # angle loop entirely (see docstring).
            q = q_current  # placeholder; ATTITUDE_IGNORE bit means it's unused
            body_roll_rate, body_pitch_rate, body_yaw_rate = roll, pitch, yaw
            # NOTE: bit 6 (0b01000000=64) is ATTITUDE_TARGET_TYPEMASK_THRUST_IGNORE,
            # NOT ignore-attitude — that mistake sent q=current (zero error, no
            # driven pitch/roll signal) and zeroed thrust (confirmed via QGC
            # showing thrust=0). ignore_attitude is bit 7 (0b10000000=128).
            type_mask = 0b10000000  # ignore attitude only -> use body rates + thrust
            _connection.mav.set_attitude_target_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                _connection.target_system,
                _connection.target_component,
                type_mask,
                q,
                body_roll_rate, body_pitch_rate, body_yaw_rate,
                thrust
            )
            if DEBUG:
                print(f"[MAVLink] SET_ATTITUDE_TARGET (rate mode) age={age*1000:.0f}ms "
                      f"sent_rates=(roll={math.degrees(body_roll_rate):+.2f}°/s,"
                      f"pitch={math.degrees(body_pitch_rate):+.2f}°/s,"
                      f"yaw={math.degrees(body_yaw_rate):+.2f}°/s) thrust={thrust:.2f}")
        else:
            # ArduPlane: compose only roll/pitch -> absolute nav_roll_cd/
            # nav_pitch_cd target. Yaw is deliberately left OUT of this
            # composition (see docstring) and spliced in raw below.
            q_delta_rp = QuaternionBase([roll, pitch, 0.0])
            roll_target, pitch_target, _ = (q_current * q_delta_rp).euler
            yaw_target = yaw
            q = QuaternionBase([roll_target, pitch_target, yaw_target])
            type_mask = 0b00000111  # ignore all body rates, use quaternion + thrust
            _connection.mav.set_attitude_target_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                _connection.target_system,
                _connection.target_component,
                type_mask,
                q,
                0.0, 0.0, 0.0,
                thrust
            )
            if DEBUG:
                print(f"[MAVLink] SET_ATTITUDE_TARGET composed_on=({math.degrees(current[0]):+.1f},"
                      f"{math.degrees(current[1]):+.1f},{math.degrees(current[2]):+.1f})° "
                      f"age={age*1000:.0f}ms err=({math.degrees(roll):+.2f},{math.degrees(pitch):+.2f},"
                      f"{math.degrees(yaw):+.2f})° "
                      f"sent_rp=({math.degrees(roll_target):+.1f},{math.degrees(pitch_target):+.1f})° "
                      f"sent_yaw_raw={math.degrees(yaw_target):+.2f}° thrust={thrust:.2f}")
    except Exception as e:
        print(f"[MAVLink] set_attitude_target failed: {e}")
