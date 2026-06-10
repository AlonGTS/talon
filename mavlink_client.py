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
DEBUG           = False  # set True to print pitch/yaw values every frame
SHOW_TELEMETRY  = False  # set True to print incoming ATTITUDE in the console


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
        "/home/mahat/webrtc_venv/bin/mavproxy.py",
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
        _connection.wait_heartbeat(timeout=5)
        _enabled = True
        print(f"[MAVLink] Connected via MAVProxy ({url}), heartbeat received.")
    except Exception as e:
        print(f"[WARNING] MAVLink primary connection failed: {e}")
        if fallback_url:
            try:
                _connection = mavutil.mavlink_connection(fallback_url)
                _enabled = True
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

def _telemetry_reader():
    while True:
        if not _enabled or _connection is None:
            time.sleep(0.5)
            continue
        try:
            msg = _connection.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            if msg and SHOW_TELEMETRY:
                import math as _math
                print(f"[Telem] roll={_math.degrees(msg.roll):+.1f}°  "
                      f"pitch={_math.degrees(msg.pitch):+.1f}°  "
                      f"yaw={_math.degrees(msg.yaw):+.1f}°")
        except Exception as e:
            print(f"[Telem] read error: {e}")
            time.sleep(0.5)

_telem_thread = threading.Thread(target=_telemetry_reader, daemon=True)
_telem_thread.start()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def disarm():
    """Disarm the FC. Call on exit."""
    if not _enabled:
        return
    from pymavlink import mavutil
    try:
        _connection.mav.command_long_send(
            _connection.target_system,
            _connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,      # 0 = disarm
            21196,  # force disarm — bypasses in-air / safety checks
            0, 0, 0, 0, 0
        )
        print("[MAVLink] DISARM command sent")
        global _launched
        _launched = False
    except Exception as e:
        print(f"[MAVLink] disarm failed: {e}")


def arm_and_set_guided():
    """Set GUIDED mode and arm the FC. Call once after connect()."""
    if not _enabled:
        print("[MAVLink] Not connected — skipping arm/GUIDED")
        return
    from pymavlink import mavutil
    _connection.mav.command_long_send(
        _connection.target_system,
        _connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        15,  # GUIDED mode number for ArduPlane
        0, 0, 0, 0, 0
    )
    print("[MAVLink] GUIDED mode command sent")
    time.sleep(0.5)
    _connection.mav.command_long_send(
        _connection.target_system,
        _connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,      # 1 = arm
        21196,  # force arm — bypasses pre-arm checks (logging, GPS, etc.)
        0, 0, 0, 0, 0
    )
    print("[MAVLink] ARM command sent")
    global _launched
    _launched = True


def send_attitude_target(pitch, yaw, roll=0.0, thrust=0.5):
    """Send SET_ATTITUDE_TARGET every frame. Units: radians. Call at ~10 Hz or faster."""
    if not _enabled:
        if DEBUG:
            print(f"[DEBUG] pitch={math.degrees(pitch):.2f}° yaw={math.degrees(yaw):.2f}°")
        return
    from pymavlink.quaternion import QuaternionBase
    try:
        q = QuaternionBase([roll, pitch, yaw])
        _connection.mav.set_attitude_target_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            _connection.target_system,
            _connection.target_component,
            0b00000111,        # ignore body rates, use quaternion + thrust
            q,
            0.0, 0.0, 0.0,    # body roll/pitch/yaw rates (ignored)
            thrust
        )
        if DEBUG:
            print(f"[MAVLink] SET_ATTITUDE_TARGET pitch={math.degrees(pitch):.2f}° "
                  f"yaw={math.degrees(yaw):.2f}° thrust={thrust:.2f}")
    except Exception as e:
        print(f"[MAVLink] set_attitude_target failed: {e}")
