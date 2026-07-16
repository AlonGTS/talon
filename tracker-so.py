#!/usr/bin/env python3
# === Imports ===
# Core vision / math / utils
import cv2
import time
import math
import numpy as np

# Concurrency primitives
from threading import Thread, Lock, Condition
import threading
from types import SimpleNamespace

# CLI args / timestamps / small GUI dialogs for file/duration picking
import tomllib
from pathlib import Path

_HERE = Path(__file__).parent
import argparse
from datetime import datetime
import tkinter as tk
from tkinter import filedialog, simpledialog

import webrtc_server
from gts_tracker import GTSTracker


# ── Tracking Quality Monitor ─────────────────────────────────────────────────
class TrackingQualityMonitor:
    """
    Per-frame confidence score [0.0–1.0] computed on top of CSRT.

    CSRT's internal PSR is not exposed via OpenCV's Python bindings; this
    class approximates it from three observable signals:

      • Frame-to-frame NCC (55 %)  — compares the current ROI patch to the
        patch from the PREVIOUS frame.  Between consecutive frames the scale
        change is tiny even when the drone is approaching the target, so NCC
        stays high on correct tracking and drops sharply on a drift event.
        (Comparing to the *initial* template would fail as the drone closes in.)

      • Velocity gate        (30 %)  — penalises bbox-centre jumps > 15 % of
        the frame's larger dimension in a single step (teleportation = drift).

      • Size-change gate     (15 %)  — penalises sudden bbox area changes
        larger than 4× in one frame (unphysical growth/shrink).

    Thresholds
    ----------
    score ≥ SCORE_GOOD        → green  box, attitude control enabled
    score ≥ SCORE_UNCERTAIN   → orange box, attitude control still enabled (visible warn)
    score <  SCORE_UNCERTAIN  → red    box, attitude control suppressed
    score <  SCORE_UNCERTAIN for BAD_FRAMES_LIMIT consecutive frames → tracking broken
    """

    SCORE_GOOD        = 0.60
    SCORE_UNCERTAIN   = 0.40
    BAD_FRAMES_LIMIT  = 1      # 1 bad frame breaks tracking immediately
    _TMPL_SIZE        = (64, 64)   # canonical patch size for NCC

    def __init__(self):
        self._prev_patch = None    # grayscale 64×64 from previous frame
        self._prev_cx    = None
        self._prev_cy    = None
        self._prev_area  = None
        self.score       = 1.0
        self.bad_frames  = 0      # consecutive frames below SCORE_UNCERTAIN

    # ── public API ────────────────────────────────────────────────────────

    def init(self, frame: np.ndarray, bbox: tuple):
        """
        Capture the first appearance patch and reset history.
        Call this on the first successful update after a new tracker is
        initialised (pass lores frame + lores bbox).
        """
        x, y, w, h = (int(v) for v in bbox)
        patch = self._safe_crop(frame, x, y, w, h)
        if patch is not None and patch.size > 0:
            gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
            self._prev_patch = cv2.resize(gray, self._TMPL_SIZE)
        else:
            self._prev_patch = None
        self._prev_cx   = x + w // 2
        self._prev_cy   = y + h // 2
        self._prev_area = max(1, w * h)
        self.score      = 1.0

    def update(self, frame: np.ndarray, bbox: tuple) -> float:
        """
        Compute confidence for this frame after a successful CSRT update.
        frame and bbox must be in lores coordinate space.
        Returns score ∈ [0.0, 1.0] and advances internal state.
        """
        if bbox is None:
            self.score = 0.0
            return self.score

        xl, yl, wl, hl = (int(v) for v in bbox)
        cx, cy         = xl + wl // 2, yl + hl // 2
        curr_area      = max(1, wl * hl)

        # ── Frame-to-frame NCC ────────────────────────────────────────────
        patch = self._safe_crop(frame, xl, yl, wl, hl)
        if patch is not None and patch.size > 0:
            gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
            curr = cv2.resize(gray, self._TMPL_SIZE)
            if self._prev_patch is not None:
                res       = cv2.matchTemplate(curr, self._prev_patch, cv2.TM_CCOEFF_NORMED)
                ncc_score = float(np.clip(res[0, 0], 0.0, 1.0))
            else:
                ncc_score = 0.5          # no previous patch yet → neutral
            self._prev_patch = curr      # roll forward
        else:
            ncc_score = 0.5              # patch out of bounds → neutral

        # ── Velocity gate ─────────────────────────────────────────────────
        if self._prev_cx is not None:
            fh, fw    = frame.shape[:2]
            jump      = math.hypot(cx - self._prev_cx, cy - self._prev_cy)
            max_jump  = max(fw, fh) * 0.15
            vel_score = float(np.clip(1.0 - jump / max_jump, 0.0, 1.0))
        else:
            vel_score = 1.0
        self._prev_cx, self._prev_cy = cx, cy

        # ── Size-change gate ──────────────────────────────────────────────
        if self._prev_area is not None:
            ratio      = max(curr_area, self._prev_area) / min(curr_area, self._prev_area)
            # ratio 1.0 → score 1.0 | ratio ≥ 4.0 → score 0.0 (linear)
            size_score = float(np.clip(1.0 - (ratio - 1.0) / 3.0, 0.0, 1.0))
        else:
            size_score = 1.0
        self._prev_area = curr_area

        # ── Combined score ────────────────────────────────────────────────
        self.score = 0.55 * ncc_score + 0.30 * vel_score + 0.15 * size_score
        return self.score

    def reset(self):
        self._prev_patch = None
        self._prev_cx    = self._prev_cy = None
        self._prev_area  = None
        self.score       = 1.0
        self.bad_frames  = 0

    # ── helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _safe_crop(frame, x, y, w, h):
        fh, fw = frame.shape[:2]
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(fw, x + w), min(fh, y + h)
        if x2 <= x1 or y2 <= y1:
            return None
        return frame[y1:y2, x1:x2]


# Load configuration
with open(_HERE / "config.toml", "rb") as _f:
    _cfg = tomllib.load(_f)

SHOW_LOCAL    = _cfg["display"]["show_local"]
MAX_BB_WIDTH  = _cfg["tracking"]["max_bb_width"]
MAX_BB_HEIGHT = _cfg["tracking"]["max_bb_height"]
MAIN_SIZES    = [tuple(s) for s in _cfg["camera"]["main_sizes"]]
LORES_SIZES   = [tuple(s) for s in _cfg["camera"]["lores_sizes"]]

_net_iface  = _cfg["network"]["interface"]
_net        = _cfg["network"][_net_iface]
BIND_IP     = _net["bind_ip"]
_VIDEO_MODE = _cfg["network"].get("video_mode", "jpeg_udp")
GCS_IP      = None   # learned dynamically from GCS heartbeat packets
print(f"[NET] interface={_net_iface}  bind={BIND_IP}  gcs=<waiting for GCS hello>")

# Shared frame buffer for WebRTC
frame_buffer = webrtc_server.FrameBuffer()

# === Global shared state (protected by frame_ready) ===
output_frame = None               # Final frame after overlays → served to MJPEG / WebRTC

# Mutable state shared between main loop, reader threads, and Flask/WebRTC
state = SimpleNamespace(
    current_frame   = None,   # Raw latest frame from camera/file (no overlays)
    frame_seq       = 0,      # Bumped each time current_frame is replaced; lets the
                               # main loop tell "new frame" apart from "same frame again"
    command_from_remote = None,   # One-letter command from web UI: 'r','s','q'
    bbox            = None,   # Current CSRT tracking box (MAIN coords: x, y, w, h)
    tracking        = False,  # Tracking on/off flag
    tracker         = None,   # OpenCV tracker object (runs on LORES frame)
    bMoovingTgt     = False,  # Target type (False=fixed, True=moving)
    lores_size      = None,   # Filled after config load below
)

_main_idx = 0       # index into MAIN_SIZES
_lores_idx = 0      # index into LORES_SIZES

main_size = list(MAIN_SIZES[_main_idx])    # [W, H] for capture/preview/output
lores_size = list(LORES_SIZES[_lores_idx]) # [W, H] for tracking
state.lores_size = lores_size

# Playback controls (used only in playback mode)
playback_rate = 1.0
seek_to_msec = None
playback_ctrl_lock = Lock()

# Playback telemetry (reader updates; main loop reads to sync trackbars)
playback_duration_ms = 0.0
playback_pos_ms = 0.0

# Local-UI (OpenCV) trackbar flags (playback-only)
_trackbar_ready = False
_suppress_trackbar_cb = False

# FPS meter (rough)
_prev_ts = time.time()
_fps_alpha = 0.9
_est_fps = 0.0

# Thread sync for frame sharing between producer (reader) and consumers (MJPEG/WebRTC)
frame_lock = Lock()
frame_ready = Condition(frame_lock)
state.frame_lock = frame_lock

# ============ Camera/File Reader (unified) ============
cap = None
picam2 = None
_reader_thread = None
_stop_reader = threading.Event()

def _reader_playback(path, loop=False):
    """
    Video file reader that respects playback_rate and seek_to_msec globals.
    Updates playback_pos_ms and playback_duration_ms for UI sync.
    """
    global cap, playback_rate, seek_to_msec
    global playback_duration_ms, playback_pos_ms

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open video file: {path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    base_delay = 1.0 / fps
    total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0
    playback_duration_ms = (total_frames / fps * 1000.0) if total_frames > 0 else 0.0

    print(f"[INFO] Playback started ({fps:.1f} fps) duration≈{playback_duration_ms/1000:.2f}s")

    while not _stop_reader.is_set():
        with playback_ctrl_lock:
            if seek_to_msec is not None:
                cap.set(cv2.CAP_PROP_POS_MSEC, float(seek_to_msec))
                seek_to_msec = None
            rate = max(0.1, float(playback_rate))

        ok, frame = cap.read()
        if not ok:
            if loop:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            else:
                print("[INFO] Playback ended")
                break

        playback_pos_ms = cap.get(cv2.CAP_PROP_POS_MSEC)

        with frame_ready:
            state.current_frame = frame
            state.frame_seq += 1
            frame_ready.notify_all()

        # Adjust pacing
        if rate > 1.0:
            frames_to_skip = int(rate) - 1
            for _ in range(frames_to_skip):
                cap.grab()
            time.sleep(base_delay * 0.25)
        else:
            time.sleep(base_delay / rate)

def _init_live_camera():
    """(Re)create and start PiCamera2 with the current main_size."""
    global picam2
    from picamera2 import Picamera2
    if picam2 is not None:
        try: picam2.stop()
        except Exception: pass
        try: picam2.close()
        except Exception: pass
        picam2 = None

    picam2 = Picamera2()
    w, h = main_size
    picam2.preview_configuration.main.size = (int(w), int(h))
    picam2.preview_configuration.main.format = "RGB888"
    picam2.configure("preview")
    picam2.set_controls({"FrameRate": 30})
    picam2.start()
    print(f"[LIVE] Camera started MAIN={w}x{h}")

def _reader_live_picam():
    """Continuously read frames from PiCamera2 and publish into current_frame."""
    global picam2
    print("[INFO] Live reader started (PiCamera2)")
    while not _stop_reader.is_set():
        frame = picam2.capture_array()
        if frame is None:
            continue
        with frame_ready:
            state.current_frame = frame  # raw MAIN frame only
            state.frame_seq += 1
            frame_ready.notify_all()

def _restart_reader_live():
    """Stop live reader, reinit camera (for new MAIN size), and restart reader."""
    global _reader_thread
    _stop_reader.set()
    if _reader_thread and _reader_thread.is_alive():
        _reader_thread.join(timeout=1.0)
    _stop_reader.clear()
    _init_live_camera()
    _reader_thread = Thread(target=_reader_live_picam, daemon=True)
    _reader_thread.start()


# === MAVLink Setup ===
import mavlink_client
_mav = _cfg["mavlink"]
mavlink_client.set_autopilot(_mav.get("autopilot", "ardupilot"))
mavlink_client.start_mavproxy(
    pixhawk_port  = _mav["pixhawk_port"],
    pixhawk_baud  = _mav["pixhawk_baud"],
    gcs_port      = _mav["gcs_port"],
    local_port    = _mav["local_port"],
    extra_outputs = _mav.get("extra_outputs", []),
)
mavlink_client.connect(url=f"udpin:0.0.0.0:{_mav['local_port']}")

# === Command-line arguments setup ===
parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=['live', 'record', 'playback'], default='live')
parser.add_argument('--video', help='Path to video file for playback')
parser.add_argument('--duration', type=int, help='Duration to record (seconds)')
parser.add_argument('--loop', action='store_true', help='Loop video in playback mode')
args = parser.parse_args()

# If recording and duration not provided on CLI, ask with a small dialog (Tk)
if args.mode == 'record' and not args.duration:
    root = tk.Tk(); root.withdraw()
    duration = simpledialog.askinteger("Recording Duration", "How many seconds to record?",
                                       minvalue=1, maxvalue=3600)
    if not duration:
        print("[ERROR] No duration selected. Exiting.")
        exit(1)
    args.duration = duration
    print(f"[INFO] Recording duration set to {args.duration} seconds")

# === Input Setup: file playback or live camera (each spawns a reader thread) ===
if args.mode == 'playback':
    if not args.video:
        root = tk.Tk(); root.withdraw()
        args.video = filedialog.askopenfilename(
            title="Select video file",
            filetypes=[("Video files", "*.avi *.mp4 *.mov *.mkv"), ("All files", "*.*")]
        )
        if not args.video:
            print("[ERROR] No file selected. Exiting...")
            exit(1)
    print(f"[INFO] Playback mode from file: {args.video} (loop={args.loop})")
    _stop_reader.clear()
    _reader_thread = Thread(target=_reader_playback, args=(args.video, args.loop), daemon=True)
    _reader_thread.start()
else:
    _stop_reader.clear()
    _init_live_camera()
    _reader_thread = Thread(target=_reader_live_picam, daemon=True)
    _reader_thread.start()

# === GTS Tracker (compiled module) ===
def create_csrt_tracker(moving: bool):
    return GTSTracker(mode="moving" if moving else "fixed")

# === Video Recording Setup (mode=record) ===
writer = None
record_start_time = None
if args.mode == 'record':
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video_filename = f"RecordingsMahat/recording_{timestamp}.avi"
    writer = cv2.VideoWriter(video_filename, fourcc, 60.0, (640, 480))
    record_start_time = time.time()
    print(f"[INFO] Recording to {video_filename}")

# === Local debug window (optional) ===
if SHOW_LOCAL:
    cv2.namedWindow("Tracker")

# ---- Trackbar callbacks (playback-only) ----
def _on_seek_trackbar(pos):
    """
    OpenCV trackbar callback for playback seek.
    Converts the trackbar position (0–1000) to a millisecond timestamp
    and stores it in seek_to_msec for the reader thread to act on.
    Suppressed while the main loop is updating the trackbar programmatically.
    """
    global seek_to_msec, _suppress_trackbar_cb
    if _suppress_trackbar_cb or playback_duration_ms <= 0:
        return
    frac = pos / 1000.0
    with playback_ctrl_lock:
        seek_to_msec = int(frac * playback_duration_ms)

def _on_rate_trackbar(val):
    """
    OpenCV trackbar callback for playback speed.
    Maps the trackbar integer value to a playback rate in the range [0.1, 8.0]×.
    Suppressed while the main loop is syncing the trackbar to the current rate.
    """
    global playback_rate, _suppress_trackbar_cb
    if _suppress_trackbar_cb:
        return
    r = max(0.1, min(8.0, val / 100.0))
    with playback_ctrl_lock:
        playback_rate = r

if SHOW_LOCAL and args.mode == 'playback':
    cv2.createTrackbar('position', 'Tracker', 0, 1000, _on_seek_trackbar)
    cv2.createTrackbar('rate x0.01', 'Tracker', int(100), 800, _on_rate_trackbar)
    _trackbar_ready = True

# === Mouse callback (local window) ===
def draw_rectangle(event, x, y, flags, param):
    """
    Local GUI selection: left-click initializes a new tracker centered at (x,y) in MAIN coords.
    """
    if event == cv2.EVENT_LBUTTONDOWN and state.current_frame is not None:
        frame_for_init = state.current_frame.copy()
        w, h = (30, 30) if state.bMoovingTgt else (80, 80)
        x0 = max(0, x - w//2); y0 = max(0, y - h//2)
        bbox_main = (x0, y0, w, h)
        # Convert MAIN → LORES
        mw, mh = frame_for_init.shape[1], frame_for_init.shape[0]
        lw, lh = state.lores_size
        sx = lw / mw; sy = lh / mh
        xb = int(x0 * sx); yb = int(y0 * sy)
        wb = max(2, int(w * sx)); hb = max(2, int(h * sy))
        lores_frame = cv2.resize(frame_for_init, (lw, lh), interpolation=cv2.INTER_LINEAR)

        tracker_local = create_csrt_tracker(state.bMoovingTgt)
        tracker_local.init(lores_frame, (xb, yb, wb, hb))

        state.tracker = tracker_local
        state.bbox = bbox_main
        state.tracking = True
        print(f"[INFO] Tracker init (MAIN) at ({x},{y}), box {w}x{h} | LORES {lw}x{lh}")

if SHOW_LOCAL:
    cv2.setMouseCallback("Tracker", draw_rectangle)


# === Helpers to cycle resolutions (LIVE mode only) ===
def _cycle_main(delta):
    global _main_idx, main_size
    _main_idx = (_main_idx + delta) % len(MAIN_SIZES)
    main_size = list(MAIN_SIZES[_main_idx])
    print(f"[LIVE] Reconfig MAIN → {main_size[0]}x{main_size[1]} (restart reader)")
    _restart_reader_live()

def _cycle_lores(delta):
    global _lores_idx
    _lores_idx = (_lores_idx + delta) % len(LORES_SIZES)
    state.lores_size = list(LORES_SIZES[_lores_idx])
    print(f"[TRACK] LORES → {state.lores_size[0]}x{state.lores_size[1]}")


import flask_app
app = flask_app.create_app(
    state, create_csrt_tracker,
    cycle_main_fn      = _cycle_main if args.mode == 'live' else None,
    cycle_lores_fn     = _cycle_lores,
    launch_fn          = lambda v=None: mavlink_client.arm_and_set_guided() if (v is None or v) else mavlink_client.disarm(),
    get_launch_state_fn= lambda: mavlink_client._launched,
)

# === Launch Flask in separate thread ===
print(f"[Flask]  http://{BIND_IP}:5000")
flask_thread = Thread(target=lambda: app.run(host="0.0.0.0", port=5000, threaded=True))
flask_thread.daemon = True
flask_thread.start()

# === Video stream (mode selected by config.toml video_mode) ===
import socket as _socket
import json as _json
import requests as _req

if _VIDEO_MODE == "webrtc":
    print(f"[WebRTC] http://{BIND_IP}:8080")
    webrtc_thread = Thread(target=webrtc_server.start, args=(frame_buffer,), daemon=True)
    webrtc_thread.start()

elif _VIDEO_MODE == "jpeg_udp":
    _UDP_PORT     = _cfg["network"].get("gcs_udp_port", 5600)
    _JPEG_QUALITY = _cfg["network"].get("gcs_jpeg_quality", 40)
    _STREAM_WIDTH = _cfg["network"].get("gcs_stream_width", 640)
    _udp_sock     = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
    _udp_sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_SNDBUF, 1 << 20)
    _JPEG_PARAMS  = [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY]
    _UDP_MAX      = 65400

    def _udp_stream_worker():
        last_gen = -1
        _t0, _sent = time.time(), 0
        print(f"[UDP]  stream worker ready — waiting for GCS to announce")
        while True:
            frame, gen = frame_buffer.get(last_gen=last_gen, timeout=0.1)
            if frame is None or GCS_IP is None:
                continue
            last_gen = gen
            _sent += 1
            _now = time.time()
            if _now - _t0 >= 5.0:
                print(f"[UDP]  {_sent/(_now-_t0):.1f} fps ({_sent} frames) → {GCS_IP}")
                _t0, _sent = _now, 0
            h_f, w_f = frame.shape[:2]
            if w_f > _STREAM_WIDTH:
                scale = _STREAM_WIDTH / w_f
                frame = cv2.resize(frame, (_STREAM_WIDTH, int(h_f * scale)), interpolation=cv2.INTER_LINEAR)
            ok, buf = cv2.imencode('.jpg', frame, _JPEG_PARAMS)
            if not ok:
                continue
            data = buf.tobytes()
            if len(data) > _UDP_MAX:
                continue
            try:
                _udp_sock.sendto(data, (GCS_IP, _UDP_PORT))
            except Exception as e:
                print(f"[UDP]  send error: {e}")

    Thread(target=_udp_stream_worker, daemon=True).start()

else:
    print(f"[WARN] Unknown video_mode '{_VIDEO_MODE}' — no video stream started")

# === UDP command channel — learns GCS IP from heartbeat, forwards commands to Flask ===
_CMD_PORT = _cfg["network"].get("gcs_cmd_port", 5601)
_cmd_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
_cmd_sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
_cmd_sock.bind(('', _CMD_PORT))

def _udp_cmd_listener():
    global GCS_IP
    print(f"[CMD]  listening for commands on UDP:{_CMD_PORT}")
    while True:
        try:
            data, addr = _cmd_sock.recvfrom(4096)
            if GCS_IP != addr[0]:
                print(f"[NET]  GCS IP {'learned' if GCS_IP is None else 'updated'}: {addr[0]}")
                GCS_IP = addr[0]
            msg = _json.loads(data.decode())
            ep  = msg.pop("endpoint", None)
            if ep:
                _req.post(f"http://127.0.0.1:5000/{ep}", data=msg, timeout=1)
        except Exception as e:
            if "timed out" not in str(e).lower():
                print(f"[CMD]  {e}")

Thread(target=_udp_cmd_listener, daemon=True).start()

_tq_monitor      = TrackingQualityMonitor()
_last_tracker_id = None   # detect tracker replacement from ANY init path (flask, mouse, BB-clamp)
_tq_needs_init   = True   # capture first patch on next successful update

# === Main Loop (render & publish) ===
_last_frame_seq = -1
while True:
    # Wait for a new current_frame from reader (frame_seq only advances when the
    # reader publishes one — without this the loop free-spins reprocessing the
    # same frame once current_frame stops being None, flooding MAVLink sends)
    with frame_ready:
        if state.current_frame is None or state.frame_seq == _last_frame_seq:
            frame_ready.wait(timeout=0.02)
        if state.current_frame is None or state.frame_seq == _last_frame_seq:
            frame = None
        else:
            frame = state.current_frame.copy()
            _last_frame_seq = state.frame_seq

    if frame is None:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        continue

    mh, mw = frame.shape[:2]
    lw, lh = state.lores_size
    sx_m2l = lw / mw
    sy_m2l = lh / mh
    sx_l2m = mw / lw
    sy_l2m = mh / lh

    # Handle commands
    if state.command_from_remote == 'r':
        state.tracking = False; state.bbox = None; state.tracker = None
        print("[INFO] Tracker reset from remote")
        state.command_from_remote = None
    elif state.command_from_remote == 's':
        state.tracking = False
        state.command_from_remote = None
    elif state.command_from_remote == 'q':
        print("[INFO] Quit requested from remote")
        break

    # Tracking on LORES
    lores_frame = cv2.resize(frame, (lw, lh), interpolation=cv2.INTER_LINEAR)
    _attitude_sent = False

    # Detect tracker replacement from ANY init path (flask_app.py's remote
    # target-select, the local mouse callback, or the BB-clamp recreate below)
    # — comparing object identity here catches all of them without needing a
    # reset call at every individual creation site.
    if state.tracker is not None and id(state.tracker) != _last_tracker_id:
        _last_tracker_id = id(state.tracker)
        _tq_needs_init   = True
        _tq_monitor.reset()

    if state.tracking and state.tracker is not None:
        try:
            success, bbox_lo = state.tracker.update(lores_frame)
            if success:
                xl, yl, wl, hl = map(int, bbox_lo)
                x = int(xl * sx_l2m); y = int(yl * sy_l2m)
                bw = max(2, int(wl * sx_l2m)); bh = max(2, int(hl * sy_l2m))
                cx, cy = x + bw // 2, y + bh // 2

                # Clamp bbox growth
                if bw > MAX_BB_WIDTH or bh > MAX_BB_HEIGHT:
                    scale_w = MAX_BB_WIDTH / bw
                    scale_h = MAX_BB_HEIGHT / bh
                    scale = min(scale_w, scale_h)
                    new_bw = max(2, int(bw * scale))
                    new_bh = max(2, int(bh * scale))
                    x = max(0, min(mw - new_bw, cx - new_bw // 2))
                    y = max(0, min(mh - new_bh, cy - new_bh // 2))
                    bw, bh = new_bw, new_bh
                    xb = int(x * sx_m2l); yb = int(y * sy_m2l)
                    wb = max(2, int(bw * sx_m2l)); hb = max(2, int(bh * sy_m2l))
                    state.tracker = create_csrt_tracker(state.bMoovingTgt)
                    state.tracker.init(lores_frame, (xb, yb, wb, hb))
                    _last_tracker_id = id(state.tracker)
                    _tq_needs_init   = True      # new tracker → re-capture patch
                    _tq_monitor.reset()
                    print(f"[INFO] BB limited to {bw}x{bh} (max {MAX_BB_WIDTH}x{MAX_BB_HEIGHT})")

                state.bbox = (x, y, bw, bh)

                # Tracking quality: init on first frame, update on all subsequent ones
                if _tq_needs_init:
                    _tq_monitor.init(lores_frame, (xl, yl, wl, hl))
                    _tq_needs_init = False
                else:
                    _tq_monitor.update(lores_frame, (xl, yl, wl, hl))
                tq = _tq_monitor.score

                # Center offsets for attitude mapping (MAIN coords)
                dx = cx - mw // 2
                dy = cy - mh // 2
                norm_dx = dx / mw
                norm_dy = dy / mh

                # Simple FOV→angle mapping (heuristic; tune to your camera FOV)
                yaw_err   =  norm_dx * math.radians(60)   # ~60° HFOV
                pitch_err = -norm_dy * math.radians(45)   # ~45° VFOV

                # Count consecutive bad frames — break tracking if drift sustained
                if tq < TrackingQualityMonitor.SCORE_UNCERTAIN:
                    _tq_monitor.bad_frames += 1
                else:
                    _tq_monitor.bad_frames = 0   # good frame resets the counter

                if _tq_monitor.bad_frames >= TrackingQualityMonitor.BAD_FRAMES_LIMIT:
                    print(f"[TQ]   Drift detected ({_tq_monitor.bad_frames} bad frames, "
                          f"score={tq:.2f}) — tracking broken, re-select target")
                    state.tracking = False
                    state.tracker  = None
                    _tq_monitor.reset()
                    # Skip the rest of the draw block — show lost message instead
                    cv2.putText(frame, "Drift — re-select target", (10, 140),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    # Only drive control surfaces when quality is sufficient
                    if tq >= TrackingQualityMonitor.SCORE_UNCERTAIN:
                        mavlink_client.send_attitude_target(pitch_err, yaw_err)
                        _attitude_sent = True

                    # Box color encodes quality level
                    if tq >= TrackingQualityMonitor.SCORE_GOOD:
                        box_color = (0, 200, 0)      # green  — good
                    elif tq >= TrackingQualityMonitor.SCORE_UNCERTAIN:
                        box_color = (0, 140, 255)    # orange — uncertain, still sending
                    else:
                        box_color = (0, 0, 255)      # red    — unstable, counting down

                    cv2.rectangle(frame, (x, y), (x + bw, y + bh), box_color, 2)
                    cv2.line(frame, (cx - 10, cy), (cx + 10, cy), box_color, 1)
                    cv2.line(frame, (cx, cy - 10), (cx, cy + 10), box_color, 1)
            else:
                # tracker.update() itself failed — don't leave state.tracker alive to
                # keep trying on its own; it can re-lock onto the wrong thing. Same
                # hard stop as the TQ drift-kill: require an explicit re-select.
                state.tracking = False
                state.tracker  = None
                _tq_monitor.reset()
                cv2.putText(frame, "Tracking lost", (10, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        except Exception as e:
            print(f"[ERROR] Tracker update failed: {e}")
            state.tracking = False
    else:
        # Persistent (not one-frame) indicator that no tracker is active —
        # without this, the "Drift — re-select target" message only shows for
        # the single frame it fires on, then the screen goes blank with no
        # explanation once state.tracker is dropped to None.
        cv2.putText(frame, "No target — click to select", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # PX4's OFFBOARD mode auto-exits (COM_OF_LOSS_T) if it stops receiving
    # setpoints, even briefly — so once launched we must keep the stream
    # alive every frame, not just while a target is actively tracked.
    # Harmless on ArduPlane too: a zero-error target just holds/centers.
    if mavlink_client._launched and not _attitude_sent:
        mavlink_client.send_attitude_target(0.0, 0.0)

    # Write to file if in record mode
    if args.mode == 'record' and writer is not None:
        writer.write(frame)
        if args.duration and (time.time() - record_start_time >= args.duration):
            print("[INFO] Reached recording duration, exiting.")
            break

    # FPS estimate
    now = time.time()
    dt = max(1e-6, now - _prev_ts)
    _prev_ts = now
    inst_fps = 1.0 / dt
    _est_fps = _fps_alpha * _est_fps + (1.0 - _fps_alpha) * inst_fps if _est_fps > 0 else inst_fps

    # Overlay text
    stamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    overlay1 = f"{stamp}"
    overlay2 = f"MAIN {mw}x{mh} | TRACK {lw}x{lh} | {int(_est_fps)} FPS"
    cv2.putText(frame, overlay1, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(frame, overlay2, (8, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
    if args.mode == 'live':
        cv2.putText(frame, "Keys: z/x MAIN -, +   c/v TRACK -, +", (8, 84),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)

    # Playback UI sync
    if SHOW_LOCAL and args.mode == 'playback' and _trackbar_ready and playback_duration_ms > 0:
        try:
            _suppress_trackbar_cb = True
            pos_frac = max(0.0, min(1.0, playback_pos_ms / playback_duration_ms))
            cv2.setTrackbarPos('position', 'Tracker', int(pos_frac * 1000))
            with playback_ctrl_lock:
                cv2.setTrackbarPos('rate x0.01', 'Tracker', int(playback_rate * 100.0))
        finally:
            _suppress_trackbar_cb = False

    # Publish final frame
    with frame_ready:
        output_frame = frame.copy()
        frame_ready.notify_all()
    frame_buffer.put(output_frame)

    # Local window
    if SHOW_LOCAL:
        cv2.imshow("Tracker", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):
        state.tracking = False; state.bbox = None; state.tracker = None
        print("[INFO] Tracker reset from Pi")
    elif args.mode == 'live':
        if key == ord('x'):   _cycle_main(+1)
        elif key == ord('z'): _cycle_main(-1)
        elif key == ord('v'): _cycle_lores(+1)
        elif key == ord('c'): _cycle_lores(-1)
    elif args.mode == 'playback':
        if key == ord('f'):
            with playback_ctrl_lock: playback_rate = min(playback_rate * 2.0, 8.0)
            print(f"[PLAYBACK] Speed {playback_rate:.1f}×")
        elif key == ord('s'):
            with playback_ctrl_lock: playback_rate = max(playback_rate / 2.0, 0.25)
            print(f"[PLAYBACK] Speed {playback_rate:.2f}×")
        elif key == ord('1'):
            with playback_ctrl_lock: playback_rate = 1.0
            print("[PLAYBACK] Speed reset to 1×")
        elif key == ord('j'):
            if cap is not None:
                pos = cap.get(cv2.CAP_PROP_POS_MSEC)
                with playback_ctrl_lock: seek_to_msec = max(0, pos - 5000)
                print(f"[PLAYBACK] Seek −5 s")
        elif key == ord('l'):
            if cap is not None:
                pos = cap.get(cv2.CAP_PROP_POS_MSEC)
                with playback_ctrl_lock: seek_to_msec = pos + 5000
                print(f"[PLAYBACK] Seek +5 s")

# === Cleanup ===
mavlink_client.disarm()
cv2.destroyAllWindows()
_stop_reader.set()
if _reader_thread and _reader_thread.is_alive():
    _reader_thread.join(timeout=1.0)
if cap:
    cap.release()
if args.mode != 'playback' and picam2 is not None:
    try: picam2.stop()
    except Exception: pass
    try: picam2.close()
    except Exception: pass
if args.mode == 'record' and writer:
    writer.release()
