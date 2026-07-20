# Talon Tracker — full app documentation & new-device bootstrap guide

This file exists so a fresh Claude Code session on a **new Raspberry Pi** can read
one document and get the app running, without re-deriving anything from scratch.
It covers what the app does, every file in the repo, and — critically — everything
that is needed to run but is **NOT in git** (system packages, venvs, systemd, and
per-device config).

## What this app is

An onboard, camera-based target tracker for a fixed-wing drone (ArduPlane, not a
copter). It runs on a Raspberry Pi mounted in the airframe, does the following in
one process (`tracker-so.py`):

1. Captures video from the Pi camera (or a file, for playback/dev).
2. Runs a CSRT-based visual tracker (`GTSTracker`, wrapped in a Cython module for
   protecting the tuning parameters as IP) on a low-res copy of each frame to find
   the selected target.
3. Converts the target's pixel offset from frame-center into pitch/yaw error
   angles and streams `SET_ATTITUDE_TARGET` MAVLink commands to the flight
   controller (via a locally-running MAVProxy relay) so the plane's camera tracks
   the target in GUIDED mode.
4. Streams live video to a ground-control browser client over WebRTC (or JPEG/UDP,
   config-selectable) and exposes a small Flask REST API for target
   selection/reset/launch/arm from that same UI.
5. Continuously scores tracking quality (`TrackingQualityMonitor`) and kills
   tracking (rather than silently drifting) if confidence drops — see
   `attitude_changes.md` for the full history of tuning this control scheme.

A separate companion app, `gcs.py`, runs on the operator's laptop/Mac (not on the
Pi) as the ground-station client — it is launched via `Mahat GCS.command` or the
`.vscode/launch.json` "GCS" configs and talks to the Pi over the network.

## Repo file manifest

Everything below is what `git ls-files` / working tree contains as of this
writing. Files marked **(gitignored)** are NOT in git and will not exist on a
fresh clone — they're either build artifacts, logs, or machine-local venvs.

| File | Role |
|---|---|
| `tracker-so.py` | **Main entrypoint.** Everything above happens here: camera/file capture loop, tracking, MAVLink send, Flask+WebRTC startup, UDP command/telemetry channel to `gcs.py`. Run with `--mode live\|record\|playback`. |
| `gts_tracker.py` | Plain-Python `GTSTracker` (wraps `cv2.TrackerCSRT_create`, loads tuning params from `config.toml`-equivalent constants). Used automatically if the compiled `.so` hasn't been built (see Build step below). |
| `gts_tracker.pyx` | Cython source, byte-identical today to `gts_tracker.py`. Compiled by `setup.py` into `gts_tracker.<tag>.so` to hide the CSRT tuning parameters from anyone who gets the Pi's filesystem. **Optional** — the app works without building it, using `gts_tracker.py` instead. |
| `setup.py` | `python3 setup.py build_ext --inplace` → compiles `gts_tracker.pyx` into a native `.so` for this exact Python version + CPU arch (aarch64). Needs Cython + a C compiler. |
| `test_gts_tracker.py` | Standalone smoke test for `GTSTracker` (synthetic frame, fixed+moving modes). Run manually: `python3 test_gts_tracker.py`. |
| `flask_app.py` | Flask REST API (`/command`, `/select_point`, `/nudge`, `/set_target_mode`, `/cycle_main`, `/cycle_lores`, `/launch`, `/toggle_record`, `/status`) bound to `tracker-so.py`'s shared `state` object. Imported and mounted by `tracker-so.py`, served on port 5000. |
| `webrtc_server.py` | aiortc-based WebRTC server + embedded HTML/JS control page (no separate templates/static dir — the page is an inline string in this file), served on port 8080. Also defines `FrameBuffer`, the thread-safe hand-off between the main loop and the streamer. |
| `mavlink_client.py` | Launches MAVProxy as a subprocess (hardcoded path, see Gotchas below), connects to it via `pymavlink`, and exposes `connect()`, `send_attitude_target()`, `set_guided_mode()`, `arm()`, `disarm()`. `connect()` calls `set_guided_mode()` automatically, so GUIDED/OFFBOARD is entered right at startup — `arm()` (wired to Launch) only arms. Runs a background thread that keeps the latest FC `ATTITUDE` cached (needed because `send_attitude_target` composes roll/pitch onto current attitude — see `attitude_changes.md`). |
| `config.toml` | All tunables: display, CSRT params (fixed vs moving), network (interface, ports, GCS IP, WebRTC/JPEG mode), MAVLink (serial port/baud, UDP ports, extra telemetry outputs), camera resolution lists. **Must be edited per-device** — see below. |
| `mav.parm` | A saved dump of ArduPlane flight-controller parameters (for reference / reloading into the FC via Mission Planner or QGroundControl). Not read by any script. |
| `talon-tracker.service` | systemd unit. Runs `tracker-so.py --mode live` as user `mahat` from `/home/mahat/talon`, using the `webrtc_venv` interpreter. Restarts on failure. |
| `gcs.py` | Ground-station client — **runs on the operator's machine, not the Pi.** Receives video (UDP/WebRTC) and posts control commands to the Pi's Flask API. |
| `Mahat GCS.command` | Double-click launcher for `gcs.py` on macOS (uses `/opt/homebrew/bin/python3`). Not relevant to the Pi side. |
| `.vscode/launch.json` | Debug configs for both `tracker-so.py` (live/playback) and `gcs.py` (auto IP from config / manual IP prompt). |
| `README.md` | **Stale** — still describes the old repo name ("ASIO"), `tracker.py` (superseded), and `tracker.service` (renamed to `talon-tracker.service`). Don't trust it for current filenames; this file supersedes it. |
| `WEBRTC_FIX.md` | Postmortem on a WebRTC reconnect-loop bug (frame-watchdog re-registration + reconnect delay). Historical context for `webrtc_server.py`. |
| `attitude_changes.md` | Running log of `mavlink_client.send_attitude_target()` control-scheme changes and why. Append-only; read before changing attitude control logic. |
| `gts-tracker-howto.txt` | Internal note on the original motivation for Cython-compiling `gts_tracker` (protecting IP from customers) and the build steps `setup.py` automates. |
| `tracker.py` | **Dead code** — pre-Cython predecessor of `tracker-so.py`. Nothing imports or launches it. Candidate for deletion (kept for now, flagged in a prior review). |
| `.gitignore` | Ignores `*.so`, `*.c`, `build/`, `__pycache__/`, `*.egg-info/`, `*.tlog`, `*.tlog.raw`. |
| `mav.tlog`, `mav.tlog.raw` **(gitignored)** | MAVLink telemetry logs, regenerated at runtime (likely by MAVProxy/pymavlink logging). Not needed to bootstrap. |
| `__pycache__/` **(gitignored)** | Build artifact. |
| `gts_tracker.*.so` **(gitignored, doesn't exist until built)** | Compiled tracker extension — see Build step. |

## What is NOT in git (must be recreated on the new Pi)

This is the part that will actually block "clone and run" on a fresh device.

### 1. OS packages (apt)

The app relies on system-level camera/vision libs that are apt packages, not pip
packages, and are exposed into the venv via `--system-site-packages` (see below):

```bash
sudo apt update
sudo apt install -y \
  python3-opencv python3-picamera2 python3-libcamera \
  libcamera-apps rpicam-apps \
  python3-venv python3-dev build-essential git
```

Confirmed present on the current Pi (Debian 12 "bookworm", Python 3.11.2,
aarch64): `python3-opencv 4.6.0`, `python3-picamera2 0.3.31`,
`python3-libcamera 0.5.2`. `cv2` and `picamera2` are imported from
`/usr/lib/python3/dist-packages/`, **not** from pip — do not `pip install
opencv-python` in the venv, it isn't used and could shadow the apt one.

`camera_auto_detect=1` and `dtoverlay=vc4-kms-v3d` must be present in
`/boot/firmware/config.txt` for the camera to be detected (standard on current
Raspberry Pi OS images with a CSI camera attached, e.g. IMX708/Camera Module 3).

### 2. Two Python venvs (both at fixed paths — see Gotchas)

**`/home/mahat/mav_venv`** — isolated (`python3 -m venv mav_venv`, no
system-site-packages). Only runs MAVProxy as a subprocess; never imported
in-process.
```bash
python3 -m venv /home/mahat/mav_venv
/home/mahat/mav_venv/bin/pip install MAVProxy pymavlink pyserial
```

**`/home/mahat/webrtc_venv`** — runs `tracker-so.py` itself. Created with
`--system-site-packages` specifically so it can see the apt-installed `cv2` and
`picamera2`:
```bash
python3 -m venv --system-site-packages /home/mahat/webrtc_venv
/home/mahat/webrtc_venv/bin/pip install \
  Flask flask-cors \
  aiohttp aiortc av \
  pymavlink pyserial \
  numpy Cython
```
Note: the live `webrtc_venv` on this Pi also has dozens of unrelated packages
(Adafruit/SenseHAT/RealSenseID/PyQt5/type-stubs/etc.) — those leaked in via
`--system-site-packages` from Raspberry Pi OS's preinstalled system Python and
**are not talon dependencies**. Don't try to replicate the full `pip freeze`;
the list above is the actual requirement set derived from every `import` in
this repo.

If `aiortc`/`av` fail to install from wheels on the new Pi's Python/OS
combination, they need FFmpeg dev headers first:
```bash
sudo apt install -y libavformat-dev libavdevice-dev libavfilter-dev \
  libopus-dev libvpx-dev pkg-config
```

### 3. Build the compiled tracker (optional)

```bash
cd /home/mahat/talon
/home/mahat/webrtc_venv/bin/python3 setup.py build_ext --inplace
```
This is **optional** — `gts_tracker.py` (plain Python, same logic) sits next to
the `.pyx` and Python will use it automatically if no compiled module named
`gts_tracker` exists. The compile step only exists to hide the CSRT tuning
constants; skip it for a working dev/bring-up and do it later if IP protection
matters for this specific device's deployment.

### 4. User permissions

The user running the service (`mahat` on the current Pi) must be in the
`dialout` group to access `/dev/ttyACM0` (the flight controller serial port):
```bash
sudo usermod -aG dialout $USER   # log out/in (or reboot) to take effect
```

### 5. Config — edit `config.toml` for the new device

These fields are device-specific and almost certainly wrong for a new Pi/plane:

- `[network] interface` — `"wlan0"` vs `"wlan1"` (current Pi uses a USB wifi
  dongle on `wlan1` connected to an SSID named "Talon").
- `[network.wlan0]` / `[network.wlan1]` `bind_ip` — must match the new Pi's
  actual IP on the chosen interface.
- `[network.wlan1] gcs_ip` — only used for the legacy JPEG/UDP video mode; the
  default `video_mode = "webrtc"` learns the GCS IP dynamically from its
  heartbeat, so this can usually be left alone.
- `[mavlink] pixhawk_port` — check with `ls /dev/ttyACM* /dev/ttyUSB*` on the
  new device (auto-falls-back to whatever it finds if the configured one is
  missing, see `mavlink_client._find_fc_port`).
- `[mavlink] extra_outputs` — list of GCS/QGC IPs that should get a dedicated
  MAVProxy telemetry feed; update for the new network.
- `[camera] main_sizes` / `lores_sizes` — tuned for the IMX708 sensor path;
  fine to leave as-is unless the new Pi has a different camera module.

### 6. systemd service

```bash
sudo cp talon-tracker.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable talon-tracker.service
sudo systemctl start talon-tracker.service
journalctl -u talon-tracker.service -f
```
If the new device uses a different username or repo path, edit `User=`,
`WorkingDirectory=`, and the venv paths in `Environment=`/`ExecStart=` in
`talon-tracker.service` before installing it.

### 7. Recording mode needs a directory

`--mode record` writes to `RecordingsMahat/recording_<timestamp>.avi` relative
to the working directory; that directory does not exist by default and isn't
created automatically — `mkdir RecordingsMahat` before using record mode.

## Gotchas / things that will silently break on a copy-paste bring-up

- **Hardcoded venv path**: `mavlink_client.py` launches MAVProxy via the literal
  string `/home/mahat/mav_venv/bin/mavproxy.py` (not relative, not read from
  config). If the new Pi uses a different username or venv location, this line
  must be edited or the MAVProxy launch will fail.
- **Two separate MAVLink stacks**: `pymavlink` must be installed in
  **both** venvs — once in `mav_venv` (for MAVProxy itself) and once in
  `webrtc_venv` (because `tracker-so.py`/`mavlink_client.py` talks MAVLink
  protocol directly over the UDP relay MAVProxy provides). Installing it in
  only one will fail confusingly in the other process.
- **`cv2`/`picamera2` come from apt, not pip** — if someone `pip install`s
  `opencv-python` into `webrtc_venv` it can shadow the working apt version with
  one that lacks camera/CSRT support expected here.
- **`talon-tracker.service` was recently renamed** from `tracker.service`
  (see git status `R tracker.service -> talon-tracker.service`) — if the old
  unit file is still installed under `/etc/systemd/system/tracker.service` on
  a device being reused, disable/remove it to avoid two competing instances.
- **Vehicle is ArduPlane (fixed-wing), not a copter.** `SET_ATTITUDE_TARGET`
  yaw is sent as a **raw rudder command**, never composed onto current heading
  — composing it (as you would on a copter) saturates the rudder. Roll/pitch
  *are* composed onto current attitude. Full reasoning in
  `mavlink_client.send_attitude_target()`'s docstring and `attitude_changes.md`.
- **Tracking-quality states are not symmetric**: "Tracking lost" (CSRT
  `update()` itself failed) can self-recover once re-selected; "No target..."
  (no tracker object at all, `state.tracker is None`) never self-recovers and
  always needs an explicit re-select. Confirmed by live test, not a bug.

## Verifying a new bring-up works

1. `source /home/mahat/webrtc_venv/bin/activate; cd /home/mahat/talon`
2. `python3 test_gts_tracker.py` → should print `[OK] mode=fixed` / `[OK]
   mode=moving` and `All tests passed.` (sanity-checks CSRT wrapping without
   needing a camera or FC).
3. `python3 tracker-so.py --mode playback --loop` (needs `show_local = true` in
   `config.toml` and a display, or run headless and hit the Flask API) to
   validate tracking + Flask + WebRTC without needing the plane airborne.
4. `python3 tracker-so.py --mode live` on the actual hardware — check console
   for `[LIVE] Camera started`, `[MAVLink] Connected via MAVProxy...`,
   `[Flask] http://<ip>:5000`, `[WebRTC] http://<ip>:8080`.
5. Enable the systemd service for unattended boot-time start (Section 6 above).
