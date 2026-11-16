"""GPS waypoint navigator: path tracker + lateral PID + heading PD.

Control law (all "+" = right):

    lateral_error = lateral_offset - cross_track          [m]
    steer = clamp(PID_lat(lateral_error), +/-lat_limit)
          + PD_head(path_bearing - heading)               [normalised -1..+1]

The lateral PID decides how hard to turn *towards* the target line; the heading
term damps it and lines the car up with the path. Clamping the lateral part
caps the approach angle, so a large cross-track error produces a steady
intercept instead of a circle.

``lateral_offset`` is the hook for the object-avoidance team: it moves the
line the PID tracks sideways without touching the controller itself. Nothing
in this package sets it yet; it stays 0.0 unless ``set_lateral_offset`` is
called.
"""
from __future__ import annotations

import csv
import math
import os
import threading
import time
from dataclasses import dataclass, fields

from .geo import LocalFrame, bearing, wrap_pi
from .gps_reader import GpsFix
from .path import PathTracker, TrackState, load_waypoints_csv
from .pid import PID, clamp


@dataclass
class NavConfig:
    """Tunable parameters. Each maps to ``GPS_NAV_<NAME>`` in myconfig.py."""

    # --- Lateral (cross-track) PID: output is normalised steering per metre ---
    kp_lat: float = 0.35
    ki_lat: float = 0.0
    kd_lat: float = 0.0
    i_limit_lat: float = 0.2      # max steering the integral may hold
    d_tau_lat: float = 0.3        # s, derivative low-pass
    lat_limit: float = 0.5        # max steering from the lateral PID alone

    # --- Heading PD: output is normalised steering per radian ---
    kp_head: float = 1.2
    kd_head: float = 0.0
    d_tau_head: float = 0.2

    lookahead_m: float = 1.5      # where along the path the heading reference is taken

    # --- Speed (fraction of cruise duty, 0..1) ---
    min_throttle: float = 0.6     # fraction of cruise kept at full steering lock
    creep_throttle: float = 0.6   # while acquiring heading
    slow_distance_m: float = 3.0  # start slowing this far from the goal
    goal_tolerance_m: float = 1.0

    # --- Heading estimation from GPS motion ---
    heading_min_speed: float = 0.4   # m/s, below this GPS course is not trusted
    heading_baseline_m: float = 0.6  # displacement needed for a position-derived heading
    heading_alpha: float = 0.6       # 1.0 = no smoothing

    # --- Safety ---
    fix_timeout_s: float = 1.0    # no new fix for this long -> stop
    min_quality: int = 1          # 1 = any fix, 2 = DGPS/SBAS, 5 = RTK float, 4 = RTK fixed only
    max_hdop: float = 2.5
    max_cte_m: float = 4.0        # further than this from the target line -> stop (latched)

    # --- Avoidance hook ---
    max_offset_m: float = 2.0     # set_lateral_offset is clamped to +/- this

    @classmethod
    def from_module(cls, config) -> "NavConfig":
        values = {}
        for f in fields(cls):
            name = "GPS_NAV_" + f.name.upper()
            if hasattr(config, name):
                values[f.name] = type(f.default)(getattr(config, name))
        return cls(**values)


@dataclass(frozen=True)
class NavCommand:
    steering: float   # -1..+1, + = right
    throttle: float   # 0..1 fraction of cruise duty
    state: str
    ok: bool          # True only while actively producing a driving command


STOP_STATES = ("IDLE", "NO_PATH", "NO_FIX", "STALE", "LOW_QUALITY", "OFF_PATH", "DONE")


def _quality_ok(quality: int, minimum: int) -> bool:
    # GGA quality codes are not ordered by accuracy: 4 (RTK fixed) beats 5 (float).
    rank = {0: 0, 6: 0, 1: 1, 2: 2, 5: 3, 4: 4}
    return rank.get(quality, 0) >= rank.get(minimum, 1)


class HeadingEstimator:
    """Compass heading from GPS motion (there is no IMU/compass on the jeep).

    Uses the receiver's course-over-ground when moving fast enough, otherwise
    the bearing between positions at least ``baseline`` metres apart. Only
    valid while driving FORWARD; invalid until the car has moved.
    """

    def __init__(self, min_speed: float, baseline: float, alpha: float):
        self.min_speed = min_speed
        self.baseline = baseline
        self.alpha = clamp(alpha, 0.05, 1.0)
        self.reset()

    def reset(self) -> None:
        self.heading = None
        self._anchor = None

    def _blend(self, measured: float) -> None:
        if self.heading is None:
            self.heading = measured
        else:
            self.heading = wrap_pi(
                self.heading + self.alpha * wrap_pi(measured - self.heading)
            )

    def update(self, x: float, y: float, fix: GpsFix) -> float | None:
        if fix.speed_mps is not None:
            if fix.speed_mps < self.min_speed:
                # Stationary: position noise would look like motion. Hold.
                self._anchor = (x, y)
                return self.heading
            if fix.course_rad is not None:
                self._anchor = (x, y)
                self._blend(fix.course_rad)
                return self.heading
        if self._anchor is None:
            self._anchor = (x, y)
            return self.heading
        dx, dy = x - self._anchor[0], y - self._anchor[1]
        if math.hypot(dx, dy) >= self.baseline:
            self._blend(bearing(dx, dy))
            self._anchor = (x, y)
        return self.heading


LOG_COLUMNS = [
    "t", "state", "active", "lat", "lon", "quality", "num_sats", "hdop",
    "x", "y", "speed", "heading_deg", "segment", "cte", "offset",
    "lat_error", "head_error_deg", "p_lat", "i_lat", "d_lat", "u_head",
    "steering", "throttle", "dist_remaining",
]


class GpsNavigator:
    """Turns GPS fixes into steering/throttle commands along a waypoint path.

    Typical use from the drive loop (called every iteration):

        cmd = navigator.update(active=in_gps_mode_and_deadman_held)

    While ``active`` is False the controller is held reset (and GPS is still
    logged, which is how paths get recorded during manual driving).
    """

    def __init__(self, cfg: NavConfig | None = None, reader=None, log_dir: str | None = None):
        self.cfg = cfg or NavConfig()
        self.reader = reader
        self._lock = threading.Lock()
        self._offset = 0.0
        self._frame = None
        self._tracker = None
        self._path_name = ""
        self._pid_lat = PID(0.0)
        self._pid_head = PID(0.0)
        self._heading = HeadingEstimator(
            self.cfg.heading_min_speed, self.cfg.heading_baseline_m, self.cfg.heading_alpha
        )
        self._apply_gains()
        self._was_active = False
        self._engaged = False
        self._latched = None          # latched stop state (OFF_PATH / DONE)
        self._last_seq = None
        self._last_fix_t = None
        self._saturated = 0
        self._cmd = NavCommand(0.0, 0.0, "IDLE", False)
        self._status = {"state": "IDLE"}
        self._log = None
        self._log_writer = None
        if log_dir:
            self._open_log(log_dir)

    # ------------------------------------------------------------------ setup

    @classmethod
    def from_config(cls, config) -> "GpsNavigator":
        """Build navigator + serial reader from the myconfig module."""
        from .gps_reader import GpsReader

        reader = GpsReader(
            getattr(config, "GPS_PORT", "/dev/ttyUSB0"),
            getattr(config, "GPS_BAUDRATE", 115200),
        ).start()
        nav = cls(
            NavConfig.from_module(config),
            reader=reader,
            log_dir=getattr(config, "GPS_NAV_LOG_DIR", None),
        )
        path_file = getattr(config, "GPS_NAV_PATH_FILE", None)
        if path_file:
            try:
                nav.load_path(path_file)
            except (OSError, ValueError) as exc:
                print(f"[gps-nav] WARNING: could not load path '{path_file}': {exc}")
        return nav

    def _apply_gains(self) -> None:
        c = self.cfg
        p = self._pid_lat
        p.kp, p.ki, p.kd = c.kp_lat, c.ki_lat, c.kd_lat
        p.i_limit, p.d_tau, p.out_limit = abs(c.i_limit_lat), c.d_tau_lat, abs(c.lat_limit)
        h = self._pid_head
        h.kp, h.ki, h.kd = c.kp_head, 0.0, c.kd_head
        h.d_tau, h.out_limit = c.d_tau_head, 1.0

    def set_gains(self, **gains) -> None:
        """Change NavConfig fields at runtime, e.g. set_gains(kp_lat=0.4)."""
        with self._lock:
            for name, value in gains.items():
                if not hasattr(self.cfg, name):
                    raise AttributeError(f"unknown nav parameter: {name}")
                setattr(self.cfg, name, type(getattr(self.cfg, name))(value))
            self._apply_gains()

    def load_path(self, csv_path: str) -> None:
        self.set_path(load_waypoints_csv(csv_path), name=os.path.basename(csv_path))

    def set_path(self, waypoints_latlon: list[tuple[float, float]], name: str = "") -> None:
        frame = LocalFrame(*waypoints_latlon[0])
        tracker = PathTracker(
            [frame.to_xy(lat, lon) for lat, lon in waypoints_latlon],
            goal_tolerance=self.cfg.goal_tolerance_m,
        )
        with self._lock:
            self._frame, self._tracker, self._path_name = frame, tracker, name
            self._engaged = False
            self._latched = None
        print(
            f"[gps-nav] Path '{name}': {len(waypoints_latlon)} waypoints, "
            f"{tracker.total_length:.1f} m"
        )

    # -------------------------------------------------------- avoidance hook

    def set_lateral_offset(self, offset_m: float) -> None:
        """Shift the line the PID tracks sideways. + = right of the path, metres.

        Intended for the object-avoidance team: publish the desired sideways
        shift here and the PID steers onto the shifted line; set it back to 0.0
        to return to the recorded path. Thread-safe; clamped to
        +/- ``max_offset_m``. A non-finite value is treated as 0.
        """
        if not math.isfinite(offset_m):
            offset_m = 0.0
        with self._lock:
            self._offset = clamp(offset_m, -self.cfg.max_offset_m, self.cfg.max_offset_m)

    def get_lateral_offset(self) -> float:
        with self._lock:
            return self._offset

    # ---------------------------------------------------------------- control

    def update(self, active: bool = True) -> NavCommand:
        """Real-time entry point: read the latest fix and step the controller."""
        fix = self.reader.latest() if self.reader is not None else None
        return self.step(fix, time.monotonic(), active)

    def step(self, fix: GpsFix | None, now: float, active: bool = True) -> NavCommand:
        """Deterministic core (also used by the simulator and tests)."""
        with self._lock:
            cmd = self._step_locked(fix, now, active)
            self._cmd = cmd
            self._status["state"] = cmd.state
            return cmd

    def _stop(self, state: str) -> NavCommand:
        return NavCommand(0.0, 0.0, state, False)

    def _reset_controllers(self) -> None:
        self._pid_lat.reset()
        self._pid_head.reset()
        self._saturated = 0
        self._last_fix_t = None

    def _step_locked(self, fix, now, active) -> NavCommand:
        cfg = self.cfg

        if active and not self._was_active:
            # Fresh engagement: forget history, re-acquire path position.
            self._reset_controllers()
            self._engaged = False
            self._latched = None
        if not active and self._was_active:
            self._reset_controllers()
        self._was_active = active

        new_fix = fix is not None and fix.seq != self._last_seq
        if new_fix:
            self._last_seq = fix.seq

        # ---- validity gates (always evaluated so the UI shows GPS health) ----
        gate = None
        if fix is None:
            gate = "NO_FIX"
        elif now - fix.t > cfg.fix_timeout_s:
            gate = "STALE"
        elif not _quality_ok(fix.quality, cfg.min_quality) or fix.hdop > cfg.max_hdop:
            gate = "LOW_QUALITY"
        elif self._tracker is None:
            gate = "NO_PATH"

        if gate is not None:
            if gate in ("NO_FIX", "STALE", "LOW_QUALITY"):
                self._heading.reset()
            self._reset_controllers()
            if new_fix:
                self._record(fix, now, gate, active, None, None, None, None, 0.0, 0.0)
            return self._stop(gate if active else "IDLE")

        if not new_fix:
            # Between GPS epochs: hold the last command (zero-order hold).
            return self._cmd if active else self._stop("IDLE")

        # ---- new measurement ----
        x, y = self._frame.to_xy(fix.lat, fix.lon)
        heading = self._heading.update(x, y, fix)

        if not active:
            self._record(fix, now, "IDLE", False, x, y, heading, None, 0.0, 0.0)
            return self._stop("IDLE")

        if not self._engaged:
            self._tracker.reset(x, y)
            self._engaged = True
        track = self._tracker.update(x, y, cfg.lookahead_m)

        if self._latched is None:
            if track.done:
                self._latched = "DONE"
            elif abs(self._offset - track.cte_right) > cfg.max_cte_m:
                self._latched = "OFF_PATH"
        if self._latched is not None:
            self._record(fix, now, self._latched, True, x, y, heading, track, 0.0, 0.0)
            return self._stop(self._latched)

        dt = 0.1 if self._last_fix_t is None else clamp(fix.t - self._last_fix_t, 0.02, 1.0)
        self._last_fix_t = fix.t

        if heading is None:
            # Direction of travel unknown: roll straight until GPS reveals it.
            self._record(fix, now, "ACQUIRE_HEADING", True, x, y, None, track, 0.0, cfg.creep_throttle)
            return NavCommand(0.0, cfg.creep_throttle, "ACQUIRE_HEADING", True)

        lat_error = self._offset - track.cte_right
        head_error = wrap_pi(track.path_bearing - heading)

        # Anti-windup: do not integrate further into a saturated actuator.
        freeze = self._saturated != 0 and (lat_error > 0) == (self._saturated > 0)
        u_lat = self._pid_lat.update(lat_error, dt, measurement=track.cte_right, freeze_integral=freeze)
        u_head = self._pid_head.update(head_error, dt)
        raw = u_lat + u_head
        steering = clamp(raw, -1.0, 1.0)
        self._saturated = 0 if raw == steering else (1 if raw > 0 else -1)

        throttle = 1.0 - (1.0 - cfg.min_throttle) * abs(steering)
        if cfg.slow_distance_m > 0.0:
            throttle *= clamp(track.dist_remaining / cfg.slow_distance_m, cfg.min_throttle, 1.0)

        self._record(
            fix, now, "TRACKING", True, x, y, heading, track, steering, throttle,
            lat_error=lat_error, head_error=head_error, u_head=u_head,
        )
        return NavCommand(steering, throttle, "TRACKING", True)

    # -------------------------------------------------------------- telemetry

    def status(self) -> dict:
        """Latest telemetry snapshot (for the web UI / debugging)."""
        with self._lock:
            return dict(self._status)

    def _open_log(self, log_dir: str) -> None:
        try:
            os.makedirs(log_dir, exist_ok=True)
            name = time.strftime("nav_%Y%m%d_%H%M%S.csv")
            self._log = open(os.path.join(log_dir, name), "w", newline="")
            self._log_writer = csv.writer(self._log)
            self._log_writer.writerow(LOG_COLUMNS)
            print(f"[gps-nav] Logging to {self._log.name}")
        except OSError as exc:
            print(f"[gps-nav] WARNING: logging disabled: {exc}")
            self._log = None

    def close(self) -> None:
        if self._log is not None:
            self._log.close()
            self._log = None

    def _record(
        self, fix, now, state, active, x, y, heading, track: TrackState | None,
        steering, throttle, lat_error=None, head_error=None, u_head=None,
    ) -> None:
        def r(value, digits=3):
            return None if value is None else round(value, digits)

        row = {
            "t": r(now),
            "state": state,
            "active": int(active),
            "lat": r(fix.lat, 8),
            "lon": r(fix.lon, 8),
            "quality": fix.quality,
            "num_sats": fix.num_sats,
            "hdop": fix.hdop,
            "x": r(x),
            "y": r(y),
            "speed": r(fix.speed_mps),
            "heading_deg": None if heading is None else round(math.degrees(heading) % 360.0, 1),
            "segment": None if track is None else track.segment,
            "cte": None if track is None else r(track.cte_right),
            "offset": r(self._offset),
            "lat_error": r(lat_error),
            "head_error_deg": None if head_error is None else round(math.degrees(head_error), 1),
            "p_lat": r(self._pid_lat.p_term, 4) if lat_error is not None else None,
            "i_lat": r(self._pid_lat.i_term, 4) if lat_error is not None else None,
            "d_lat": r(self._pid_lat.d_term, 4) if lat_error is not None else None,
            "u_head": r(u_head, 4),
            "steering": r(steering),
            "throttle": r(throttle),
            "dist_remaining": None if track is None else r(track.dist_remaining, 2),
        }
        row["path"] = self._path_name
        self._status = row
        if self._log_writer is not None:
            try:
                self._log_writer.writerow(["" if row[c] is None else row[c] for c in LOG_COLUMNS])
                self._log.flush()
            except (OSError, ValueError):
                pass
