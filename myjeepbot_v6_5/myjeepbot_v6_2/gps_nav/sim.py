"""Offline simulator: kinematic bicycle + simulated GPS, to sanity-check gains.

The vehicle numbers in ``SimVehicle`` are GUESSES for the jeep. Measure the
real ones (see GPS_NAV.md) before trusting simulated gains on hardware.
"""
from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass, field

from .geo import LocalFrame
from .gps_reader import GpsFix
from .navigator import GpsNavigator
from .path import PathTracker


@dataclass
class SimVehicle:
    wheelbase_m: float = 0.9
    max_wheel_angle_deg: float = 25.0   # road-wheel angle at steering = +/-1
    steer_rate_per_s: float = 2.0       # normalised steering units per second
    cruise_speed_mps: float = 1.5       # speed at throttle = 1.0
    speed_tau_s: float = 0.6


@dataclass
class SimGps:
    rate_hz: float = 10.0
    noise_m: float = 0.03               # 1-sigma per axis (0.03 ~ RTK, 0.5+ ~ plain GPS)
    latency_s: float = 0.15
    quality: int = 4
    hdop: float = 0.8


@dataclass
class SimResult:
    finished: bool
    time_s: float
    final_state: str
    t: list = field(default_factory=list)
    x: list = field(default_factory=list)
    y: list = field(default_factory=list)
    cte: list = field(default_factory=list)       # true cross-track, + right
    steering: list = field(default_factory=list)

    def cte_stats(self, after_s: float = 0.0) -> tuple[float, float, float]:
        """(mean, rms about zero, max abs) of true cross-track after ``after_s``."""
        vals = [c for t, c in zip(self.t, self.cte) if t >= after_s]
        if not vals:
            return 0.0, 0.0, 0.0
        mean = sum(vals) / len(vals)
        rms = math.sqrt(sum(v * v for v in vals) / len(vals))
        return mean, rms, max(abs(v) for v in vals)


def simulate(
    nav: GpsNavigator,
    waypoints_latlon: list[tuple[float, float]],
    start_xy: tuple[float, float] = (0.0, 0.0),
    start_heading_deg: float | None = None,
    duration_s: float = 120.0,
    vehicle: SimVehicle | None = None,
    gps: SimGps | None = None,
    seed: int = 0,
    offset_schedule=None,       # optional f(t) -> lateral offset in metres
    gps_dropout=None,           # optional (t_start, t_end) with no fixes
) -> SimResult:
    vehicle = vehicle or SimVehicle()
    gps = gps or SimGps()
    rng = random.Random(seed)

    nav.set_path(waypoints_latlon, name="sim")
    frame = LocalFrame(*waypoints_latlon[0])
    pts = [frame.to_xy(lat, lon) for lat, lon in waypoints_latlon]
    truth = PathTracker(pts, goal_tolerance=0.0)
    truth.reset(*start_xy)

    x, y = start_xy
    if start_heading_deg is None:
        heading = math.atan2(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
    else:
        heading = math.radians(start_heading_deg)
    speed = 0.0
    steer_actual = 0.0
    max_delta = math.radians(vehicle.max_wheel_angle_deg)

    dt = 0.01
    history = deque()   # (t, x, y, heading, speed) for GPS latency
    next_gps_t = 0.0
    seq = 0
    fix = None
    result = SimResult(False, 0.0, "IDLE")

    steps = int(duration_s / dt)
    for k in range(steps):
        t = k * dt
        history.append((t, x, y, heading, speed))
        while len(history) > 1 and history[1][0] <= t - gps.latency_s:
            history.popleft()

        if t >= next_gps_t:
            next_gps_t += 1.0 / gps.rate_hz
            in_dropout = gps_dropout is not None and gps_dropout[0] <= t < gps_dropout[1]
            if not in_dropout:
                _, gx, gy, gh, gv = history[0]
                lat, lon = frame.to_latlon(
                    gx + rng.gauss(0.0, gps.noise_m), gy + rng.gauss(0.0, gps.noise_m)
                )
                seq += 1
                fix = GpsFix(
                    t=t, seq=seq, lat=lat, lon=lon, quality=gps.quality,
                    num_sats=14, hdop=gps.hdop, speed_mps=gv,
                    course_rad=gh + rng.gauss(0.0, math.radians(2.0)) if gv > 0.2 else None,
                )

        if offset_schedule is not None:
            nav.set_lateral_offset(offset_schedule(t))
        cmd = nav.step(fix, t, active=True)

        # Actuators
        max_step = vehicle.steer_rate_per_s * dt
        steer_actual += max(-max_step, min(max_step, cmd.steering - steer_actual))
        target_speed = cmd.throttle * vehicle.cruise_speed_mps
        speed += (target_speed - speed) * dt / vehicle.speed_tau_s

        # Bicycle model (compass heading: clockwise +, so right steer increases it)
        heading += speed / vehicle.wheelbase_m * math.tan(steer_actual * max_delta) * dt
        x += speed * math.sin(heading) * dt
        y += speed * math.cos(heading) * dt

        if k % 10 == 0:
            result.t.append(t)
            result.x.append(x)
            result.y.append(y)
            result.cte.append(truth.update(x, y).cte_right)
            result.steering.append(steer_actual)

        result.time_s = t
        result.final_state = cmd.state
        if cmd.state == "DONE":
            result.finished = True
            break
        if cmd.state == "OFF_PATH":
            break
    return result


def demo_path(kind: str = "l") -> list[tuple[float, float]]:
    """Synthetic test paths (lat, lon) near UCSD, 1 m waypoint spacing."""
    frame = LocalFrame(32.8811, -117.2340)
    if kind == "straight":
        xy = [(0.0, float(i)) for i in range(0, 41)]
    elif kind == "l":
        xy = [(0.0, float(i)) for i in range(0, 21)]
        xy += [(float(i), 20.0) for i in range(1, 21)]
    elif kind == "s":
        xy = [(4.0 * math.sin(i / 6.0), float(i)) for i in range(0, 61)]
    else:
        raise ValueError(f"unknown demo path: {kind}")
    return [frame.to_latlon(e, n) for e, n in xy]
