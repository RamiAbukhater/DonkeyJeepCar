"""Waypoint path loading and tracking (cross-track / heading reference)."""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass

from .geo import bearing


def load_waypoints_csv(path: str) -> list[tuple[float, float]]:
    """Load (lat, lon) waypoints in decimal degrees.

    Accepts a header row naming ``lat``/``lon`` (or ``latitude``/``longitude``)
    columns in any position; without a header the first two columns are used.
    Lines starting with '#' are ignored.
    """
    rows = []
    with open(path, newline="") as fh:
        for row in csv.reader(line for line in fh if not line.lstrip().startswith("#")):
            if row and any(cell.strip() for cell in row):
                rows.append([cell.strip() for cell in row])
    if not rows:
        raise ValueError(f"{path}: no waypoints")

    lat_i, lon_i = 0, 1
    header = [cell.lower() for cell in rows[0]]
    for names, which in ((("lat", "latitude"), "lat"), (("lon", "longitude", "lng"), "lon")):
        for name in names:
            if name in header:
                if which == "lat":
                    lat_i = header.index(name)
                else:
                    lon_i = header.index(name)
    try:
        float(rows[0][lat_i])
    except ValueError:
        rows = rows[1:]  # header row

    waypoints = [(float(r[lat_i]), float(r[lon_i])) for r in rows]
    if len(waypoints) < 2:
        raise ValueError(f"{path}: need at least 2 waypoints, got {len(waypoints)}")
    return waypoints


@dataclass(frozen=True)
class TrackState:
    cte_right: float       # m, car position relative to path, + = right of path
    path_bearing: float    # rad, path direction at the lookahead point
    dist_remaining: float  # m along the path to the final waypoint
    segment: int           # index of the segment currently being followed
    done: bool


class PathTracker:
    """Tracks progress along a polyline of (east, north) points."""

    def __init__(self, points_xy: list[tuple[float, float]], goal_tolerance: float = 1.0):
        # Drop zero-length segments (duplicate waypoints).
        pts = [points_xy[0]]
        for p in points_xy[1:]:
            if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-3:
                pts.append(p)
        if len(pts) < 2:
            raise ValueError("path needs at least 2 distinct waypoints")
        self.points = pts
        self.goal_tolerance = goal_tolerance
        self._len = [
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:])
        ]
        self._bearing = [
            bearing(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:])
        ]
        self.total_length = sum(self._len)
        self._idx = 0

    @property
    def num_segments(self) -> int:
        return len(self._len)

    def _project(self, i: int, x: float, y: float) -> tuple[float, float]:
        """(fraction along segment i, signed distance right of it)."""
        ax, ay = self.points[i]
        bx, by = self.points[i + 1]
        ux, uy = (bx - ax) / self._len[i], (by - ay) / self._len[i]
        rx, ry = x - ax, y - ay
        along = rx * ux + ry * uy
        left = ux * ry - uy * rx
        return along / self._len[i], -left

    def _distance(self, i: int, x: float, y: float) -> float:
        """Distance from (x, y) to the nearest point ON segment i."""
        t, _ = self._project(i, x, y)
        t = min(1.0, max(0.0, t))
        ax, ay = self.points[i]
        bx, by = self.points[i + 1]
        return math.hypot(x - (ax + t * (bx - ax)), y - (ay + t * (by - ay)))

    def reset(self, x: float, y: float) -> None:
        """Start from whichever segment is nearest (lets you engage mid-path)."""
        self._idx = min(range(self.num_segments), key=lambda i: self._distance(i, x, y))

    def update(self, x: float, y: float, lookahead: float = 0.0) -> TrackState:
        # Progress only moves forward: advance once we pass a segment's end, or
        # once the next segment is the closer one (cutting inside a corner).
        t, right = self._project(self._idx, x, y)
        while self._idx < self.num_segments - 1 and (
            t >= 1.0
            or self._distance(self._idx + 1, x, y) < self._distance(self._idx, x, y)
        ):
            self._idx += 1
            t, right = self._project(self._idx, x, y)

        t_clamped = min(1.0, max(0.0, t))
        remaining = (1.0 - t_clamped) * self._len[self._idx] + sum(
            self._len[self._idx + 1:]
        )

        # Path direction a little ahead of the car, so turns are anticipated.
        look_i = self._idx
        ahead = lookahead - (1.0 - t_clamped) * self._len[self._idx]
        while ahead > 0.0 and look_i < self.num_segments - 1:
            look_i += 1
            ahead -= self._len[look_i]

        done = (self._idx == self.num_segments - 1 and t >= 1.0) or (
            remaining <= self.goal_tolerance
        )
        return TrackState(
            cte_right=right,
            path_bearing=self._bearing[look_i],
            dist_remaining=remaining,
            segment=self._idx,
            done=done,
        )
