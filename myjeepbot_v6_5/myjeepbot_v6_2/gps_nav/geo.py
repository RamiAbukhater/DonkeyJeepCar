"""Small geodesy helpers: lat/lon <-> local metres, compass bearings."""
from __future__ import annotations

import math

EARTH_RADIUS_M = 6378137.0


def wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def bearing(d_east: float, d_north: float) -> float:
    """Compass bearing (rad, 0 = north, clockwise +) of a displacement."""
    return math.atan2(d_east, d_north)


class LocalFrame:
    """Flat-earth east/north frame around an origin.

    Equirectangular projection; error is far below GPS noise for paths
    shorter than a few kilometres.
    """

    def __init__(self, lat0_deg: float, lon0_deg: float):
        self.lat0 = lat0_deg
        self.lon0 = lon0_deg
        self._m_per_deg_lat = math.radians(1.0) * EARTH_RADIUS_M
        self._m_per_deg_lon = self._m_per_deg_lat * math.cos(math.radians(lat0_deg))

    def to_xy(self, lat_deg: float, lon_deg: float) -> tuple[float, float]:
        east = (lon_deg - self.lon0) * self._m_per_deg_lon
        north = (lat_deg - self.lat0) * self._m_per_deg_lat
        return east, north

    def to_latlon(self, east: float, north: float) -> tuple[float, float]:
        lat = self.lat0 + north / self._m_per_deg_lat
        lon = self.lon0 + east / self._m_per_deg_lon
        return lat, lon
