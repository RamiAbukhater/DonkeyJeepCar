"""GPS/GNSS waypoint navigation for JeepBot (PID path follower).

Conventions used everywhere in this package:
  * Local frame: x = east (m), y = north (m), origin at the first waypoint.
  * Angles are compass bearings in radians: 0 = north, clockwise positive.
  * Lateral quantities are positive to the RIGHT of the path direction.
  * Steering output is normalised to [-1, +1], +1 = full right
    (same convention as the joystick stick in jeepbot_drive.py).
"""
from .navigator import GpsNavigator, NavCommand, NavConfig

__all__ = ["GpsNavigator", "NavCommand", "NavConfig"]
