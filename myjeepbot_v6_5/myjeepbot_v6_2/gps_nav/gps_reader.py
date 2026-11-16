"""NMEA GPS/GNSS reader (background thread, pyserial only)."""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

KNOTS_TO_MPS = 0.514444

# GGA fix-quality field
QUALITY_NAMES = {
    0: "no fix",
    1: "GPS",
    2: "DGPS",
    4: "RTK fixed",
    5: "RTK float",
    6: "dead reckoning",
}


@dataclass(frozen=True)
class GpsFix:
    """One position epoch. ``t`` is time.monotonic() at receipt."""

    t: float
    seq: int
    lat: float
    lon: float
    quality: int = 0
    num_sats: int = 0
    hdop: float = 99.0
    speed_mps: float | None = None   # ground speed, if the receiver reports it
    course_rad: float | None = None  # course over ground (compass), if reported


def _checksum_ok(sentence: str) -> bool:
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, _, given = sentence[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(given[:2], 16)
    except ValueError:
        return False


def _dm_to_deg(value: str, hemi: str) -> float:
    """NMEA ddmm.mmmm / dddmm.mmmm -> signed decimal degrees."""
    raw = float(value)
    degrees = int(raw // 100)
    result = degrees + (raw - degrees * 100) / 60.0
    return -result if hemi in ("S", "W") else result


def _float(text: str) -> float | None:
    try:
        return float(text)
    except ValueError:
        return None


class NmeaParser:
    """Feeds NMEA lines; emits a GpsFix on every GGA (position) sentence.

    Speed and course come from RMC/VTG and are attached to the next GGA.
    """

    def __init__(self):
        self._seq = 0
        self._speed = None
        self._course = None
        self._motion_t = 0.0
        self.bad_sentences = 0

    def feed(self, line: str, now: float) -> GpsFix | None:
        line = line.strip()
        if not line:
            return None
        if not _checksum_ok(line):
            self.bad_sentences += 1
            return None
        fields = line[1:].split("*")[0].split(",")
        kind = fields[0][2:]  # drop talker id (GP, GN, ...)
        try:
            if kind == "RMC" and len(fields) > 8:
                self._set_motion(fields[7], fields[8], now)
            elif kind == "VTG" and len(fields) > 5:
                self._set_motion(fields[5], fields[1], now)
            elif kind == "GGA" and len(fields) > 8:
                return self._gga(fields, now)
        except (ValueError, IndexError):
            self.bad_sentences += 1
        return None

    def _set_motion(self, knots: str, course_deg: str, now: float) -> None:
        speed = _float(knots)
        course = _float(course_deg)
        self._speed = None if speed is None else speed * KNOTS_TO_MPS
        self._course = None if course is None else math.radians(course)
        self._motion_t = now

    def _gga(self, f: list[str], now: float) -> GpsFix | None:
        quality = int(f[6] or 0)
        if not f[2] or not f[4]:
            # No position yet; still report so the caller can see "no fix".
            lat = lon = 0.0
            quality = 0
        else:
            lat = _dm_to_deg(f[2], f[3])
            lon = _dm_to_deg(f[4], f[5])
        motion_fresh = (now - self._motion_t) < 1.0
        self._seq += 1
        return GpsFix(
            t=now,
            seq=self._seq,
            lat=lat,
            lon=lon,
            quality=quality,
            num_sats=int(f[7] or 0),
            hdop=_float(f[8]) or 99.0,
            speed_mps=self._speed if motion_fresh else None,
            course_rad=self._course if motion_fresh else None,
        )


class GpsReader:
    """Reads NMEA from a serial port in a daemon thread.

    ``latest()`` never blocks. If the port cannot be opened (GPS unplugged) the
    thread keeps retrying; the navigator just sees no fix and stays stopped.
    """

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._lock = threading.Lock()
        self._fix: GpsFix | None = None
        self._parser = NmeaParser()
        self._stop = threading.Event()
        self._thread = None
        self.connected = False

    def start(self) -> "GpsReader":
        self._thread = threading.Thread(target=self._run, daemon=True, name="gps-reader")
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def latest(self) -> GpsFix | None:
        with self._lock:
            return self._fix

    def _run(self) -> None:
        import serial

        warned = False
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baudrate, timeout=1.0) as ser:
                    self.connected = True
                    warned = False
                    print(f"[gps] Reading NMEA from {self.port} @ {self.baudrate}")
                    while not self._stop.is_set():
                        raw = ser.readline()
                        if not raw:
                            continue
                        line = raw.decode("ascii", errors="ignore")
                        fix = self._parser.feed(line, time.monotonic())
                        if fix is not None:
                            with self._lock:
                                self._fix = fix
            except Exception as exc:  # port missing, unplugged, permission...
                self.connected = False
                if not warned:
                    print(f"[gps] WARNING: cannot read {self.port}: {exc} (retrying)")
                    warned = True
                self._stop.wait(2.0)


__all__ = ["GpsFix", "GpsReader", "NmeaParser", "QUALITY_NAMES"]
