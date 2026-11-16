"""Run from myjeepbot_v6_2:  python3 -m unittest gps_nav.test_gps_nav -v"""
import contextlib
import io
import math
import os
import tempfile
import unittest

from gps_nav.geo import LocalFrame, bearing, wrap_pi
from gps_nav.gps_reader import GpsFix, NmeaParser
from gps_nav.navigator import GpsNavigator, NavConfig
from gps_nav.path import PathTracker, load_waypoints_csv
from gps_nav.pid import PID
from gps_nav.sim import SimGps, demo_path, simulate


def nmea(body: str) -> str:
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return f"${body}*{checksum:02X}"


def quiet_nav(**cfg) -> GpsNavigator:
    return GpsNavigator(NavConfig(**cfg))


def run_sim(nav, kind, **kwargs):
    with contextlib.redirect_stdout(io.StringIO()):
        return simulate(nav, demo_path(kind), **kwargs)


class TestGeo(unittest.TestCase):
    def test_round_trip_and_scale(self):
        frame = LocalFrame(32.88, -117.23)
        lat, lon = frame.to_latlon(12.5, -40.0)
        east, north = frame.to_xy(lat, lon)
        self.assertAlmostEqual(east, 12.5, places=6)
        self.assertAlmostEqual(north, -40.0, places=6)
        # 1e-5 deg of latitude is ~1.11 m
        self.assertAlmostEqual(frame.to_xy(32.88001, -117.23)[1], 1.113, places=2)

    def test_bearing_is_compass(self):
        self.assertAlmostEqual(bearing(0, 1), 0.0)
        self.assertAlmostEqual(bearing(1, 0), math.pi / 2)
        self.assertAlmostEqual(wrap_pi(3 * math.pi / 2), -math.pi / 2)


class TestNmea(unittest.TestCase):
    def test_gga_with_rmc_motion(self):
        p = NmeaParser()
        self.assertIsNone(p.feed(nmea("GNRMC,123519,A,3252.866,N,11714.040,W,2.0,90.0,230394,,"), 1.0))
        fix = p.feed(nmea("GNGGA,123519,3252.866,N,11714.040,W,4,12,0.9,100.0,M,,M,,"), 1.05)
        self.assertAlmostEqual(fix.lat, 32.8811, places=4)
        self.assertAlmostEqual(fix.lon, -117.2340, places=4)
        self.assertEqual((fix.quality, fix.num_sats), (4, 12))
        self.assertAlmostEqual(fix.speed_mps, 2.0 * 0.514444, places=4)
        self.assertAlmostEqual(fix.course_rad, math.pi / 2, places=4)

    def test_bad_checksum_rejected(self):
        p = NmeaParser()
        self.assertIsNone(p.feed("$GNGGA,123519,3252.866,N,11714.040,W,4,12,0.9,100.0,M,,M,,*00", 1.0))
        self.assertEqual(p.bad_sentences, 1)

    def test_empty_position_is_no_fix(self):
        fix = NmeaParser().feed(nmea("GNGGA,123519,,,,,0,00,99.9,,,,,,"), 1.0)
        self.assertEqual(fix.quality, 0)


class TestPath(unittest.TestCase):
    def test_cross_track_sign_and_progress(self):
        # Path heads north then east.
        tracker = PathTracker([(0, 0), (0, 10), (10, 10)], goal_tolerance=0.5)
        s = tracker.update(1.0, 5.0)
        self.assertAlmostEqual(s.cte_right, 1.0)       # east of a northbound path = right
        self.assertAlmostEqual(s.path_bearing, 0.0)
        self.assertAlmostEqual(s.dist_remaining, 15.0)
        s = tracker.update(5.0, 9.0)                   # second leg, south of eastbound = right
        self.assertEqual(s.segment, 1)
        self.assertAlmostEqual(s.cte_right, 1.0)
        self.assertFalse(s.done)
        self.assertTrue(tracker.update(9.8, 10.0).done)

    def test_lookahead_sees_the_turn(self):
        tracker = PathTracker([(0, 0), (0, 10), (10, 10)])
        self.assertAlmostEqual(tracker.update(0, 9.5, lookahead=2.0).path_bearing, math.pi / 2)

    def test_reset_engages_mid_path(self):
        tracker = PathTracker([(0, 0), (0, 10), (10, 10)])
        tracker.reset(6.0, 10.5)
        self.assertEqual(tracker.update(6.0, 10.5).segment, 1)

    def test_csv_with_header_and_extra_columns(self):
        with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as fh:
            fh.write("# comment\nname,lon,lat\na,-117.2,32.8\nb,-117.3,32.9\n")
        try:
            self.assertEqual(load_waypoints_csv(fh.name), [(32.8, -117.2), (32.9, -117.3)])
        finally:
            os.unlink(fh.name)


class TestPid(unittest.TestCase):
    def test_terms_and_limits(self):
        pid = PID(kp=1.0, ki=1.0, kd=0.0, i_limit=0.2, out_limit=1.0)
        for _ in range(100):
            pid.update(0.5, 0.1)
        self.assertAlmostEqual(pid.i_term, 0.2)         # integral clamped
        self.assertAlmostEqual(pid.output, 0.7)
        pid.update(0.5, 0.1, freeze_integral=True)
        self.assertAlmostEqual(pid.i_term, 0.2)

    def test_no_derivative_kick_on_setpoint_step(self):
        pid = PID(kp=0.0, kd=1.0)
        pid.update(0.0, 0.1, measurement=0.0)
        pid.update(5.0, 0.1, measurement=0.0)           # setpoint jumped, car did not
        self.assertEqual(pid.d_term, 0.0)


class TestNavigatorSafety(unittest.TestCase):
    def setUp(self):
        self.nav = quiet_nav()
        with contextlib.redirect_stdout(io.StringIO()):
            self.nav.set_path(demo_path("straight"))
        self.lat, self.lon = demo_path("straight")[0]

    def fix(self, t, seq, **kw):
        base = dict(t=t, seq=seq, lat=self.lat, lon=self.lon, quality=4, num_sats=12,
                    hdop=0.8, speed_mps=1.0, course_rad=0.0)
        base.update(kw)
        return GpsFix(**base)

    def test_no_fix_and_stale_stop(self):
        self.assertEqual(self.nav.step(None, 0.0).state, "NO_FIX")
        self.assertEqual(self.nav.step(self.fix(0.0, 1), 0.05).state, "TRACKING")
        cmd = self.nav.step(self.fix(0.0, 1), 1.5)       # same fix, 1.5 s old
        self.assertEqual((cmd.state, cmd.throttle, cmd.ok), ("STALE", 0.0, False))

    def test_low_quality_stops(self):
        self.assertEqual(self.nav.step(self.fix(0.0, 1, quality=0), 0.0).state, "LOW_QUALITY")
        self.assertEqual(self.nav.step(self.fix(0.1, 2, hdop=9.0), 0.1).state, "LOW_QUALITY")

    def test_rtk_fixed_outranks_float(self):
        self.nav.set_gains(min_quality=4)
        self.assertEqual(self.nav.step(self.fix(0.0, 1, quality=5), 0.0).state, "LOW_QUALITY")
        self.assertEqual(self.nav.step(self.fix(0.1, 2, quality=4), 0.1).state, "TRACKING")

    def test_off_path_latches_until_reengaged(self):
        far_lat, far_lon = LocalFrame(self.lat, self.lon).to_latlon(10.0, 5.0)
        self.assertEqual(self.nav.step(self.fix(0.0, 1, lat=far_lat, lon=far_lon), 0.0).state, "OFF_PATH")
        self.assertEqual(self.nav.step(self.fix(0.1, 2), 0.1).state, "OFF_PATH")
        self.nav.step(self.fix(0.2, 3), 0.2, active=False)
        self.assertEqual(self.nav.step(self.fix(0.3, 4), 0.3).state, "TRACKING")

    def test_inactive_is_idle_and_zero(self):
        cmd = self.nav.step(self.fix(0.0, 1), 0.0, active=False)
        self.assertEqual((cmd.state, cmd.steering, cmd.throttle, cmd.ok), ("IDLE", 0.0, 0.0, False))

    def test_steering_signs(self):
        frame = LocalFrame(self.lat, self.lon)
        lat, lon = frame.to_latlon(-1.0, 5.0)            # 1 m LEFT of a northbound path
        self.assertGreater(self.nav.step(self.fix(0.0, 1, lat=lat, lon=lon), 0.0).steering, 0.0)
        nav2 = quiet_nav()
        with contextlib.redirect_stdout(io.StringIO()):
            nav2.set_path(demo_path("straight"))
        lat, lon = frame.to_latlon(0.0, 5.0)             # on the path, pointing 20 deg right
        cmd = nav2.step(self.fix(0.0, 1, lat=lat, lon=lon, course_rad=math.radians(20)), 0.0)
        self.assertLess(cmd.steering, 0.0)

    def test_offset_is_clamped_and_sanitised(self):
        self.nav.set_lateral_offset(99.0)
        self.assertEqual(self.nav.get_lateral_offset(), self.nav.cfg.max_offset_m)
        self.nav.set_lateral_offset(float("nan"))
        self.assertEqual(self.nav.get_lateral_offset(), 0.0)


class TestClosedLoop(unittest.TestCase):
    def test_converges_from_two_metres_off(self):
        r = run_sim(quiet_nav(), "straight", start_xy=(2.0, 0.0))
        self.assertTrue(r.finished, r.final_state)
        _, rms, worst = r.cte_stats(after_s=10.0)
        self.assertLess(rms, 0.15)
        self.assertLess(worst, 0.4)

    def test_completes_l_and_s_paths(self):
        for kind in ("l", "s"):
            r = run_sim(quiet_nav(), kind)
            self.assertTrue(r.finished, f"{kind}: {r.final_state}")
            self.assertLess(r.cte_stats()[2], 1.5, kind)

    def test_starts_pointing_the_wrong_way(self):
        r = run_sim(quiet_nav(), "straight", start_heading_deg=70.0)
        self.assertTrue(r.finished, r.final_state)

    def test_survives_plain_gps_noise_at_5hz(self):
        r = run_sim(quiet_nav(), "l", gps=SimGps(rate_hz=5.0, noise_m=0.4, quality=1, hdop=1.2))
        self.assertTrue(r.finished, r.final_state)

    def test_lateral_offset_shifts_the_tracked_line(self):
        # The avoidance hook: +1 m (right) between t=8 s and t=18 s, then back.
        r = run_sim(quiet_nav(), "straight",
                    offset_schedule=lambda t: 1.0 if 8.0 <= t < 18.0 else 0.0)
        self.assertTrue(r.finished, r.final_state)
        held = [c for t, c in zip(r.t, r.cte) if 14.0 <= t < 18.0]
        back = [c for t, c in zip(r.t, r.cte) if t >= 24.0]
        self.assertAlmostEqual(sum(held) / len(held), 1.0, delta=0.15)
        self.assertAlmostEqual(sum(back) / len(back), 0.0, delta=0.15)

    def test_gps_dropout_stops_then_resumes(self):
        nav = quiet_nav()
        r = run_sim(nav, "straight", gps_dropout=(8.0, 12.0))
        self.assertTrue(r.finished, r.final_state)
        # The car must not have covered cruise distance during the 4 s dropout.
        y_at = lambda when: min(zip(r.t, r.y), key=lambda p: abs(p[0] - when))[1]
        self.assertLess(y_at(12.0) - y_at(9.5), 1.5)


if __name__ == "__main__":
    unittest.main()
