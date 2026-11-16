#!/usr/bin/env python3
"""GPS navigation helper commands.

  python3 gps_tool.py monitor                 # live fix quality / position / speed
  python3 gps_tool.py make-path LOG.csv       # turn a driven log into a waypoint path
  python3 gps_tool.py report LOG.csv          # tracking-error numbers for a run
  python3 gps_tool.py plot LOG.csv            # plots (needs matplotlib)
  python3 gps_tool.py sim --path l            # try the current myconfig gains offline
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time


def _num(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _read_log(path: str) -> list[dict]:
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def cmd_monitor(args) -> int:
    import myconfig
    from gps_nav.gps_reader import GpsReader, QUALITY_NAMES

    port = args.port or myconfig.GPS_PORT
    baud = args.baud or myconfig.GPS_BAUDRATE
    reader = GpsReader(port, baud).start()
    last_seq, last_t, rate = None, None, 0.0
    print("Ctrl+C to stop.")
    try:
        while True:
            fix = reader.latest()
            if fix is None:
                print("\rwaiting for NMEA GGA sentences...   ", end="", flush=True)
            elif fix.seq != last_seq:
                if last_t is not None and fix.t > last_t:
                    rate = 0.8 * rate + 0.2 / (fix.t - last_t) if rate else 1.0 / (fix.t - last_t)
                last_seq, last_t = fix.seq, fix.t
                speed = "--" if fix.speed_mps is None else f"{fix.speed_mps:.2f}"
                course = "--" if fix.course_rad is None else f"{math.degrees(fix.course_rad) % 360:.0f}"
                print(
                    f"\r{QUALITY_NAMES.get(fix.quality, fix.quality)} | sats={fix.num_sats} "
                    f"hdop={fix.hdop:.1f} | {fix.lat:.7f}, {fix.lon:.7f} | "
                    f"speed={speed} m/s course={course} deg | {rate:.1f} Hz   ",
                    end="", flush=True,
                )
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    return 0


def cmd_make_path(args) -> int:
    from gps_nav.geo import LocalFrame

    rows = [
        r for r in _read_log(args.log)
        if _num(r.get("lat")) and int(_num(r.get("quality")) or 0) > 0
    ]
    if args.min_quality:
        rows = [r for r in rows if int(float(r["quality"])) in args.min_quality]
    if len(rows) < 2:
        print("Not enough valid fixes in log.")
        return 1

    frame = LocalFrame(float(rows[0]["lat"]), float(rows[0]["lon"]))
    kept, last = [], None
    for r in rows:
        lat, lon = float(r["lat"]), float(r["lon"])
        x, y = frame.to_xy(lat, lon)
        if last is None or math.hypot(x - last[0], y - last[1]) >= args.spacing:
            kept.append((lat, lon))
            last = (x, y)
    if len(kept) < 2:
        print("Log covers less than one waypoint spacing; drive further.")
        return 1

    with open(args.out, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["lat", "lon"])
        for lat, lon in kept:
            writer.writerow([f"{lat:.8f}", f"{lon:.8f}"])
    print(f"Wrote {len(kept)} waypoints ({args.spacing} m spacing) to {args.out}")
    return 0


def cmd_report(args) -> int:
    rows = [r for r in _read_log(args.log) if r.get("state") == "TRACKING"]
    if not rows:
        print("No TRACKING rows in this log.")
        return 1
    errs = [_num(r["lat_error"]) for r in rows if _num(r["lat_error"]) is not None]
    steer = [_num(r["steering"]) for r in rows if _num(r["steering"]) is not None]
    t0, t1 = float(rows[0]["t"]), float(rows[-1]["t"])
    # Sign changes of the error, ignoring +/-0.1 m so GPS noise is not counted.
    big = [e for e in errs if abs(e) > 0.1]
    crossings = sum(1 for a, b in zip(big, big[1:]) if a * b < 0)
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    saturated = sum(1 for s in steer if abs(s) > 0.98) / max(1, len(steer))
    print(f"tracking time      : {t1 - t0:.1f} s ({len(rows)} fixes, {len(rows) / max(t1 - t0, 1e-6):.1f} Hz)")
    print(f"lateral error mean : {sum(errs) / len(errs):+.2f} m   (bias -> consider Ki or steering centre)")
    print(f"lateral error RMS  : {rms:.2f} m")
    print(f"lateral error max  : {max(abs(e) for e in errs):.2f} m")
    print(f"side-to-side swaps : {crossings} (error swinging past +/-0.1 m; many = oscillating, lower Kp or raise heading gain)")
    print(f"steering saturated : {100 * saturated:.0f}% of the time")
    return 0


def cmd_plot(args) -> int:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed: pip install matplotlib")
        return 1
    rows = [r for r in _read_log(args.log) if _num(r.get("x")) is not None]
    if not rows:
        print("No positions with a loaded path in this log.")
        return 1
    t = [float(r["t"]) - float(rows[0]["t"]) for r in rows]
    fig, ax = plt.subplots(1, 3, figsize=(16, 5))
    ax[0].plot([float(r["x"]) for r in rows], [float(r["y"]) for r in rows], ".-", ms=2)
    ax[0].set_aspect("equal")
    ax[0].set_title("Track (east/north, m)")
    for key, label in (("cte", "cross-track"), ("offset", "offset"), ("lat_error", "lateral error")):
        ax[1].plot(t, [_num(r[key]) if _num(r[key]) is not None else float("nan") for r in rows], label=label)
    ax[1].set_title("Lateral (m, + right)")
    ax[1].legend()
    for key in ("steering", "p_lat", "i_lat", "d_lat", "u_head"):
        ax[2].plot(t, [_num(r[key]) if _num(r[key]) is not None else float("nan") for r in rows], label=key)
    ax[2].set_title("Steering and PID terms")
    ax[2].legend()
    for a in ax[1:]:
        a.set_xlabel("s")
        a.grid(True)
    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=120)
        print(f"Saved {args.save}")
    else:
        plt.show()
    return 0


def cmd_sim(args) -> int:
    import myconfig
    from gps_nav.navigator import GpsNavigator, NavConfig
    from gps_nav.sim import SimGps, demo_path, simulate

    nav = GpsNavigator(NavConfig.from_module(myconfig))
    result = simulate(
        nav,
        demo_path(args.path),
        start_xy=(args.start_offset, 0.0),
        gps=SimGps(rate_hz=args.gps_hz, noise_m=args.noise),
    )
    mean, rms, worst = result.cte_stats(after_s=8.0)
    print(f"final state: {result.final_state} after {result.time_s:.1f} s")
    print(f"cross-track after 8 s: mean {mean:+.2f} m, RMS {rms:.2f} m, max {worst:.2f} m")
    print("(vehicle model is a guess; use this for sanity, not final gains)")
    return 0 if result.finished else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("monitor", help="show live GPS fixes")
    p.add_argument("--port")
    p.add_argument("--baud", type=int)
    p.set_defaults(func=cmd_monitor)

    p = sub.add_parser("make-path", help="build a waypoint CSV from a nav log")
    p.add_argument("log")
    p.add_argument("--out", default="gps_nav/paths/path.csv")
    p.add_argument("--spacing", type=float, default=1.0, help="metres between waypoints")
    p.add_argument("--min-quality", type=int, nargs="*", help="only keep these GGA quality codes, e.g. 4 5")
    p.set_defaults(func=cmd_make_path)

    p = sub.add_parser("report", help="tracking-error summary of a run")
    p.add_argument("log")
    p.set_defaults(func=cmd_report)

    p = sub.add_parser("plot", help="plot a nav log")
    p.add_argument("log")
    p.add_argument("--save", help="write a PNG instead of opening a window")
    p.set_defaults(func=cmd_plot)

    p = sub.add_parser("sim", help="simulate the current gains")
    p.add_argument("--path", default="l", choices=["straight", "l", "s"])
    p.add_argument("--start-offset", type=float, default=1.0, help="metres right of the path at start")
    p.add_argument("--gps-hz", type=float, default=10.0)
    p.add_argument("--noise", type=float, default=0.03, help="GPS noise sigma, metres")
    p.set_defaults(func=cmd_sim)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
