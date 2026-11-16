# GPS / GNSS waypoint navigation (PID)

Follows a recorded list of GPS waypoints by steering with a PID controller.
Lives in `gps_nav/`, is switched on with `ENABLE_GPS_NAV = True` in
`myconfig.py`, and adds two modes to the web UI:

| Mode | Steering | Throttle |
|---|---|---|
| `gps_steering` | GPS PID | **you**, on the joystick (use this for all tuning) |
| `gps_drive` | GPS PID | navigator, at `GPS_NAV_CRUISE_DUTY` |

Both modes only act while the **deadman button is held** (even with
`--no-deadman`). Releasing it centres the steering and ramps the drive to zero.

## How it works

```
GPS (NMEA serial) -> local east/north metres -> path tracker -> cross-track error (m, + = right of path)
                                                             -> path direction 1.5 m ahead
GPS course over ground --------------------------------------> heading

lateral_error = lateral_offset - cross_track
steering      = clamp(PID_lat(lateral_error), +/-LAT_LIMIT) + PD_head(path_direction - heading)
```

* The **lateral PID** turns the car towards the line. Its output is capped at
  `GPS_NAV_LAT_LIMIT`, which caps the approach angle so a big error becomes a
  steady intercept, not a circle.
* The **heading term** damps the lateral loop and lines the car up with the path.
* Steering is normalised `-1..+1` (`+1` = full right), then mapped through the
  same `DEFAULT_STEERING_LEFT/CENTER/RIGHT_DEG` endpoints as the joystick.
* The controller updates once per GPS fix (5-10 Hz) and holds its output in between.
* There is no compass or IMU: heading comes from GPS motion, so it is unknown
  until the car has rolled forward ~0.5 m. In that phase (`ACQUIRE_HEADING`) the
  car goes straight. **Start each run already pointing roughly along the path.**
  Reversing confuses the heading estimate; GPS modes are forward-only.

### When it refuses to drive

| State | Meaning |
|---|---|
| `NO_FIX` / `STALE` | no GPS data, or none for `GPS_NAV_FIX_TIMEOUT_S` |
| `LOW_QUALITY` | fix type below `GPS_NAV_MIN_QUALITY` or HDOP above `GPS_NAV_MAX_HDOP` |
| `NO_PATH` | waypoint file missing or unreadable |
| `OFF_PATH` | more than `GPS_NAV_MAX_CTE_M` from the line. Latched: release and re-press the deadman |
| `DONE` | within `GPS_NAV_GOAL_TOLERANCE_M` of the last waypoint |

## Object-avoidance hook (not implemented here)

Avoidance does not take over the steering. It moves the line the PID tracks:

```python
navigator.set_lateral_offset(0.8)   # track a line 0.8 m to the RIGHT of the path
navigator.set_lateral_offset(0.0)   # back to the recorded path
```

Positive = right, metres, clamped to `GPS_NAV_MAX_OFFSET_M`, thread-safe, and
0.0 until someone calls it. `navigator` is created in `jeepbot_drive.drive()`.
Nothing calls this yet. Things the two teams still need to agree on: who calls
it and how often, what happens if the caller dies while an offset is set (there
is no timeout today), and whether avoidance also needs to slow or stop the car.

## Files

| File | Purpose |
|---|---|
| `gps_nav/navigator.py` | controller, safety gates, logging, offset hook |
| `gps_nav/pid.py` | PID (integral clamp, filtered derivative on measurement) |
| `gps_nav/path.py` | waypoint CSV loader and path tracker |
| `gps_nav/gps_reader.py` | NMEA serial reader thread (GGA + RMC/VTG) |
| `gps_nav/sim.py` | bicycle-model simulator |
| `gps_nav/test_gps_nav.py` | `python3 -m unittest gps_nav.test_gps_nav` |
| `gps_tool.py` | `monitor`, `make-path`, `report`, `plot`, `sim` |

---

# Physical testing and tuning

Do the steps in order. Each one removes a cause of confusing behaviour in the
next. Until step 6 passes, **nobody rides in the jeep**, test in an open area
with a clear margin of several metres, and keep the max duty low.

## 0. Things to find out first

- [ ] Which GNSS receiver is on the jeep, its serial port and baud rate. Set
      `GPS_PORT` (prefer `/dev/serial/by-id/...`) and `GPS_BAUDRATE`. The
      default 460800 is copied from the earlier ROS stack and may be wrong.
- [ ] That it outputs NMEA **GGA** (required) and **RMC or VTG** (for speed and
      course). Set the output rate to 10 Hz if the receiver allows it.
- [ ] Whether RTK corrections are available. Plain GPS wanders 1-2 m, which is
      wider than a sidewalk; the PID will faithfully follow that wander.
- [ ] Mount the antenna rigidly, clear view of the sky, ideally on the
      centreline above the rear axle, away from the motors and VESCs.

## 1. GPS bench check (car stationary, outdoors)

```bash
python3 gps_tool.py monitor
```

- [ ] Fix type, satellites and HDOP appear; note the update rate.
- [ ] Leave it 5 minutes and watch how far lat/lon drift. 1e-5 degrees of
      latitude is about 1.1 m. That drift is your accuracy floor.
- [ ] Walk the car 10 m: `speed` and `course` should show sensible values
      (course 0 = north, 90 = east).

Set `GPS_NAV_MIN_QUALITY` and `GPS_NAV_MAX_HDOP` from what you see (for RTK use
`5` to require float or better, `4` for fixed only).

## 2. Steering direction (wheels off the ground)

Set `ENABLE_GPS_NAV = True` and start `python3 myconfig.py drive`.

- [ ] In manual mode, confirm stick right turns the wheels right and forward
      stick spins the wheels forward. The navigator assumes both.
- [ ] Measure the real steering range. If left and right lock are not
      symmetric about centre the car pulls to one side; fix
      `DEFAULT_STEERING_CENTER_DEG` before tuning anything.

## 3. Record a path

Paths are made from the log that is written whenever GPS nav is enabled.

1. Start the drive program, drive the route manually at walking pace. Begin
   with a **straight line of 30 m or more**.
2. Stop the program, then:

```bash
python3 gps_tool.py make-path gps_logs/nav_<timestamp>.csv --out gps_nav/paths/path.csv
```

3. `python3 gps_tool.py plot gps_logs/nav_<timestamp>.csv` and check the track
   looks like what you drove. A jagged track means the GPS is too noisy to follow.

## 4. First closed-loop test: `gps_steering` on a straight

You control the throttle. Start on the path, pointing along it.

- [ ] Select **GPS Steer**, hold the deadman, feed in slow throttle.
- [ ] The state should go `ACQUIRE_HEADING` then `TRACKING` within about a metre.
- [ ] **If it steers away from the path, stop.** Re-check step 2.
- [ ] Repeat starting 1 m to the left, then 1 m to the right of the path.

## 5. Tuning the gains

Change one gain at a time in `myconfig.py`, repeat the same run, then compare:

```bash
python3 gps_tool.py report gps_logs/nav_<timestamp>.csv
python3 gps_tool.py plot   gps_logs/nav_<timestamp>.csv
```

Order:

1. **`GPS_NAV_KP_HEAD`** (start 1.2). With `GPS_NAV_KP_LAT = 0`, the car should
   turn parallel to the path and hold that direction without weaving. Weaving:
   lower it. Slow to line up: raise it.
2. **`GPS_NAV_KP_LAT`** (start 0.15, work up towards 0.35-0.5). Starting 1 m
   off, it should reach the line in a few metres and stay. Slow S-shaped
   weaving across the line: lower `KP_LAT` or raise `KP_HEAD`. Sits beside the
   line without closing: raise `KP_LAT`.
3. **`GPS_NAV_LAT_LIMIT`** (0.5). Lower it if the car dives at the path too steeply.
4. **`GPS_NAV_KD_LAT`** (0, try 0.1-0.2). Only if it still overshoots after
   1-3. It amplifies GPS noise, so twitchy steering means too much.
5. **`GPS_NAV_KI_LAT`** (0, try 0.02-0.05). Only if `report` shows a steady
   mean error on a long straight. Fix the steering centre first; that is the
   usual cause.
6. **`GPS_NAV_LOOKAHEAD_M`** (1.5). On a path with corners: turning late,
   raise it; cutting corners, lower it.

Gains depend on speed. Tune at the speed you intend to run and retune if the
cruise speed changes much. The defaults come from a simulator whose wheelbase,
steering angle and speed are guesses, so treat them as a starting point only.

Rough targets: RMS lateral error near the GPS noise floor from step 1, no
sustained weaving, steering saturated only in sharp corners.

## 6. `gps_drive` (navigator controls the throttle)

- [ ] Wheels off the ground first: find the duty that gives walking pace and
      set `GPS_NAV_CRUISE_DUTY` (starts at 0.10). It is open-loop duty, not
      speed control, so the car slows on grass and uphill and speeds up downhill.
- [ ] Run the straight, then the cornered path. Confirm it stops at the end (`DONE`).
- [ ] Failure tests, each should stop the car: release the deadman; unplug the
      GPS while moving (`STALE` within 1 s); cover the antenna; carry the car
      5 m off the path and engage (`OFF_PATH`).

## 7. Offset hook check (ahead of the avoidance team)

In `gps_steering` on a straight, temporarily add
`navigator.set_lateral_offset(1.0)` after the navigator is created. The car
should settle 1 m right of the recorded path, and the `offset` and `cte`
columns in the log should match. Remove the line afterwards.

## Known limits

- Open-loop throttle; no speed feedback.
- Heading comes from GPS motion only, so it is unreliable below ~0.4 m/s.
- Antenna position off the rear axle is not compensated.
- One path per run, no looping, forward only.
- Nothing here detects obstacles or people. A person must hold the deadman and watch the car.
