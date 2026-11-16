# ===============================================
# JEEPBOT CONFIG FILE
# ===============================================

# ===============================================
# CONTROLLER SELECTION
# ===============================================
from controller_profiles import resolve as _resolve_controller

CONTROLLER = "radiomaster"

_ACTIVE_CONTROLLER = _resolve_controller(CONTROLLER)
# Substring used by the reader class to select the USB device by name.
CONTROLLER_DEVICE_NAME = _ACTIVE_CONTROLLER["device_name"]

# ===============================================
# Steering — VESC motor + encoder (SetPosition)
# Degrees map to encoder position, NOT servo PWM.
# ===============================================
DEFAULT_STEERING_PORT        = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0"
DEFAULT_STEERING_AXIS        = _ACTIVE_CONTROLLER["steering_axis"]
DEFAULT_STEERING_CENTER_DEG  = -5.00
DEFAULT_STEERING_LEFT_DEG    = -45
DEFAULT_STEERING_RIGHT_DEG   = 45
DEFAULT_STEERING_WRAP_DEG    = 0.0
DEFAULT_STEERING_RAMP_DEG_PER_S = 280.0
DEFAULT_STEERING_MIN_STEP_DEG   = 0.8
DEFAULT_STEERING_SMOOTHING   = 0.03
DEFAULT_INVERT_STEERING      = _ACTIVE_CONTROLLER["invert_steering"]

# ===============================================
# Drive — VESC duty-cycle (4WD)
# ===============================================
DEFAULT_DRIVE_PORT           = "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0"
DEFAULT_DRIVE_AXIS           = _ACTIVE_CONTROLLER["drive_axis"]
DEFAULT_INVERT_DRIVE         = _ACTIVE_CONTROLLER["invert_drive"]
DEFAULT_MAX_DUTY             = 0.35
DEFAULT_DRIVE_RAMP_DUTY_PER_S = 0.35
DEFAULT_DRIVE_SMOOTHING      = 0.08
DEFAULT_DRIVE_BRAKE_CURRENT_A = 0.0

# ===============================================
# Controller / loop
# ===============================================
DEFAULT_DEADMAN_BUTTON       = _ACTIVE_CONTROLLER["deadman_button"]
DEFAULT_RATE_HZ              = 100.0
DEFAULT_BAUDRATE             = 115200
DEFAULT_DEADZONE             = _ACTIVE_CONTROLLER["deadzone"]
DEFAULT_NO_DEADMAN           = False
DEFAULT_EXPO                 = 0.35

# ===============================================
# Web server + OAK-D Pro camera
# Both run as in-process daemon threads.
# ===============================================
ENABLE_WEB_SERVER            = True
WEB_HOST                     = "0.0.0.0"
WEB_PORT                     = 8887
WEB_PORT_REAR = 8888
# Drive modes: manual | record | auto_steering | full_drive
DEFAULT_DRIVE_MODE           = "manual"

# ===============================================
# Camera toggles + resolution (front / rear independent)
# ===============================================

ENABLE_FRONT_CAMERA          = True
ENABLE_REAR_CAMERA           = True

# Capture (and stream) resolution per camera as (width, height).
# Common OAK-D sizes: (1920, 1080), (1280, 720), (640, 480).
FRONT_CAMERA_RESOLUTION      = (640, 480)
REAR_CAMERA_RESOLUTION       = (640, 480)

# front may come up as device 0 or 1. If front and rear are swapped, flip this:
SWAP_CAMERAS                 = False

# Most reliable across reboots: pin each role to a specific OAK-D serial (MxId).
# Leave both "" to use discovery order + SWAP_CAMERAS above. Serials are printed
# at startup ("[camera] Found N OAK device(s): [...]"), or list them with:
#   python3 -c "import depthai as dai; print([d.getMxId() for d in dai.Device.getAllAvailableDevices()])"
FRONT_CAMERA_MXID = "18443010D128850E00"   # confirmed working — move to front
REAR_CAMERA_MXID  = "1844301021F3840E00"

# ===============================================
# YOLO detection overlay
# ===============================================
# Requires running with the YOLO virtualenv:
#   source /home/jeepbot/yolo-test/bin/activate
ENABLE_YOLO_DETECTION        = True
YOLO_MODEL_PATH              = "/home/jeepbot/yolo_demo/best_v2.pt"
YOLO_IMGSZ                   = 320
YOLO_CONFIDENCE              = 0.25
YOLO_INTERVAL_S              = 0.20

# Estimate object distance by sampling the aligned OAK-D stereo depth map
# inside each YOLO box. Depth values are millimeters.
ENABLE_DEPTH_DISTANCE        = True
DEPTH_MIN_MM                 = 150
DEPTH_MAX_MM                 = 10000
DEPTH_ROI_SCALE              = 0.35

# ===============================================
# Autonomous driving
# ===============================================
# Path to trained Keras model for auto_steering / full_drive.
# Set to None (or a non-existent path) to disable AI modes.
DEFAULT_MODEL_PATH           = "models/jeep_pilot.keras"

# Root directory for tub recordings (sub-folders created per session).
DEFAULT_TUB_PATH             = "tub"


# ===============================================
# GPS / GNSS waypoint navigation (PID path follower) — see GPS_NAV.md
# Adds web modes: gps_steering (GPS steers, you throttle) and gps_drive.
# ===============================================
ENABLE_GPS_NAV               = False
# Use a /dev/serial/by-id/... path so the port never swaps with the VESCs.
GPS_PORT                     = "/dev/ttyUSB0"
GPS_BAUDRATE                 = 460800
# Waypoint CSV (lat,lon). Build one with: python3 gps_tool.py make-path <log>
GPS_NAV_PATH_FILE            = "gps_nav/paths/path.csv"
# Every GPS fix + controller term is logged here (all modes). None disables.
GPS_NAV_LOG_DIR              = "gps_logs"

# Drive duty used in gps_drive at full "throttle" (capped by DEFAULT_MAX_DUTY).
GPS_NAV_CRUISE_DUTY          = 0.10

# Lateral PID: steering (-1..+1) per metre of cross-track error.
GPS_NAV_KP_LAT               = 0.35
GPS_NAV_KI_LAT               = 0.0
GPS_NAV_KD_LAT               = 0.0
GPS_NAV_LAT_LIMIT            = 0.5    # max steering from the lateral PID alone
# Heading PD: steering per radian of heading error.
GPS_NAV_KP_HEAD              = 1.2
GPS_NAV_KD_HEAD              = 0.0
GPS_NAV_LOOKAHEAD_M          = 1.5

# Safety gates: outside these the navigator commands a stop.
GPS_NAV_MIN_QUALITY          = 1      # 1 any fix, 2 DGPS/SBAS, 5 RTK float, 4 RTK fixed
GPS_NAV_MAX_HDOP             = 2.5
GPS_NAV_FIX_TIMEOUT_S        = 1.0
GPS_NAV_MAX_CTE_M            = 4.0
GPS_NAV_GOAL_TOLERANCE_M     = 1.0

# Largest sideways shift the avoidance team may request (set_lateral_offset).
GPS_NAV_MAX_OFFSET_M         = 2.0


# ===============================================
# Start JeepBot from terminal
#   python3 myconfig.py drive
# ===============================================
if __name__ == "__main__":
    import sys
    from jeepbot_drive import drive

    if len(sys.argv) < 2:
        print("Usage: python3 myconfig.py drive")
        raise SystemExit(1)

    command = sys.argv[1]
    sys.argv.pop(1)

    if command == "drive":
        drive(config=sys.modules[__name__])
    else:
        print(f"Unknown command: {command}")
        print("Use: python3 myconfig.py drive")
