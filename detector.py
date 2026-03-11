"""
Regenerative Braking Detector by Kieranmcm07
--------------------------------------------
So how this works is that it detects when a EV/hybrid is in the regeneratrive braking mode by
reading data from an ELM327-compatible OBD-II adapter via the `python-OBD` library.

Regen is detected using one or more of these signals (depends which ones are available):
  1. Motor torque is negative (motor acting as generator)          [preferred]
  2. Battery current is positive / charging (current flowing in)  [fallback]
  3. Throttle is released + vehicle is decelerating               [heuristic]

Supports manufacturer-specific PIDs for:
  - Nissan Leaf (Gen 1 & 2)
  - Chevrolet Bolt / Volt
  - Toyota Prius / RAV4 Hybrid
  - Hyundai Ioniq / Kona EV
  - BMW i3 / i8
  - Generic OBD-II (heuristic fallback)

Requirements:
    pip install obd

Hardware:
    ELM327 Bluetooth/USB OBD-II adapter plugged into your car's OBD-II port.

Usage:
    python regen_detector.py                            # auto-detect port + generic mode
    python regen_detector.py --vehicle nissan_leaf      # Nissan Leaf specific PIDs
    python regen_detector.py --vehicle chevy_bolt       # Chevy Bolt/Volt specific PIDs
    python regen_detector.py --vehicle toyota_prius     # Toyota Prius/hybrid specific PIDs
    python regen_detector.py --vehicle hyundai_ioniq    # Hyundai EV specific PIDs
    python regen_detector.py --vehicle bmw_i3           # BMW i3/i8 specific PIDs
    python regen_detector.py --port /dev/ttyUSB0        # specify serial port manually
    python regen_detector.py --log regen_log.csv        # also write data to CSV
    python regen_detector.py --simulate                 # run on fake data (no car needed)
    python regen_detector.py --list-vehicles            # show all supported vehicles
"""

import argparse
import csv
import sys
import time
from datetime import datetime


# Dependency Checking and Imports Stuff :)
try:
    import obd
    from obd import OBDCommand
    from obd.protocols import ECU
except ImportError:
    print(
        "Error: 'python-OBD' library not found. Please install it with 'pip install obd'"
    )
    sys.exit(1)


# Configuration (make sure that the PIDs you want to use are supported by your car's ECU)
POLL_INTERVAL_SEC = 0.25  # How often to query the OBD adapter (seconds)
REGEN_TORQUE_THRESHOLD = -5  # Nm  – torque below this = regen (motor braking)
REGEN_CURRENT_THRESHOLD = 1.0  # A   – battery current above this = charging = regen
DECEL_THRESHOLD = -0.5  # m/s² – heuristic: deceleration stronger than this
THROTTLE_OFF_THRESHOLD = 5  # %   – throttle below this = foot off pedal


# Manufacturer-specific PID definitions

# Custom PIDs use raw OBD mode 0x21 or 0x22 commands.
# Each entry: (mode, pid_hex, bytes, description, decode_fn)
# decode_fn receives raw bytes and returns a float value.


def _decode_nissan_leaf_battery_current(messages):
    # Nissan Leaf battery current. Positive = charging (regen).
    d = messages[0].data
    raw = d[3] << 8 | d[4]
    # Signed 16-bit, LSB = 0.1A, positive = discharge on some firmwares
    if raw > 32767:
        raw -= 65536
    return raw * 0.1  # amps; positive means charging in our convention


def _decode_nissan_leaf_motor_torque(messages):
    # Nissan Leaf motor torque. Negative = regen.
    d = messages[0].data
    raw = d[3] << 8 | d[4]
    if raw > 32767:
        raw -= 65536
    return raw * 0.5  # Nm


def _decode_chevy_bolt_battery_current(messages):
    # Chevy Bolt HV battery current.
    d = messages[0].data
    raw = d[3] << 8 | d[4]
    if raw > 32767:
        raw -= 65536
    return raw * 0.05  # amps


def _decode_toyota_prius_motor_torque(messages):
    # Toyota Prius MG2 motor torque.
    d = messages[0].data
    raw = d[3] << 8 | d[4]
    if raw > 32767:
        raw -= 65536
    return raw * 0.317  # Nm


def _decode_hyundai_ioniq_battery_current(messages):
    # Hyundai Ioniq / Kona EV battery current.
    d = messages[0].data
    raw = d[3] << 8 | d[4]
    if raw > 32767:
        raw -= 65536
    return raw * 0.1  # amps


def _decode_bmw_i3_battery_current(messages):
    # BMW i3 HV battery current.
    d = messages[0].data
    raw = d[3] << 8 | d[4]
    if raw > 32767:
        raw -= 65536
    return raw * 0.1  # amps


# Vehicle profiles | (Copy and pasted from another source, so may be some redundant fields)
VEHICLE_PROFILES = {
    "generic": {
        "name": "Generic OBD-II",
        "description": "Heuristic detection using standard OBD-II PIDs (works on any car)",
        "torque_cmd": None,
        "current_cmd": None,
    },
    "nissan_leaf": {
        "name": "Nissan Leaf (Gen 1 & 2)",
        "description": "Uses Leaf-specific battery current and motor torque PIDs",
        "torque_cmd": OBDCommand(
            "LEAF_MOTOR_TORQUE",
            "Nissan Leaf Motor Torque",
            b"022101",
            6,
            _decode_nissan_leaf_motor_torque,
            ECU.ENGINE,
            True,
        ),
        "current_cmd": OBDCommand(
            "LEAF_BATTERY_CURRENT",
            "Nissan Leaf Battery Current",
            b"022101",
            6,
            _decode_nissan_leaf_battery_current,
            ECU.ENGINE,
            True,
        ),
    },
    "chevy_bolt": {
        "name": "Chevrolet Bolt EV / Volt",
        "description": "Uses GM-specific HV battery current PID",
        "torque_cmd": None,
        "current_cmd": OBDCommand(
            "BOLT_BATTERY_CURRENT",
            "Chevy Bolt HV Battery Current",
            b"222B16",
            6,
            _decode_chevy_bolt_battery_current,
            ECU.ENGINE,
            True,
        ),
    },
    "toyota_prius": {
        "name": "Toyota Prius / RAV4 Hybrid",
        "description": "Uses Toyota MG2 torque PID for accurate regen detection",
        "torque_cmd": OBDCommand(
            "PRIUS_MG2_TORQUE",
            "Toyota Prius MG2 Motor Torque",
            b"21E4",
            6,
            _decode_toyota_prius_motor_torque,
            ECU.ENGINE,
            True,
        ),
        "current_cmd": None,
    },
    "hyundai_ioniq": {
        "name": "Hyundai Ioniq / Kona EV",
        "description": "Uses Hyundai-specific battery current PID",
        "torque_cmd": None,
        "current_cmd": OBDCommand(
            "IONIQ_BATTERY_CURRENT",
            "Hyundai Ioniq Battery Current",
            b"220101",
            6,
            _decode_hyundai_ioniq_battery_current,
            ECU.ENGINE,
            True,
        ),
    },
    "bmw_i3": {
        "name": "BMW i3 / i8",
        "description": "Uses BMW-specific HV battery current PID",
        "torque_cmd": None,
        "current_cmd": OBDCommand(
            "BMW_BATTERY_CURRENT",
            "BMW i3 Battery Current",
            b"220D3C",
            6,
            _decode_bmw_i3_battery_current,
            ECU.ENGINE,
            True,
        ),
    },
}


# Helper: simple numerical differentition for speed to get acceleration
# Recently learned this in object oriented programming class, so thought it would be fun to implement here for the deceleration heuristic in generic mode.
class Differnetiator:
    def __init__(self):
        self._prev_value = None
        self._prev_time = None

    def update(self, value: float) -> float | None:
        now = time.monotonic()
        result = None
        if self._prev_value is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                result = (value - self._prev_value) / dt
        self._prev_value = value
        self._prev_time = now
        return result


# The actual detector thing, so that we can keep track of state and stuff. Put it in a class, keeping stuff organised and easy to fix later if needed.
# (Used Claude AI to help me make this class)
class RegenDetector:
    def __init__(
        self,
        port: str | None = None,
        log_path: str | None = None,
        simulate: bool = False,
        vehicle: str = "generic",
    ):
        self.port = port
        self.log_path = log_path
        self.simulate = simulate
        self.vehicle_key = vehicle
        self.vehicle_profile = VEHICLE_PROFILES.get(
            vehicle, VEHICLE_PROFILES["generic"]
        )
        self.connection = None
        self._csv_file = None
        self._csv_writer = None
        self._speed_diff = Differnetiator()

        # some stats
        self.total_samples = 0
        self.regen_samples = 0
        self.regen_sessions = 0
        self._in_regen = False
        self._session_start = None

# Connection
def connect(self):
    if self.simulate:
        print("Running in simulation mode with fake data (no car needed)")
        return
    print(f"Vehicle Profile: {self.vehicle_profile['name']}")
    print(f"{self.vehicle_profile['description']}\n")
    print("Connecting to OBD-II adapter...", end=" ",flush=True)
    
    kwargs = {"portstr": self.port} if self.port else {}
    self.connection = obd.OBD(**kwargs)
    
    if not self.connection.is_connected():
    