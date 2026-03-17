"""

NOTE: I have used Claude in this project to experiment with the AI model.

In this project im mainly trying to learn how OBD protocols work and othwr things.


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
    print("Connecting to OBD-II adapter...", end=" ", flush=True)

    kwargs = {"portstr": self.port} if self.port else {}
    self.connection = obd.OBD(**kwargs)

    if not self.connection.is_connected():
        print("FAILED!")
        print(" - Check that your ELM327 adapter is plugged in.")
        print(" - Try specifying --port manually (e.g. --port /dev/ttyUSB0 or COM3).")

    print(f"OK ({self.connection.port_name()})")
    print(f"Protocol: {self.connection.protocol_name()}\n")

    # register custom PIDs if defined
    profile = self.vehicle_profile
    if profile.get("torque_cmd"):
        self.connection.supported_commands.add(profile["torque_cmd"])
    if profile.get("current_cmd"):
        self.connection.supported_commands.add(profile["current_cmd"])


# Some csv logging stuff
def _open_log(self):
    # if not a log path then skip this yap
    if not self.log_path:
        return

    self._csv_file = open(self.log_path, "w", newline="")
    fieldnames = [
        "timestamp",
        "speed_kph",
        "throttle_pct",
        "torque_nm",
        "battery_current_a",
        "acceleration_ms2",
        "regen",
        "method",
        "vehicle",
    ]
    self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=fieldnames)
    self._csv_writer.writeheader()
    print(f"Logging all that data toooo: {self.log_path}\n")

    def _log_row(self, row: dict):
        if self._csv_writer:
            self._csv_writer.writerow(row)
            self._csv_file.flush()

    def _close_log(self):
        if self._csv_file:
            self._csv_file.close()


# OBD query helpers
def _query_standard(self, cmd) -> float | None:
    if not self.connection:
        return None
    resp = self.connection.query(cmd)
    if resp.is_null():
        return None
    return resp.value.magnitude


def _query_custom(self, cmd) -> float | None:
    if not self.connection:
        return None
    try:
        resp = self.connection.query(cmd)
        if resp.is_null():
            return None
        return resp.value
    except Exception:
        return None


# some simulated stuff (so like if theres no car to test it on and stuff)
_sim_step = 0


def _simulate_data(self) -> dict:
    import math

    t = self._sim_step * POLL_INTERVAL_SEC
    self._sim_step += 1
    cycle = t % 20
    if cycle < 5:
        speed, torque, throttle, current = cycle * 10, 80, 60, -15
    elif cycle < 8:
        speed, torque, throttle, curren = 50, 10, 20, -2
    elif cycle < 14:
        speed = max(0, 50 - (cycle - 8) * 8)
        torque, throttle, current = -30, 0, 18
    else:
        speed, torque, throttle, currrent = 0, 0, 0, 0
    noise = math.sin(t * 7) * 2
    return {
        "speed_kph": round(speed + noise, 1),
        "throttle_pct": round(max(0, throttle + noise), 1),
        "torque_nm": round(torque + noise, 1),
        "battery_current_a": round(current - noise, 1),
    }


# The actual regen detection logic (well i really hope it works, but i guess we will see when i test it on a car)
def _detect_regen(self, speed, throttle, torque, current, accel) -> tuple[bool, str]:
    # method 1: motor torque which is most reliable
    if torque is not None:
        return (torque < REGEN_TORQUE_THRESHOLD), "negative_torque"
    # method 2: battery current (positive = charging = regen)
    if current is not None:
        return (current > REGEN_CURRENT_THRESHOLD), "battery_current"
    # method 3: heuristic – throttle released + decelerating
    if throttle is not None and accel is not None:
        return (
            throttle < THROTTLE_OFF_THRESHOLD and accel < DECEL_THRESHOLD,
            "heuristic",
        )
    return False, "unknown"


# main area of program
def run(self):
    self.connect()
    self._open_log()
    profile = self.vehicle_profile

    print("Monitoring for regenerative braking. CTRL+C to kill program.")
    print(
        f"{'Time':12} {'Speed':>9} {'Throttle':>9} {'Torque':>10} {'BatCurr:':>9} {'Accel':>9} {'Regen':>7}"
    )
    print("-" * 78)

    try:
        while True:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:12]

            if self.simulate:
                raw = self._simulate_readings()
                speed = raw["speed_kph"]
                throttle = raw["throttle_pct"]
                torque = raw["torque_nm"]
                current = raw["battery_current_a"]
            else:
                speed = self._query_standard(obd.commands.SPEED)
                throttle = self._query_standard(obd.commands.THROTTLE_POS)
                torque = self._query_custom(profile.get("torque_cmd"))
                current = self._query_custom(profile.get("current_cmd"))

            accel = self._speed_diff.update(speed / 3.6 if speed is not None else 0.0)

            is_regen, method = self._detect_regen(
                speed, throttle, torque, current, accel
            )

            # Session tracking
            self.total_samples += 1
            if is_regen:
                self.regen_samples += 1
                if not self._in_regen:
                    self._in_regen = True
                    self.regen_sessions += 1
                    self._session_start = datetime.now()
            else:
                if self._in_regen:
                    duration = (datetime.now() - self._session_start).total_seconds()
                    print(
                        f"\n  ✅ Regen session #{self.regen_sessions} ended "
                        f"({duration:.1f}s)\n"
                    )
                self._in_regen = False

            def fmt(v, unit=""):
                return f"{v:>7.1f}{unit}" if v is not None else "    n/a "

            regen_flag = "⚡ YES" if is_regen else "  no "
            print(
                f"{ts:12}  {fmt(speed,'kph')}  {fmt(throttle,'%')}  "
                f"{fmt(torque,'Nm')}  {fmt(current,'A')}  "
                f"{fmt(accel,'m/s²')}  {regen_flag}",
                flush=True,
            )

            self._log_row(
                {
                    "timestamp": datetime.now().isoformat(),
                    "speed_kph": speed,
                    "throttle_pct": throttle,
                    "torque_nm": torque,
                    "battery_current_a": current,
                    "acceleration_ms2": round(accel, 3) if accel else None,
                    "regen": int(is_regen),
                    "method": method,
                    "vehicle": self.vehicle_key,
                }
            )

            time.sleep(POLL_INTERVAL_SEC)

    except KeyboardInterrupt:
        pass
    finally:
        self._print_summary()
        self._close_log()
        if self.connection:
            self.connection.close()


# A little summary
def _print_summary(self):
    print("\n" + "=" * 50)
    print("  Session Summary")
    print("=" * 50)
    pct = self.regen_samples / self.total_samples * 100 if self.total_samples else 0
    print(f"  Vehicle         : {self.vehicle_profile['name']}")
    print(f"  Total samples   : {self.total_samples}")
    print(f"  Regen samples   : {self.regen_samples}  ({pct:.1f}%)")
    print(f"  Regen sessions  : {self.regen_sessions}")
    if self.log_path:
        print(f"  Log saved to    : {self.log_path}")
    print("=" * 50)


# entry point to run the program
def main():
    pass


if __name__ == "__main__":
    main()
