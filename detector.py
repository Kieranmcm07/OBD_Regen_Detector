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
    print("Error: 'python-OBD' library not found. Please install it with 'pip install obd'")
    sys.exit(1)
    
    
    

# Configuration (make sure that the PIDs you want to use are supported by your car's ECU)
POLL_INTERVAL_SEC = 0.25        # How often to query the OBD adapter (seconds)
REGEN_TORQUE_THRESHOLD = -5     # Nm  – torque below this = regen (motor braking)
REGEN_CURRENT_THRESHOLD = 1.0   # A   – battery current above this = charging = regen
DECEL_THRESHOLD = -0.5          # m/s² – heuristic: deceleration stronger than this
THROTTLE_OFF_THRESHOLD = 5      # %   – throttle below this = foot off pedal