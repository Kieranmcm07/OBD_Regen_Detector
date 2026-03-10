# obd-regen-detector

> Real-time regenerative braking detector for EVs and hybrids via OBD-II

Connects to your car's OBD-II port through an ELM327 adapter and detects when the vehicle is in **regenerative braking mode** — live, in your terminal. Supports manufacturer-specific PIDs for accurate readings on popular EV/hybrid platforms, with a universal heuristic fallback for any OBD-II vehicle.

---

## Features

- ⚡ **Real-time regen detection** at up to 4 samples/second
- 🚗 **Multi-vehicle support** with manufacturer-specific PIDs
- 📊 **Smart detection cascade** — torque → battery current → heuristic fallback
- 📄 **CSV logging** for post-drive analysis
- 🔧 **Simulation mode** — test without a car or adapter
- 📈 **Session statistics** — regen %, session count, duration

---

## Supported Vehicles

| `--vehicle` flag | Vehicle |
|---|---|
| `generic` | Any OBD-II car (heuristic fallback) |
| `nissan_leaf` | Nissan Leaf Gen 1 & 2 |
| `chevy_bolt` | Chevrolet Bolt EV / Volt |
| `toyota_prius` | Toyota Prius / RAV4 Hybrid |
| `hyundai_ioniq` | Hyundai Ioniq / Kona EV |
| `bmw_i3` | BMW i3 / i8 |

---

## Requirements

**Software**
```
Python 3.10+
pip install obd
```

**Hardware**
- ELM327-compatible OBD-II adapter (Bluetooth, Wi-Fi, or USB)
- Any EV or hybrid vehicle with a standard OBD-II port (1996+)

---

## Installation

```bash
git clone https://github.com/YOUR_USERNAME/obd-regen-detector.git
cd obd-regen-detector
pip install obd
```

---

## Usage

### Test without a car (simulation mode)
```bash
python regen_detector.py --simulate
```

### Generic OBD-II (any car)
```bash
python regen_detector.py
```

### With a specific vehicle profile
```bash
python regen_detector.py --vehicle nissan_leaf
python regen_detector.py --vehicle toyota_prius
python regen_detector.py --vehicle chevy_bolt
```

### Specify serial port manually
```bash
# Linux/macOS
python regen_detector.py --port /dev/ttyUSB0

# Windows
python regen_detector.py --port COM3
```

### Log data to CSV
```bash
python regen_detector.py --vehicle nissan_leaf --log my_drive.csv
```

### List all supported vehicles
```bash
python regen_detector.py --list-vehicles
```

---

## How It Works

Regen is detected using a **priority cascade** — it uses the most accurate method available for your vehicle:

```
1. Negative motor torque  →  motor torque < -5 Nm       [most accurate]
2. Battery charging       →  battery current > 1.0 A    [accurate]
3. Heuristic fallback     →  throttle < 5% + decel      [works on any car]
```

Standard OBD-II does not expose motor torque or battery current as universal PIDs — they are manufacturer-specific. When a vehicle profile is selected (`--vehicle`), the script queries the correct proprietary PIDs. On `generic` mode, the heuristic fallback is used automatically.

---

## Example Output

```
🚗  Vehicle profile : Nissan Leaf (Gen 1 & 2)

🔌  Connecting to OBD-II adapter… OK  (/dev/ttyUSB0)
    Protocol : ISO 15765-4 (CAN 11/500)

▶  Monitoring for regenerative braking. Press Ctrl+C to stop.

Time           Speed   Throttle      Torque    BatCurr      Accel   REGEN
------------------------------------------------------------------------------
09:14:32.1    50.0kph    18.0%     12.0Nm     -2.0A    0.1m/s²     no
09:14:32.3    49.2kph     0.0%    -28.5Nm     17.4A   -2.1m/s²  ⚡ YES
09:14:32.6    46.8kph     0.0%    -31.0Nm     19.2A   -3.0m/s²  ⚡ YES

  ✅ Regen session #1 ended (4.2s)

==================================================
  SESSION SUMMARY
==================================================
  Vehicle         : Nissan Leaf (Gen 1 & 2)
  Total samples   : 240
  Regen samples   : 58  (24.2%)
  Regen sessions  : 7
  Log saved to    : my_drive.csv
==================================================
```

---

## CSV Output Format

When `--log` is used, data is saved in this format:

| Column | Description |
|---|---|
| `timestamp` | ISO 8601 datetime |
| `speed_kph` | Vehicle speed in km/h |
| `throttle_pct` | Throttle position (%) |
| `torque_nm` | Motor torque in Nm (if available) |
| `battery_current_a` | Battery current in amps (if available) |
| `acceleration_ms2` | Calculated acceleration (m/s²) |
| `regen` | `1` = regen active, `0` = not |
| `method` | Detection method used |
| `vehicle` | Vehicle profile key |

---

## Adding a Custom Vehicle

To add support for a new vehicle, define a new decode function and add a profile entry in `VEHICLE_PROFILES` inside `regen_detector.py`:

```python
def _decode_my_car_current(messages):
    d = messages[0].data
    raw = (d[3] << 8 | d[4])
    if raw > 32767:
        raw -= 65536
    return raw * 0.1  # amps

VEHICLE_PROFILES["my_car"] = {
    "name": "My Car Model",
    "description": "Custom PID for my vehicle",
    "torque_cmd": None,
    "current_cmd": OBDCommand(
        "MY_CAR_CURRENT", "My Car Battery Current",
        b"220101", 6, _decode_my_car_current, ECU.ENGINE, True,
    ),
}
```

Pull requests for new vehicle profiles are welcome!

---

## Troubleshooting

**Adapter not detected**
- Make sure the ELM327 adapter is fully seated in the OBD-II port
- Try specifying `--port` manually
- On Linux, you may need: `sudo usermod -aG dialout $USER` then re-login

**All torque/current values show `n/a`**
- Your vehicle may not support those PIDs — the script will fall back to heuristic detection automatically
- Try a different `--vehicle` profile if you're not using `generic`

**Regen not detected on generic mode**
- The heuristic requires both throttle release AND measurable deceleration
- At very low speeds or on flat roads, deceleration may be below the threshold

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

## Contributing

Issues and PRs welcome, especially for new vehicle PID profiles. Please include the make, model, year, PID command bytes, and decode formula.
