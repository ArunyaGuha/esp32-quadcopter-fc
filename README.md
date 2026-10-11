# ESP32 Quadcopter Flight Controller

Custom flight controller firmware for a brushed micro quadcopter, written from scratch on an ESP32. Control runs over Bluetooth LE from a Python keyboard script on a laptop.

## What it does

- Reads a GY-91 (MPU6500) over I2C and estimates pitch/roll with a complementary filter
- Treats the board's start position as level (0, 0) at boot
- Angle-mode PID for self-leveling
- X-quad motor mixer driving 4 brushed motors through MOSFETs with 20 kHz PWM
- Soft-start ramp on motor output to limit current surges (avoids brownouts on a small battery)
- Failsafe: disarms on BLE disconnect or after 500 ms with no packet; disarm cuts motors instantly
- Onboard blue LED is on while armed

## Hardware

| Part | Notes |
|---|---|
| ESP32-WROOM-32 dev board | |
| GY-91 (MPU6500 + BMP280) | SDA = GPIO21, SCL = GPIO22, 3.3 V |
| 4x 8520 coreless motors | each driven by an AO3400 N-MOSFET |
| 1S LiPo + TPS63020 buck-boost | motors on raw battery, ESP32 + IMU on the regulated 3.3 V rail |
| 10 kOhm gate pull-downs | one per MOSFET, prevents motor spin during boot |

Motor pins (top view, make sure each physical motor sits in the matching corner):

```
 M1 GPIO25 (FL)   M2 GPIO26 (FR)
 M3 GPIO27 (RL)   M4 GPIO33 (RR)
```

## Setup

1. Install the **esp32 by Espressif Systems** board package in Arduino IDE. BLE and Wire come with it, no extra libraries.
2. Open `firmware/QuadFC_BLE/QuadFC_BLE.ino`, select **ESP32 Dev Module**, upload.
3. Keep the drone still and flat for the first few seconds after power-on while it calibrates.
4. On the laptop: `pip install bleak keyboard`, then `python controller/ble_controller.py` (Windows: run as Administrator for the `keyboard` library).

## Controls

| Key | Action |
|---|---|
| SPACE | arm / disarm |
| UP / DOWN | throttle |
| I / K | pitch forward / back |
| J / L | roll left / right |
| X | emergency disarm |
| Q | quit (disarms first) |

## Command packet

9 bytes, little-endian, written to characteristic `beb5483e-36e1-4688-b7f5-ea07361b26a8`:

| Field | Type | Range |
|---|---|---|
| arm | uint8 | 0 / 1 |
| throttle | int16 | 0 to 1000 |
| pitch | int16 | -500 to 500 |
| roll | int16 | -500 to 500 |
| yaw | int16 | -500 to 500 (received, not mixed yet) |

Python format string: `'<Bhhhh'`.

## Bring-up checklist (props off first)

1. Boot disarmed: serial shows `M1:0 M2:0 M3:0 M4:0` and `P`/`R` near 0.
2. Arm with throttle 0: motors stay off.
3. Raise throttle: all four ramp up evenly.
4. Tilt the frame by hand while armed: the motors on the side tilting down speed up.
5. Disarm: all four cut instantly.
6. Only then fit props and test restrained.

## Lessons learned

- Gyro calibration needs `int16_t` casts when summing samples, otherwise negative readings turn into ~65000 offsets.
- Motors in the wrong corners cause a steady one-sided drift that no trim or calibration will fix.
- Instant motor spin-up on a small battery can brown out the ESP32; the ramp in `mixMotors()` helps.
- Serial Monitor garbage on some boards is a crystal/baud quirk; try other baud settings before blaming the code.

## Status

Flying. Next up: yaw control, PID tuning on real hardware, battery voltage monitoring.
