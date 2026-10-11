"""Keyboard BLE controller for the ESP32 quadcopter.

Controls
    SPACE      arm / disarm
    UP / DOWN  throttle up / down
    I / K      pitch forward / back
    J / L      roll left / right
    X          emergency disarm (throttle 0)
    Q          quit (disarms first)

Install:  pip install bleak keyboard
Run:      python ble_controller.py        (Windows: run the terminal as Administrator)
"""
import asyncio
import struct

import keyboard
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "ESP32-Quad-Control"
CONTROL_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

SEND_HZ = 50
THROTTLE_STEP = 10
STICK_STEP = 20
MAX_STICK = 500
MAX_THROTTLE = 1000


async def main():
    print(f"Scanning for {DEVICE_NAME} ...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15)
    if device is None:
        print("Device not found. Is the ESP32 powered and advertising?")
        return

    arm = 0
    throttle = 0
    pitch = 0
    roll = 0
    yaw = 0
    armed_edge = False

    async with BleakClient(device) as client:
        print("Connected. SPACE arm/disarm, UP/DOWN throttle, I/K pitch, J/L roll, X e-stop, Q quit.")
        while True:
            if keyboard.is_pressed("q"):
                break

            if keyboard.is_pressed("x"):
                arm, throttle = 0, 0

            if keyboard.is_pressed("space"):
                if not armed_edge:
                    arm = 1 - arm
                    if arm == 0:
                        throttle = 0
                    armed_edge = True
            else:
                armed_edge = False

            if keyboard.is_pressed("up"):
                throttle = min(throttle + THROTTLE_STEP, MAX_THROTTLE)
            if keyboard.is_pressed("down"):
                throttle = max(throttle - THROTTLE_STEP, 0)

            if keyboard.is_pressed("i"):
                pitch = min(pitch + STICK_STEP, MAX_STICK)
            elif keyboard.is_pressed("k"):
                pitch = max(pitch - STICK_STEP, -MAX_STICK)
            else:
                pitch = int(pitch * 0.8)

            if keyboard.is_pressed("l"):
                roll = min(roll + STICK_STEP, MAX_STICK)
            elif keyboard.is_pressed("j"):
                roll = max(roll - STICK_STEP, -MAX_STICK)
            else:
                roll = int(roll * 0.8)

            packet = struct.pack("<Bhhhh", arm, throttle, pitch, roll, yaw)
            await client.write_gatt_char(CONTROL_CHAR_UUID, packet, response=False)

            print(f"Arm:{arm} Thr:{throttle:4d} Pitch:{pitch:4d} Roll:{roll:4d}   ", end="\r")
            await asyncio.sleep(1 / SEND_HZ)

        # leave the drone disarmed
        await client.write_gatt_char(
            CONTROL_CHAR_UUID, struct.pack("<Bhhhh", 0, 0, 0, 0, 0), response=False
        )
        print("\nDisarmed and disconnected.")


if __name__ == "__main__":
    asyncio.run(main())
