#!/usr/bin/env python3

import subprocess
from fs68 import FS68, OutputType, OutputMode, PowerMode


def modprobe(module: str):
    if subprocess.call(["modprobe", module]) != 0:
        print(f"> Failed to load module: {module}")
    else:
        print(f"> Loaded module: {module}")


modprobe("nct7802")


with FS68() as fs:

    version = fs.mcu.get_version()
    print(f"> Detected MCU:PIC16F1829 version: {version}")

    fs.set_power_mode(PowerMode.STARTUP)
    print("> Power Mode set for STARTUP")

    fs.get_output(OutputType.BUZZER).set(OutputMode.ON)
    print("> Buzzer ENABLED")

    brightness = 0.3
    fs.set_led_brightness(brightness)
    print(f"> LED Brightness set to {brightness * 100:.0f}%")
