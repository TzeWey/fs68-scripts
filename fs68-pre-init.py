#!/usr/bin/env python3

from fs68 import FS68, OutputType, OutputMode, PowerMode


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
