#!/usr/bin/env python3

from fs68 import FS68, OutputType, OutputMode, PowerMode


with FS68() as fs:

    fs.buzzer.beep(440, 0.4)
    print("> Beep for shutdown")

    fs.set_power_mode(PowerMode.SHUTDOWN)
    print("> Power Mode set for SHUTDOWN")

    fs.output(OutputType.BUZZER).set(OutputMode.OFF)
    print("> Buzzer DISABLED")
