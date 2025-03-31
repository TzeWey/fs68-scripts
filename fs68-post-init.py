#!/usr/bin/env python3

from fs68 import FS68, OutputType, OutputMode


with FS68() as fs:

    fs.output(OutputType.SYSTEM_STATUS_GREEN).set(OutputMode.ON)
    print("> SYSTEM_STATUS_GREEN set to ON")

    fs.buzzer.beep(440, 0.2)
    print("> Beep for post-init")
