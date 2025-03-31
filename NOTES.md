Scratchpad
```
fs.output(OutputType.BUZZER).set(OutputMode.OFF)
fs.output(OutputType.SYSTEM_STATUS_RED).set(OutputMode.OFF)
fs.output(OutputType.SYSTEM_POWER_BLUE).set(OutputMode.ON)
fs.output(OutputType.SYSTEM_STATUS_GREEN).set(OutputMode.ON)
fs.output(OutputType.SYSTEM_STATUS_GREEN).set(OutputMode.OFF)
fs.output(OutputType.SYSTEM_STATUS_RED).set(OutputMode.FLASH_2HZ)

fs.output(OutputType.NETWORK_STATUS).set(OutputMode.ON)
fs.output(OutputType.STORAGE_STATUS_GREEN).set(OutputMode.OFF)
fs.output(OutputType.STORAGE_STATUS_RED).set(OutputMode.OFF)
fs.output(OutputType.SIDE_RED_INNER).set(OutputMode.OFF)
fs.output(OutputType.SIDE_RED_MID).set(OutputMode.OFF)
fs.output(OutputType.SIDE_RED_OUTER).set(OutputMode.OFF)

fs.set_led_brightness(100)

fs.buzzer.beep(440, 0.2)

fs.hwmon.probe()
for key in sorted(fs.hwmon.devices.keys()):
    print(str(fs.hwmon.devices[key]))

fan_cpu = fs.get_fan(FanType.CPU)
print(f"fan_cpu:     {fan_cpu}")
fan_cpu.pwm = 60
fan_cpu.pwm -= 30

fan_storage = fs.get_fan(FanType.STORAGE)
print(f"fan_storage: {fan_storage}")
fan_storage.pwm = 60
fan_storage.pwm -= 30

zone_cpu = fs.get_zone_temp(TempZone.CPU)
print(f"zone_cpu:  {zone_cpu.value}")

zone_sys = fs.get_zone_temp(TempZone.SYSTEM)
print(f"zone_sys:  {zone_sys.value}")

zone_phy = fs.get_zone_temp(TempZone.PHY)
print(f"zone_phy:  {zone_phy.value}")

zone_nvme = fs.get_zone_temp(TempZone.NVME)
print(f"zone_nvme: {zone_nvme.value}")

```