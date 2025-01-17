#!/usr/bin/env python3

# nohup python3 /root/fs68-scripts/fs68-fand.py >/dev/null 2>&1 &

from logging import getLogger, INFO, Formatter
from logging.handlers import RotatingFileHandler
from pathlib import Path
from time import sleep

from fs68 import FS68, FS68_FAN, FS68_TEMP, TempZone, FanType

POLL_INTERVAL = 10

SYS_TARGET_TEMPERATURE = 60.0
SYS_DELTA_TEMP_THRESHOLD = 1.0

SSD_TARGET_TEMPERATURE = 60.0
SSD_DELTA_TEMP_THRESHOLD = 1.0

LOG_NAME = Path(__file__).parent.resolve().joinpath("fs68-fand.log")
LOG_MAX_BYTES = 10 * 1024 * 1024
LOG_COUNT = 5

log_formatter = Formatter("%(asctime)s.%(msecs)03d [%(levelname)-8s] %(message)s", datefmt="%d-%m-%Y %H:%M:%S")
log_handler = RotatingFileHandler(LOG_NAME, maxBytes=LOG_MAX_BYTES, backupCount=LOG_COUNT)
log_handler.setFormatter(log_formatter)
log = getLogger(__name__)
log.setLevel(INFO)
log.addHandler(log_handler)


class FanControlLoop():
    def __init__(self, name: str, temp: FS68_TEMP, fan: FS68_FAN,
                 target_temperature: float, delta_temp_threshold: float, max_pwm_step: int = 5, min_pwm: int = 50):
        self._name = name
        self._temp = temp
        self._fan = fan
        self._target_temperature = target_temperature
        self._delta_temp_threshold = delta_temp_threshold
        self._max_pwm_step = max_pwm_step
        self._min_pwm = min_pwm
        self._prev_temp_value = 100.0  # ensure first run

    @property
    def name(self):
        return self._name

    @property
    def temp(self):
        return self._temp

    @property
    def fan(self):
        return self._fan

    def map_temp_to_pwm(self, temp: float):
        """
        Map the temperature to the desired PWM value.
        """
        target_temperature = self._target_temperature
        if temp < target_temperature:
            return self._min_pwm

        # get the difference above threshold
        desired_pwm = temp-target_temperature
        # fudge factor the difference
        desired_pwm = desired_pwm*10/18
        # square it
        desired_pwm = desired_pwm*desired_pwm
        # add it to the base_pwm value
        desired_pwm = self._min_pwm+desired_pwm

        # over max - truncate to max
        if desired_pwm > 255:
            desired_pwm = 255

        desired_pwm = int(desired_pwm)
        return desired_pwm

    def run(self):
        curr_temp_value = self.temp.value
        log.info(f"{self.name} TMP: {curr_temp_value:.02f}C")
        # log.info(f"{self.temp}")

        fan = self.fan
        log.info(f"{self.name} FAN: speed={fan.speed} pwm={fan.pwm}")

        desired_pwm = self.map_temp_to_pwm(curr_temp_value)
        log.info(f"{self.name} FAN: Desired fan PWM is {desired_pwm}")

        curr_fan_pwm = fan.pwm
        delta_pwm = abs(curr_fan_pwm - desired_pwm)  # limit step size increase

        if delta_pwm == 0:
            return log.info(f"{self.name} FAN: No change required")

        if desired_pwm > curr_fan_pwm:
            # fan speed increase desired - react immediately
            new_fan_pwm = curr_fan_pwm + min(delta_pwm, self._max_pwm_step)
            fan.pwm = new_fan_pwm
            log.info(f"{self.name} FAN: INCREASE fan PWM from {curr_fan_pwm} to {new_fan_pwm}")
        else:
            # fan speed decrease desired - check for sufficient temperature change
            delta_temp = abs(self._prev_temp_value - curr_temp_value)
            if delta_temp > self._delta_temp_threshold:
                new_fan_pwm = curr_fan_pwm - min(delta_pwm, self._max_pwm_step)
                log.info(f"{self.name} FAN: DECREASE fan PWM from {curr_fan_pwm} to {new_fan_pwm}")
                fan.pwm = new_fan_pwm
            else:
                prev = f"{self._prev_temp_value:.02f}"
                log.info(f"{self.name} FAN: INSUFFICIENT delta of {delta_temp:.02f} to reduce fan speed, prev={prev}")
                return  # do not preserve temperature

        self._prev_temp_value = curr_temp_value


def main():
    # Wait 1 poll interval before starting to avoid conflicts with other post-init scripts
    sleep(POLL_INTERVAL)

    with FS68() as fs:
        fs.probe_devices()  # first probe

        sys_ctrl = FanControlLoop("SYS",
                                  fs.get_zone_temp([TempZone.CPU, TempZone.SYSTEM, TempZone.PHY]),
                                  fs.get_fan(FanType.CPU),
                                  target_temperature=SYS_TARGET_TEMPERATURE,
                                  delta_temp_threshold=SYS_DELTA_TEMP_THRESHOLD,
                                  )

        ssd_ctrl = FanControlLoop("SSD",
                                  fs.get_zone_temp(TempZone.NVME),
                                  fs.get_fan(FanType.STORAGE),
                                  target_temperature=SSD_TARGET_TEMPERATURE,
                                  delta_temp_threshold=SSD_DELTA_TEMP_THRESHOLD,
                                  )

        while (True):
            fs.probe_devices()

            for control_loop in [sys_ctrl, ssd_ctrl]:
                control_loop.run()

            sleep(POLL_INTERVAL)


main()
