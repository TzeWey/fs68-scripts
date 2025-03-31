
import os
import errno
import sys
import stat
import struct
import termios
import mmap
import resource
import re

from abc import ABC
from enum import IntEnum, auto
from select import select
from time import sleep, monotonic
from typing import List, Dict, AnyStr, Union
from pathlib import Path
from logging import getLogger

log = getLogger(__name__)


MEM_ADDRESS_LED_BRIGHTNESS_PWM = 0xFED80403


################################################################################
# Serial
################################################################################
#
# backend for serial IO for POSIX compatible systems, like Linux, OSX
#
# This file is part of pySerial. https://github.com/pyserial/pyserial
# (C) 2001-2020 Chris Liechti <cliechti@gmx.net>
#
# SPDX-License-Identifier:    BSD-3-Clause
#
# parts based on code from Grant B. Edwards  <grante@visi.com>:
#  ftp://ftp.visi.com/users/grante/python/PosixSerial.py
#
# references: http://www.easysw.com/~mike/serial/serial.html
#
# This serial implementation is the minimum needed to work on TrueNAS @115200n8
#
class Serial():
    """
    Simple Serial port class.
    Fixed to 8-bits, 1-stop bit, no parity, no flow control
    """

    def __init__(self, port=None, baudrate=115200, timeout=None):
        self.is_open = False

        # correct values are assigned below through properties
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout

        if self._port is None:
            raise Exception("Port must be configured.")

        if port is not None:
            self.open()

    @property
    def closed(self):
        return not self.is_open

    def _reconfigure_port(self, force_update=False):
        if self.fd is None:
            raise Exception("Invalid file descriptor")

        try:
            orig_attr = termios.tcgetattr(self.fd)
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = orig_attr
        except termios.error as msg:
            raise Exception(f"Could not configure port: {msg}")

        ispeed = ospeed = getattr(termios, f"B{self._baudrate}")  # setup baud rate

        # set up raw mode / no echo / binary
        cflag |= (termios.CLOCAL | termios.CREAD)
        lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE |
                   termios.ECHOK | termios.ECHONL | termios.ISIG | termios.IEXTEN)
        oflag &= ~(termios.OPOST | termios.ONLCR | termios.OCRNL)
        iflag &= ~(termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IGNBRK)

        cflag &= ~termios.CSIZE
        cflag |= termios.CS8  # 8 bits
        cflag &= ~(termios.CSTOPB)  # 1 stop bit
        iflag &= ~(termios.INPCK | termios.ISTRIP)  # no parity
        cflag &= ~(termios.PARENB | termios.PARODD)
        iflag &= ~(termios.IXON | termios.IXOFF)  # disable flow control
        cflag &= ~(termios.CRTSCTS)  # rtscts - disable

        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0

        # activate settings
        if force_update or [iflag, oflag, cflag, lflag, ispeed, ospeed, cc] != orig_attr:
            termios.tcsetattr(self.fd, termios.TCSANOW,
                              [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

    def _reset_input_buffer(self):
        """Clear input buffer, discarding all that is in the buffer."""
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def open(self):
        if self.is_open:
            raise Exception("Port is already open.")
        self.fd = None
        try:
            self.fd = os.open(self._port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as msg:
            self.fd = None
            raise Exception(msg.errno, "Could not open port {}: {}".format(self._port, msg))

        try:
            self._reconfigure_port(force_update=True)
            self._reset_input_buffer()
        except BaseException:
            try:
                os.close(self.fd)
            except Exception:
                # ignore any exception when closing the port
                # also to keep original exception that happened when setting up
                pass
            self.fd = None
            raise

        self.is_open = True

    def close(self):
        """Close port"""
        if self.is_open:
            if self.fd is not None:
                os.close(self.fd)
                self.fd = None
            self.is_open = False

    def read(self, size=1):
        """
        Read size bytes from the serial port. If a timeout is set it may
        return less characters as requested. With no timeout it will block
        until the requested number of bytes is read.
        """
        if not self.is_open:
            raise Exception("Port not open")
        read = bytearray()

        def time_left():
            if timeout_time is None:
                return None
            return timeout_time - monotonic()

        timeout_time = monotonic() + self._timeout if self._timeout is not None else None

        while len(read) < size:
            try:
                ready, _, _ = select([self.fd], [], [], time_left())
                if not ready:
                    break  # timeout
                buf = os.read(self.fd, size - len(read))
            except OSError as e:
                if e.errno not in (errno.EAGAIN, errno.EALREADY, errno.EWOULDBLOCK, errno.EINPROGRESS, errno.EINTR):
                    raise Exception(f"Read failed: {e}")
            else:
                if not buf:
                    raise Exception("Read failed, buffer is empty")
                read.extend(buf)

            if (timeout_time is not None) and (time_left() <= 0):
                break
        return bytes(read)

    def write(self, data):
        """Output the given byte string over the serial port."""
        if not self.is_open:
            raise Exception("Port not open")

        def to_bytes(seq):
            """Convert a sequence to a bytes type"""
            if isinstance(seq, bytes):
                return seq
            elif isinstance(seq, bytearray):
                return bytes(seq)
            else:
                raise TypeError(f"Only bytes allowed: {seq}")

        d = to_bytes(data)
        tx_len = length = len(d)
        while tx_len > 0:
            try:
                return os.write(self.fd, d)
            except Exception:
                raise
            except OSError as e:
                if e.errno not in (errno.EAGAIN, errno.EALREADY, errno.EWOULDBLOCK, errno.EINPROGRESS, errno.EINTR):
                    raise Exception(f"Write failed: {e}")
        return length - len(d)


################################################################################
# Types
################################################################################
class OutputType(IntEnum):
    BUZZER = auto()
    NETWORK_STATUS = auto()
    STORAGE_STATUS_GREEN = auto()
    STORAGE_STATUS_RED = auto()
    SIDE_RED_INNER = auto()
    SIDE_RED_MID = auto()
    SIDE_RED_OUTER = auto()
    SYSTEM_POWER_BLUE = 0x98
    SYSTEM_STATUS_GREEN = 0x70
    SYSTEM_STATUS_RED = 0x78

    @classmethod
    def is_valid(cls, value):
        return value in iter(cls)

    @classmethod
    def is_valid_for_mcu(cls, value):
        return value in [cls.SYSTEM_POWER_BLUE, cls.SYSTEM_STATUS_GREEN, cls.SYSTEM_STATUS_RED]


class OutputMode(IntEnum):
    OFF = 0
    ON = 3
    FLASH_2HZ = 4
    FLASH_4HZ = 5
    FLASH_8HZ = 6
    FLASH_16HZ = 7

    @classmethod
    def is_valid(cls, value):
        return value in iter(cls)

    @classmethod
    def is_valid_for_mcu(cls, value):
        return value in iter(cls)


class TempZone(IntEnum):
    CPU = 0
    SYSTEM = auto()
    PHY = auto()
    NVME = auto()

    @classmethod
    def is_valid(cls, value):
        return value in iter(cls)


class FanType(IntEnum):
    CPU = auto()
    STORAGE = auto()

    @classmethod
    def is_valid(cls, value):
        return value in iter(cls)


# TODO: find out how this works, the bits seem to mean something.
# De-compiling libgeneraldrv.so seems to indicate:
# - bit0 is something called "eup"
# - bit1 is "wol"
# - bit2 is "alarm"
# - bit3 is "rtc"
# - bit4 is "sleep"
# - bit6:7 is some kind of "resume mode"
class PowerMode(IntEnum):
    STARTUP = 0xA0
    SHUTDOWN = 0x20
    NIGHT = 0x1A

    @classmethod
    def is_valid(cls, value):
        return value in iter(cls)


################################################################################
# Helper Functions
################################################################################
def readline(path: str):
    if os.path.exists(path):
        with open(path, "r") as f:
            return f.readline().strip()
    return ""


def writeline(path: str, line: Union[str, int]):
    if os.path.exists(path):
        with open(path, "w") as f:
            f.write(str(line))
            f.flush()


def to_hex(bytes: bytearray):
    if bytes is None:
        return ""
    return " ".join([f"{x:02X}" for x in bytes])


def assert_u8(value: int, name: str):
    assert (value & 0xFF) == value, f"'{name}' [{value}] must be between 0 and 255 inclusive"


def pr_dbg(*args, **kwargs):
    log.debug(*args, file=sys.stderr, **kwargs)


def pr_err(*args, **kwargs):
    log.error(*args, file=sys.stderr, **kwargs)


################################################################################
# FS68 Base Classes
################################################################################
class FS68_OUTPUT(ABC):
    def set(self, mode: OutputMode):
        raise NotImplementedError()


class FS68_TEMP(ABC):
    @property
    def name(self) -> str:
        raise NotImplementedError()

    @property
    def description(self) -> str:
        raise NotImplementedError()

    @property
    def value(self) -> float:
        raise NotImplementedError()

    def __str__(self):
        return f"{self.value:.02f} "


class FS68_FAN(ABC):
    @property
    def device(self) -> str:
        raise NotImplementedError()

    @property
    def speed(self) -> int:
        raise NotImplementedError()

    @property
    def pwm(self) -> int:
        raise NotImplementedError()

    @pwm.setter
    def pwm(self, value: int):
        raise NotImplementedError()

    def __str__(self):
        return f"speed={self.speed}, pwm={self.pwm}"


################################################################################
# FS68 Memtool
################################################################################
class FS68_Memtool(object):
    def __init__(self):
        self.pagesize = resource.getpagesize()
        self.file = "/dev/mem"

    def __get_map_offsets(self, offset: int, size: int):
        map_start = offset & ~(self.pagesize - 1)
        map_offset = offset - map_start
        map_size = map_offset + size
        return map_start, map_size, map_offset

    def read_u8(self, offset: int):
        mem_file = os.open(self.file, os.O_RDONLY, stat.S_IRUSR | stat.S_IWUSR)
        map_start, map_size, map_offset = self.__get_map_offsets(offset, 1)
        with mmap.mmap(mem_file, map_size, mmap.MAP_SHARED, mmap.PROT_READ, 0, map_start) as map:
            return map[map_offset]

    def write_u8(self, offset: int, value: int):
        mem_file = os.open(self.file, os.O_RDWR | os.O_CREAT,
                           stat.S_IRUSR | stat.S_IWUSR)
        map_start, map_size, map_offset = self.__get_map_offsets(offset, 1)
        with mmap.mmap(mem_file, map_size, mmap.MAP_SHARED, mmap.PROT_WRITE, 0, map_start) as map:
            map[map_offset] = (value & 0xFF)


################################################################################
# FS68 GPIO
################################################################################
class FS68_GpioPin(object):
    GPIO_ROOT = "/sys/class/gpio"
    GPIO_EXPORT = os.path.join(GPIO_ROOT, "export")
    GPIO_UNEXPORT = os.path.join(GPIO_ROOT, "unexport")

    def __init__(self, index: int, active_low: bool = False):
        self.__index = index
        self.__name = f"gpio{index}"
        self.__active_low = active_low
        self.__is_open = False

    def __export(self):
        if self.__is_open:
            return
        if not os.path.exists(os.path.join(self.GPIO_ROOT, self.__name)):
            writeline(self.GPIO_EXPORT, self.__index)
        self.__is_open = True

    def __unexport(self):
        if not self.__is_open:
            return
        if os.path.exists(os.path.join(self.GPIO_ROOT, self.__name)):
            writeline(self.GPIO_UNEXPORT, self.__index)
        self.__is_open = False

    def __set_direction(self, direction: str):
        assert direction in ["in", "out"]
        assert os.path.exists(os.path.join(self.GPIO_ROOT, self.__name))
        gpio_path = os.path.join(self.GPIO_ROOT, self.__name, "direction")
        writeline(gpio_path, direction)

    def __set_active_low(self, value: bool):
        assert os.path.exists(os.path.join(self.GPIO_ROOT, self.__name))
        gpio_path = os.path.join(self.GPIO_ROOT, self.__name, "active_low")
        writeline(gpio_path, "1" if value else "0")

    def __set_value(self, value: bool):
        assert os.path.exists(os.path.join(self.GPIO_ROOT, self.__name))
        gpio_path = os.path.join(self.GPIO_ROOT, self.__name, "value")
        writeline(gpio_path, "1" if value else "0")

    def close(self):
        self.__unexport()

    def set(self, enable: bool):
        self.__export()
        self.__set_direction("out")
        self.__set_active_low(self.__active_low)
        self.__set_value(enable)


class FS68_Gpio(object):
    def __init__(self):
        self.__pins: Dict[OutputType, FS68_GpioPin] = {
            (OutputType.BUZZER):  FS68_GpioPin(0x250),
            (OutputType.NETWORK_STATUS): FS68_GpioPin(0x294, active_low=True),
            (OutputType.STORAGE_STATUS_GREEN): FS68_GpioPin(0x293),
            (OutputType.STORAGE_STATUS_RED): FS68_GpioPin(0x244, active_low=True),
            (OutputType.SIDE_RED_INNER): FS68_GpioPin(0x24B, active_low=True),
            (OutputType.SIDE_RED_MID): FS68_GpioPin(0x24F, active_low=True),
            (OutputType.SIDE_RED_OUTER): FS68_GpioPin(0x26A, active_low=True),
        }

    def close(self):
        for key in self.__pins:
            self.__pins[key].close()

    def get_pin(self, type: OutputType):
        return self.__pins[type]


class FS68_GpioOutput(FS68_OUTPUT):
    def __init__(self, pin: FS68_GpioPin):
        assert isinstance(pin, FS68_GpioPin), f"Invalid GPIO Pin: {pin}"
        self.__pin = pin

    def set(self, mode: OutputMode):
        if mode not in [OutputMode.ON, OutputMode.OFF]:
            log.warning("System GPIO only supports 'ON' and 'OFF', flashing would need to be implemented manually")
        self.__pin.set(mode != OutputMode.OFF)


################################################################################
# FS68 Buzzer
################################################################################
class FS68_Buzzer(object):
    PCSPKR_PATH = "/dev/input/by-path/platform-pcspkr-event-spkr"

    def beep(self, frequency: int, duration: float):
        with open(self.PCSPKR_PATH, "wb") as f:
            f.write(struct.pack("<QQHHL", 0, 0, 0x12, 0x2, frequency))
            f.flush()
            sleep(duration)
            f.write(struct.pack("<QQHHL", 0, 0, 0x12, 0x2, 0))
            f.flush()


################################################################################
# FS68 MCU
################################################################################
class FS68_Mcu(object):

    def __init__(self):
        self.port = None
        self.is_open = False

    def open(self, timeout=5.0):
        if self.is_open:
            return
        self.port = Serial("/dev/ttyS1", baudrate=115200, timeout=timeout)
        self.is_open = True

    def close(self):
        if self.port and self.port.is_open:
            self.port.close()
        self.is_open = False

    def send_command(self, command: bytearray, response_count: int = 0):
        self.open()
        self.port.write(command)
        pr_dbg(f"TX > {to_hex(command)}")
        response = self.port.read(len(command) + response_count)
        pr_dbg(f"RX < {to_hex(response)}")
        if response[0] != ((command[0] & 0xF0) | 0x0A):
            pr_err(f"unexpected response header: {to_hex(response)}")
        return response[len(command):]

    def get_version(self):
        return self.send_command(bytearray([0x41, 0x00, 0x00]), 3)[2]

    def get_fan_pwm(self):
        return self.send_command(bytearray([0x31, 0x00, 0x00]), 3)[2]

    def set_fan_pwm(self, value: int):
        assert_u8(value, "Fan PWM")
        return self.send_command(bytearray([0x30, 0x00, value & 0xFF]))

    def get_fan_speed(self):
        speed_lsb = self.send_command(bytearray([0x31, 0x10, 0x00]), 3)[2]
        speed_msb = self.send_command(bytearray([0x31, 0x11, 0x00]), 3)[2]
        return (speed_msb << 8) | (speed_lsb)

    def set_led_mode(self, led: OutputType, mode: OutputMode):
        assert OutputType.is_valid_for_mcu(led), f"Unsupported LED Type: {led}"
        assert OutputMode.is_valid_for_mcu(mode), f"Unsupported LED Mode: {mode}"
        value = ((led + mode) & 0xFF)
        return self.send_command(bytearray([0x10, 0x02, value]))

    def get_power_mode(self):
        response = self.send_command(bytearray([0x11, 0x00, 0x00]), 3)
        return response

    def set_power_mode(self, mode: PowerMode):
        assert PowerMode.is_valid(mode), f"Unsupported Power Mode: {mode}"
        return self.send_command(bytearray([0x10, 0x00, mode]))


class FS68_McuFan(FS68_FAN):
    def __init__(self, mcu: FS68_Mcu):
        self.__mcu = mcu

    @property
    def device(self):
        return "mcu"

    @property
    def speed(self):
        return self.__mcu.get_fan_speed()

    @property
    def pwm(self):
        return self.__mcu.get_fan_pwm()

    @pwm.setter
    def pwm(self, value: int):
        self.__mcu.set_fan_pwm(value)


class FS68_McuOutput(FS68_OUTPUT):
    def __init__(self, mcu: FS68_Mcu, type: OutputType):
        assert OutputType.is_valid_for_mcu(
            type), f"Unsupported Output Type: {type}"
        self.__mcu = mcu
        self.__type = type

    def set(self, mode: OutputMode):
        assert OutputMode.is_valid_for_mcu(
            mode), f"Unsupported Output Mode: {mode}"
        self.__mcu.set_led_mode(self.__type, mode)


################################################################################
# FS68 HWMON
################################################################################
class FS68_Hwmon_Device(object):
    HWMON_ROOT = "/sys/class/hwmon"

    def __init__(self, node_path: str):
        assert os.path.isdir(
            node_path), f"Path '{node_path}' is not a directory"

        self.__node_path = node_path
        self.__device_path = self.__get_device_path()
        self.__type = self.__get_type()
        self.__name = self.__get_name()
        self.__description = self.__get_description()
        self.__temps: Dict[AnyStr, "FS68_Hwmon_TempSensor"] = {}
        self.__temps_probed = False
        self.__fans: Dict[AnyStr, "FS68_Hwmon_FanSensor"] = {}
        self.__fans_probed = False

    def __get_device_path(self):
        # get symlink source of hwmon device
        return os.path.realpath(self.node_path)

    def __get_type(self):
        return FS68_Hwmon_Device.get_type(self.node_path)

    def __get_name(self):
        if self.type == "nvme":
            return os.path.basename(os.path.dirname(self.device_path))
        if self.type == "phy":
            # use the XGBE device function as the phy index as it should be consistent
            xgbe_function = str(Path(self.device_path).parents[2])[-1:]
            return f"phy{xgbe_function}"
        return self.type  # Default to 'type'

    def __get_description(self):
        if self.type == "nvme":
            parent = os.path.dirname(self.device_path)
            model = readline(os.path.join(parent, "model"))
            serial = readline(os.path.join(parent, "serial"))
            if model and serial:
                return f"{model} [{serial}]"
            if model:
                return model
            return "NVMe Device"
        if self.type == "phy":
            mdio_bus = Path(self.device_path).parents[1]
            uevent_path = os.path.join(mdio_bus, "uevent")
            with open(uevent_path, "r") as f:
                while True:
                    line = f.readline()
                    if not line:
                        break
                    matches = re.findall("DRIVER=(.*)", line)
                    if matches:
                        return matches[0]
            return "PHY Device"
        if self.type == "k10temp":
            return "AMD CPU"
        if self.type == "nct7802":
            return "Nuvoton NCT7802"
        return os.path.basename(self.node_path)

    def __probe_temps(self):
        if self.__temps_probed:
            return
        self.__temps_probed = True
        temps = FS68_Hwmon_TempSensor.probe(self, self.node_path)
        for temp in temps:
            self.__temps[temp.node] = temp

    def __probe_fans(self):
        # skip if already probed
        if self.__fans_probed:
            return
        self.__fans_probed = True

        # only "nct7802" has 1 fan at fan1 and pwm1
        if self.type != "nct7802":
            return

        fan = FS68_Hwmon_FanSensor(self, 1, 1)
        self.__fans[fan.node] = fan

    @property
    def node(self):
        return os.path.basename(self.__node_path)

    @property
    def node_path(self):
        return self.__node_path

    @property
    def device_path(self):
        return self.__device_path

    @property
    def type(self):
        return self.__type

    @property
    def name(self):
        return self.__name

    @property
    def description(self):
        return self.__description

    @property
    def temps(self):
        self.__probe_temps()
        return self.__temps

    @property
    def fans(self):
        self.__probe_fans()
        return self.__fans

    def __str__(self):
        response = ""
        response += f" {self.node} [type={self.type},name={self.name}]\n"
        response += f"  node_path={self.node_path}\n"
        response += f"  device_path={self.device_path}\n"
        response += f"  description={self.description}\n"
        response += f"  temps[{len(self.temps)}]\n"
        for key in sorted(self.temps.keys()):
            response += f"   {self.temps[key]}\n"
        if len(self.fans) > 0:
            response += f"  fans[{len(self.fans)}]\n"
            for key in sorted(self.fans.keys()):
                response += f"   {self.fans[key]}\n"
        return response

    @classmethod
    def get_type(cls, node_path: str):
        # assume 'phy' if the path contains 'mdio_bus'
        if "mdio_bus" in os.path.realpath(node_path):
            return "phy"
        return readline(os.path.join(node_path, "name"))


class FS68_Hwmon_FanSensor(FS68_FAN):
    def __init__(self, parent: FS68_Hwmon_Device, fan_index: int, pwm_index: int):
        self.__parent = parent
        self.__fan_name = f"fan{fan_index}"
        self.__pwm_name = f"pwm{pwm_index}"

    @property
    def node(self):
        return self.__fan_name

    @property
    def device(self):
        return self.__parent.name

    @property
    def pwm(self):
        path = os.path.join(self.__parent.node_path, self.__pwm_name)
        return int(readline(path))

    @pwm.setter
    def pwm(self, value: int):
        assert_u8(value, "PWM")
        path = os.path.join(self.__parent.node_path, self.__pwm_name)
        writeline(path, str(value))

    @property
    def speed(self):
        path = os.path.join(self.__parent.node_path, f"{self.__fan_name}_input")
        return int(readline(path))


class FS68_Hwmon_TempSensor(FS68_TEMP):
    def __init__(self, parent: FS68_Hwmon_Device, index: int):
        self.__parent = parent
        self.__index = index

    def __to_temp_c(self, input: str):
        return float(int(input)/1000)

    @property
    def index(self):
        return self.__index

    @property
    def device(self):
        return self.__parent.name

    @property
    def node(self):
        return f"temp{self.index}"

    @property
    def name(self):
        label = readline(os.path.join(self.__parent.node_path, f"{self.node}_label"))
        return label if len(label) else self.node

    @property
    def value(self):
        input = readline(os.path.join(self.__parent.node_path, f"{self.node}_input"))
        return self.__to_temp_c(input)

    @property
    def description(self):
        return self.__parent.description

    def __str__(self):
        return f"{self.value:.02f}  {self.device:8s} {self.name:10s} {self.description}"

    @classmethod
    def probe(cls, parent: FS68_Hwmon_Device, node_path: str) -> List["FS68_Hwmon_TempSensor"]:
        assert os.path.isdir(
            node_path), f"Path '{node_path}' is not a directory"
        # find all temp sensors
        sensors = []
        re_temp = re.compile(r"^temp(\d+)_input$")
        for node_name in os.listdir(node_path):
            m = re_temp.match(node_name)
            if not m:
                continue
            index = int(m.group(1))
            sensors.append(cls(parent, index))
        return sensors


class FS68_Hwmon(object):
    HWMON_ROOT = "/sys/class/hwmon"
    HWMON_TYPE_FILTER = ["nvme", "k10temp", "nct7802", "phy"]

    def __init__(self):
        self.__devices: Dict[str, FS68_Hwmon_Device] = {}

    def probe(self):
        for node_name in os.listdir(self.HWMON_ROOT):
            # skip if already probed
            if node_name in self.__devices:
                continue

            node_path = os.path.join(self.HWMON_ROOT, node_name)
            node_type = FS68_Hwmon_Device.get_type(node_path)

            # skip if is not the 'type' we are interested in
            if node_type not in self.HWMON_TYPE_FILTER:
                continue

            device = FS68_Hwmon_Device(node_path)
            self.__devices[device.name] = device

    def get_fans(self, device_name: str, fan_name: str = None):
        if device_name not in self.devices:
            return []
        device = self.devices[device_name]
        if fan_name is None:
            return list(device.fans.values())
        if fan_name not in device.fans:
            return []
        return [device.fans[fan_name]]

    def get_fan(self, device_name: str, fan_name: str):
        fans = self.get_fans(device_name, fan_name)
        return fans[0] if len(fans) > 0 else None

    def get_temps(self, device_name: str, temp_name: str = None):
        if device_name not in self.devices:
            return []
        device = self.devices[device_name]
        if temp_name is None:
            return list(device.temps.values())
        if temp_name not in device.temps:
            return []
        return [device.temps[temp_name]]

    def get_temp(self, device_name: str, temp_name: str):
        temps = self.get_temps(device_name, temp_name)
        return temps[0] if len(temps) > 0 else None

    def get_temps_by_type(self, device_type: str, temp_name: str = None):
        devices = [x for x in self.devices.values() if x.type == device_type]
        if len(devices) == 0:
            return []
        if temp_name is None:
            temps = []
            for device in devices:
                temps.extend(list(device.temps.values()))
            return temps
        return []

    @property
    def devices(self):
        return self.__devices


################################################################################
# FS68 Temp Aggregator
################################################################################
class FS68_TempAggregator(FS68_TEMP):
    def __init__(self, sensor: List[FS68_Hwmon_TempSensor], name: str = "Temp Aggregator"):
        assert isinstance(sensor, list), f"'temps' must be a list: {sensor}"
        self.__sensors = sensor
        self.__name = name
        self.__max_sensor = None

        # Sort sensor list for display
        self.__sensors.sort(key=lambda x: (x.device, x.name))

    @property
    def name(self):
        return self.__name

    @property
    def device(self):
        raise self.__name

    @property
    def value(self):
        """
        Calculate the maximum of all temperatures
        """
        if self.__sensors is None or len(self.__sensors) == 0:
            return None
        max_value = 0
        max_sensor = None
        for sensor in self.__sensors:
            value = sensor.value
            if value > max_value:
                max_value = value
                max_sensor = sensor
        self.__max_sensor = max_sensor
        return max_value

    @property
    def max_sensor(self):
        return self.__max_sensor

    @property
    def sensors(self):
        return self.__sensors

    def __str__(self):
        response = []
        for i in range(0, len(self.sensors)):
            sensor = self.sensors[i]
            marker = "*" if self.max_sensor is sensor else " "
            response.append(f" {marker}{sensor}")
        return "\n".join(response)


################################################################################
# FS68 Main Class
################################################################################
class FS68(object):
    def __init__(self):
        self.is_open = False
        self.mem = FS68_Memtool()
        self.hwmon = FS68_Hwmon()
        self.mcu = FS68_Mcu()
        self.gpio = FS68_Gpio()
        self.buzzer = FS68_Buzzer()

    def open(self):
        self.is_open = True

    def close(self):
        self.gpio.close()
        self.mcu.close()
        self.is_open = False

    def probe_devices(self):
        self.hwmon.probe()

    def set_led_brightness(self, brightness: float):
        value = int(brightness * 255) & 0xFF
        assert_u8(value, "LED Brightness")
        self.mem.write_u8(MEM_ADDRESS_LED_BRIGHTNESS_PWM, value)

    def set_power_mode(self, mode: PowerMode):
        self.mcu.set_power_mode(mode)

    def output(self, ledType: OutputType) -> Union[FS68_OUTPUT, None]:
        assert OutputType.is_valid(ledType), f"Unsupported LEDType: {ledType}"
        if OutputType.is_valid_for_mcu(ledType):
            return FS68_McuOutput(self.mcu, ledType)
        return FS68_GpioOutput(self.gpio.get_pin(ledType))

    def get_fan(self, type: FanType) -> Union[FS68_FAN, None]:
        assert FanType.is_valid(type), f"Unsupported FanType: {type}"
        if type == FanType.CPU:
            return self.hwmon.get_fan("nct7802", "fan1")
        if type == FanType.STORAGE:
            return FS68_McuFan(self.mcu)
        return None

    def get_zone_temp(self, zones: TempZone | List[TempZone]):
        """
        Returns an instance of FS68_TempAggregator
        """
        if not isinstance(zones, list):
            zones = [zones]  # make into list

        for zone in zones:
            assert TempZone.is_valid(zone), f"Unsupported TempZone: {zone}"

        temps = []
        names = []

        def append_temps_list(t: FS68_TEMP | List[FS68_TEMP]):
            temps.extend(t) if isinstance(t, list) else temps.append(t)

        for zone in zones:
            if zone == TempZone.CPU:
                append_temps_list(self.hwmon.get_temps("k10temp"))
                names.append("CPU")
            if zone == TempZone.SYSTEM:
                append_temps_list(self.hwmon.get_temps("nct7802"))
                names.append("BOARD")
            if zone == TempZone.PHY:
                append_temps_list(self.hwmon.get_temps_by_type("phy"))
                names.append("PHY")
            if zone == TempZone.NVME:
                append_temps_list(self.hwmon.get_temps_by_type("nvme"))
                names.append("NVME")

        return FS68_TempAggregator(temps, ",".join(names)) if len(temps) else None

    def __enter__(self):
        if not self.is_open:
            self.open()
        return self

    def __exit__(self, *args, **kwargs):
        self.close()
