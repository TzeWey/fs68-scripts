#!/usr/bin/env python
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
# This copy is reduces to the minimum needed to work on TrueNAS Scale @115200n8
#

import os
import errno
import termios
import select
import time


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
            return timeout_time - time.monotonic()

        timeout_time = time.monotonic() + self._timeout if self._timeout is not None else None

        while len(read) < size:
            try:
                ready, _, _ = select.select([self.fd], [], [], time_left())
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
