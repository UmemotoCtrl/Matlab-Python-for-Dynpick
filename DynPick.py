#!/usr/bin/env python
# -*- coding: utf-8 -*-

# MIT License
# Copyright (c) 2025 Kazuki UMEMOTO
# see https://github.com/UmemotoCtrl/Matlab-Python-for-Wacho-Tech-Dynpick for details

import time
import struct
import re
import logging
from typing import List, Optional, Sequence

import serial
import serial.tools.list_ports

logger = logging.getLogger(__name__)
# logging.basicConfig(level=logging.INFO)

class DynPick:
    """
    DynPick serial interface.
    Use with: with DynPick(port) as dpick: ...
    """

    PROTOCOL_PACKET_SIZE = 27
    DEFAULT_BAUD = 921600
    DEFAULT_WAIT = 0.050
    DEFAULT_TIMEOUT = 0.5

    # class-level safe defaults (immutable)
    _DEFAULT_SENSITIVITY = (65.470, 65.440, 65.080, 1638.500, 1639.250, 1640.750)
    _DEFAULT_ZERO_OUTPUT = (0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000)

    ver: str = "25.11.19"

    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout: float = DEFAULT_TIMEOUT):
        self.port = port
        self.baud = baud
        self.wait_time = self.DEFAULT_WAIT
        self.timeout = timeout
        self.is_started = False
        self.force: List[float] = [0.0] * 6

        # instance calibration (copied from defaults)
        self.sensitivity: List[float] = list(self._DEFAULT_SENSITIVITY)
        self.zero_output: List[int] = list(self._DEFAULT_ZERO_OUTPUT)

        # open serial with timeout to allow non-blocking checks
        self.ser = serial.Serial(port, baudrate=baud, timeout=self.timeout)
        # ensure sensor not continuously sending on init
        try:
            self.stop_continuous_read()
        except Exception:
            # ignore errors during initialization
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def close(self) -> None:
        if getattr(self, "ser", None) and self.ser.is_open:
            try:
                self.stop_continuous_read()
            except Exception:
                pass
            self.ser.close()
            logger.debug("Serial port closed.")

    # --- low-level helpers ---
    def _write(self, cmd: bytes) -> None:
        self.ser.flush()
        self.ser.write(cmd)
        time.sleep(self.wait_time)

    def _read_available(self) -> bytes:
        n = self.ser.in_waiting
        if n:
            return self.ser.read(n)
        return b""

    # --- public API ---
    def print_firmware_version(self) -> None:
        self._write(b"V")
        data = self._read_available()
        if data:
            logger.info(data.decode(errors="ignore"))
        else:
            logger.info("No response for firmware version.")

    def print_sensitivity(self) -> None:
        self._write(b"p")
        data = self._read_available()
        if data:
            logger.info(data.decode(errors="ignore"))
        else:
            logger.info("No response for sensitivity info.")

    def set_sensitivity(self) -> bool:
        """
        Query sensor for sensitivity values and update instance calibration.
        Returns True on success.
        """
        self._write(b"p")
        data = self._read_available()
        if not data:
            logger.warning("No data received from sensor for sensitivity.")
            return False

        text = data.decode(errors="ignore")
        # robust number parsing: integers, decimals, scientific
        nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)
        try:
            if len(nums) >= 6:
                vals = [float(nums[i]) for i in range(6)]
                # replace zeros with inf to avoid division by zero as before
                self.sensitivity = [v if v != 0.0 else float("inf") for v in vals]
                logger.info("Sensitivities updated: %s", self.sensitivity)
                return True
            else:
                logger.warning("Expected 6 sensitivity values, got %d: %s", len(nums), nums)
                return False
        except Exception as e:
            logger.exception("Failed to parse sensitivity data: %s", e)
            return False

    def read_temperature(self) -> Optional[float]:
        self._write(b"T")
        data = self._read_available()
        if not data:
            return None
        # expect at least 4+2 bytes as in original (=4s2B)
        try:
            if len(data) < 6:
                logger.debug("Temperature packet too short: %d bytes", len(data))
                return None
            raw, b1, b2 = struct.unpack("=4s2B", data[:6])
            temp_hex = raw.decode(errors="ignore")
            return float(int(temp_hex, 16)) / 16.0
        except Exception:
            logger.exception("Failed to decode temperature from: %r", data)
            return None

    def read_once(self) -> List[float]:
        """One-shot read: sends 'R' and returns 6-channel force/torque."""
        self._write(b"R")
        # shorter wait as original used wait_time/5
        time.sleep(max(0.001, self.wait_time / 5.0))
        in_waiting = self.ser.in_waiting
        if in_waiting >= self.PROTOCOL_PACKET_SIZE:
            recv = self.ser.read(self.PROTOCOL_PACKET_SIZE)
            try:
                parts = struct.unpack("=B4s4s4s4s4s4s2B", recv)[1:7]
                return self._bytes_to_double(list(parts))
            except Exception:
                logger.exception("Failed to unpack read_once packet.")
                return [float("nan")] * 6
        logger.debug("No available data for read_once (in_waiting=%d).", in_waiting)
        return [float("nan")] * 6

    def start_continuous_read(self) -> None:
        self.ser.flush()
        self._write(b"S")
        self.is_started = True
        # try an immediate read to prime
        self.force = self.read_continuous()

    def stop_continuous_read(self) -> None:
        try:
            self._write(b"E")
        except Exception:
            # if serial not open or write fails, ignore
            pass
        self.ser.flush()
        self.is_started = False

    def read_continuous(self) -> List[float]:
        """
        Non-blocking read of continuous stream. Returns last known force values.
        Protocol: lines ending with LF(0x0A), CR(0x0D) expected.
        """
        if not self.is_started:
            logger.debug("Data send is NOT started.")
            return self.force

        in_waiting = self.ser.in_waiting
        if in_waiting < self.PROTOCOL_PACKET_SIZE:
            return self.force

        recv = self.ser.read(in_waiting)
        if not recv:
            return self.force

        # find last LF (0x0A) and extract preceding packet
        try:
            if b"\n" in recv:
                idx = recv.rfind(b"\n")
                start = idx - self.PROTOCOL_PACKET_SIZE + 1
                if start >= 0:
                    packet = recv[start : idx + 1]
                    # check CR present at expected position (as original)
                    if len(packet) == self.PROTOCOL_PACKET_SIZE and packet[-2] == 0x0D:
                        parts = struct.unpack("=B4s4s4s4s4s4s2B", packet)[1:7]
                        self.force = self._bytes_to_double(list(parts))
        except Exception:
            logger.exception("Failed to parse continuous packet.")
        return self.force

    def _bytes_to_double(self, arg_bytes6: Sequence[bytes]) -> List[float]:
        """
        Convert 6 ASCII-hex fields (bytes) to scaled floats using zero_output and sensitivity.
        """
        hex_strs = []
        for b in arg_bytes6:
            if isinstance(b, (bytes, bytearray)):
                s = b.decode(errors="ignore").strip()
            else:
                s = str(b).strip()
            hex_strs.append(s)

        try:
            ints = [int(s, 16) for s in hex_strs]
        except Exception:
            logger.exception("Failed to convert hex strings to int: %s", hex_strs)
            return [float("nan")] * 6

        doubles = []
        for i in range(6):
            try:
                val = float(ints[i] - self.zero_output[i]) / float(self.sensitivity[i])
            except Exception:
                val = float("nan")
            doubles.append(val)
        return doubles

    def set_calibration(
        self,
        sensitivity: Optional[List[float]] = None,
        zero_output: Optional[List[int]] = None,
    ) -> None:
        if sensitivity is not None:
            if len(sensitivity) != 6:
                raise ValueError("sensitivity must be a list of 6 elements.")
            self.sensitivity = list(sensitivity)
        if zero_output is not None:
            if len(zero_output) != 6:
                raise ValueError("zero_output must be a list of 6 elements.")
            self.zero_output = list(zero_output)

    # --- utilities for port discovery ---
    @classmethod
    def open_ports_by_serial_number(cls, serial_number: str) -> "DynPick":
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.serial_number == serial_number:
                return cls(p.device)
        raise ValueError(f"{serial_number} was not found.")

    @staticmethod
    def device_name_from_serial_number(serial_number: str) -> Optional[str]:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.serial_number == serial_number:
                return p.device
        return None

    @staticmethod
    def print_list_ports() -> None:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            vid = p.vid or 0
            pid = p.pid or 0
            sn = p.serial_number or "None"
            manu = p.manufacturer or ""
            logger.info("Name:%s, Serial Number:%s, VID:PID:%04X:%04X, Manufacturer:%s", p.device, sn, vid, pid, manu)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)    # To see packet error messages
    # logging.disable(logging.CRITICAL) # to disable all logging
    # logging.disable(logging.NOTSET) # to enable logging again
    logging.info("DynPick Python Interface Version %s", DynPick.ver)
    DynPick.print_list_ports()
    ports = serial.tools.list_ports.comports()
    '''
    # Port Open from Port Name
    # ls -l /dev/tty.* # for macOS
    dpick = DynPick('/dev/tty.usbserial-AU05U761A')
    # See device manager for Windows OS
    dpick = DynPick('COM4')
    # The argument (serial number) should be rewritten using the output from DynPick.print_list_ports() as the reference.
    # dpick = DynPick.open_ports_by_serial_number('AU05U761A')
    '''
    if not ports:
        logger.info("No serial ports found.")
    else:
        dpick = DynPick.open_ports_by_serial_number(ports[-1].serial_number)
        dpick.print_firmware_version()
        dpick.print_sensitivity()
        logger.info("Temperature: %.2f °C", dpick.read_temperature())
        dpick.set_sensitivity()
        logger.info("%s [N], [Nm]", dpick.read_once())
        dpick.start_continuous_read()
        for _ in range(10):
            time.sleep(0.3)
            logger.info(dpick.read_continuous())
        dpick.stop_continuous_read()