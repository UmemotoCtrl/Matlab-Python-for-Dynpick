#!/usr/bin/env python
# -*- coding: utf-8 -*-

# MIT License
# Copyright (c) 2025 Kazuki UMEMOTO
# see https://github.com/UmemotoCtrl/Matlab-Python-for-Wacho-Tech-Dynpick for details


import time
import serial
import serial.tools.list_ports
import struct
import re
from typing import List, Optional, Sequence

class DynPick:
    # LPFとゼロ点の設定は未実装，他は実装済み
    ver: str = '25.11.18'  # 最終更新日
    wait_time: float = 0.050
    timeout : float = 0.500
    is_started: bool = False
    force: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # 校正パラメータ（クラス全体で共有）
    sensitivity: List[float] = [65.470, 65.440, 65.080, 1638.500, 1639.250, 1640.750]
    zero_output: List[int] = [0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000]

    def __init__(self, port):
        self.ser = serial.Serial(
            port,
            baudrate = 921600,
            # parity = serial.PARITY_NONE,
            # bytesize = serial.EIGHTBITS,
            # stopbits = serial.STOPBITS_ONE,
            # timeout = None,
            # xonxoff = 0,
            # rtscts = 0,
            )
        self.stop_continuous_read()
    def __del__(self):
        self.close()
    
    def close(self):
        if self.ser.is_open:
            self.stop_continuous_read()
            self.ser.close()
    
    def print_firmware_version(self):
        time.sleep(self.wait_time)
        self.ser.flush()
        self.ser.write(b'V')
        time.sleep(self.wait_time)
        in_waiting = self.ser.in_waiting
        if in_waiting > 0:
            print(self.ser.read(in_waiting).decode())
    def print_sensitivity(self):
        time.sleep(self.wait_time)
        self.ser.flush()
        self.ser.write(b'p')
        time.sleep(self.wait_time)
        in_waiting = self.ser.in_waiting
        if in_waiting > 0:
            print(self.ser.read(in_waiting).decode())

    def set_sensitivity(self) -> bool:
        time.sleep(self.wait_time)
        self.ser.flush()
        self.ser.write(b'p')
        time.sleep(self.wait_time)
        in_waiting = self.ser.in_waiting
        if in_waiting > 0:
            recv = self.ser.read(in_waiting).decode()
            try:
                numbers = re.findall(r'\d+\.\d+', recv)
                if len(numbers) == 6:
                    # 0ならfloat('inf')に、ゼロ割回避
                    DynPick.sensitivity = [
                        float(n) if float(n) != 0.0 else float('inf') for n in numbers
                    ]
                    return True
                else:
                    print(f'Expected 6 sensitivity values, but got {len(numbers)}: {numbers}')
                    return False
            except Exception as e:
                print(f'Failed to parse sensitivity data: {e}')
                print(f'Received data: {recv}')
                return False
        else:
            print('No data received from sensor.')
            return False

    def read_temperature(self) -> Optional[float]:
        time.sleep(self.wait_time)
        self.ser.flush()
        self.ser.write(b'T')
        time.sleep(self.wait_time)
        in_waiting = self.ser.in_waiting
        if in_waiting > 0:
            recv = self.ser.read(in_waiting)
            data = struct.unpack('=4s2B', recv)
            return float(int(data[0].decode(), 16)) / 16.0
        else:
            return None

    def read_once(self) -> List[float]:
        self.ser.write(b'R')
        time.sleep(self.wait_time/5)
        in_waiting = self.ser.in_waiting
        if in_waiting == 27:
            recv = self.ser.read(in_waiting)
            data = struct.unpack('=B4s4s4s4s4s4s2B', recv)[1:7]
            return self.bytesToDouble(list(data))
        else:
            print('No available data.')
            return [float('nan')] * 6

    def start_continuous_read(self):
        self.ser.flush()
        self.ser.write(b'S')
        time.sleep(self.wait_time)
        self.is_started = True
        self.force = self.read_continuous()
    def stop_continuous_read(self):
        self.ser.write(b'E')
        time.sleep(self.wait_time)
        self.ser.flush()
        self.is_started = False
    def read_continuous(self) -> List[float]:
        # 13	0D	CR
        # 10	0A	LF
        if self.is_started:
            in_waiting = self.ser.in_waiting
            if in_waiting >= 27*2:
                recv = self.ser.read(in_waiting)
                if 0x0A in recv:
                    ii = recv.rfind(0x0A)
                    recv = recv[ii-27+1:ii+1]
                    if recv[25] == 0x0D:
                        data = struct.unpack('=B4s4s4s4s4s4s2B', recv)[1:7]
                        self.force = self.bytesToDouble(list(data))
                    # self.ser.flush()
                return self.force
            else:
                return self.force
        else:
            print('Data send is NOT started.')
            return self.force

    def bytesToDouble(self, argBytes6: Sequence[bytes]) -> List[float]:
        # 16進ASCII文字列 -> int
        hex_strs = [b.decode() if isinstance(b, (bytes, bytearray)) else str(b) for b in argBytes6]
        retInt6 = [int(s, 16) for s in hex_strs]

        # ゼロ点補正および感度によるスケーリング
        retDouble6 = [
            float(retInt6[i] - self.zero_output[i]) / self.sensitivity[i]
            for i in range(6)
        ]
        return retDouble6

    def set_calibration(self, sensitivity: Optional[List[float]] = None, zero_output: Optional[List[int]] = None) -> None:
        if sensitivity is not None:
            if len(sensitivity) != 6:
                raise ValueError('sensitivity must be a list of 6 elements.')
            self.sensitivity = sensitivity
        if zero_output is not None:
            if len(zero_output) != 6:
                raise ValueError('zero_output must be a list of 6 elements.')
            self.zero_output = zero_output

    @classmethod
    def open_ports_by_serial_number(cls, serial_number):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return cls(port.device)
        raise ValueError(serial_number + " was not found.")
    @staticmethod
    def device_name_from_serial_number(serial_number):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return port.device
        return None
    @staticmethod
    def print_list_ports():
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid is None:
                port.serial_number = "None"
                port.vid = 0
                port.pid = 0
            print('Name:%s, Serial Number:%s, VID:PID:%04X:%04X, Manufacturer:%s'%(
                port.device,
                port.serial_number,
                port.vid,
                port.pid,
                port.manufacturer) )
            # print("-----------")
            # print(port.device)
            # print(port.name)
            # print(port.description)
            # print(port.hwid)
            # print(port.vid)
            # print(port.pid)
            # print(port.serial_number)
            # print(port.location)
            # print(port.manufacturer)
            # print(port.product)
            # print(port.interface)

if __name__ == '__main__':
    DynPick.print_list_ports()

    '''
    # Port Open from Port Name
    # ls -l /dev/tty.* # for macOS
    dpick = DynPick('/dev/tty.usbserial-AU02EQ8G')
    dpick = DynPick('/dev/tty.usbserial-AU05U761')    
    # See device manager for Windows OS
    dpick = DynPick('COM4')
    # The argument (serial number) should be rewritten using the output from DynPick.print_list_ports() as the reference.
    # dpick = DynPick.open_ports_by_serial_number('AU05U761A')
    '''
    # Open the last port.
    ports = serial.tools.list_ports.comports()
    dpick = DynPick.open_ports_by_serial_number(ports[-1].serial_number)
    # dpick.print_firmware_version()
    # dpick.print_sensitivity()
    # print(dpick.read_temperature())
    dpick.set_sensitivity()
    print(dpick.read_once(), "[N], [Nm]")

    dpick.start_continuous_read()
    for i in range(10):
        print(dpick.read_continuous())
        time.sleep(0.3)
    dpick.stop_continuous_read()
