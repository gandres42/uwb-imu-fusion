import serial
from threading import Thread, Lock
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import os
import time

class AHRS():
    def __init__(self, serial_port):
        self.raw_yaw = 0
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.heading = 0
        self.altitude = 0
        self.fused_heading = 0
        self.raw_accel_x = 0
        self.raw_accel_y = 0
        self.raw_accel_z = 0
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.disp_x = 0
        self.disp_y = 0
        self.disp_z = 0
        self.quat_w = 0
        self.quat_x = 0
        self.quat_y = 0
        self.quat_z = 0
        self.mpu_temp = 0
        self.opstatus = 0
        self.sensor_status = 0
        self.cal_status = 0
        self.selftest_status = 0
        self.timestamp = 0

        self.internal_offset = 0

        self.active = True
        self.uart = serial.Serial(serial_port, baudrate=48000000, timeout=0)
        self.uart_lock = Lock()
        self.val_lock = Lock()
        self.listen_thread = Thread(target=self._listen_thread)
        self.listen_thread.start()

    def _signed_hundredths(self, buffer):
        return int.from_bytes(buffer, signed=True, byteorder='little')/100

    def _unsigned_hundredths(self, buffer):
        return int.from_bytes(buffer, signed=False, byteorder='little')/100
    
    def _signed_thousandths(self, buffer):
        return int.from_bytes(buffer, signed=True, byteorder='little')/1000

    def _q16_16(self, buffer):
        q_number = int.from_bytes(buffer, signed=True, byteorder='little')
        return round(q_number / (1 << 16), 2)

    def _int(self, buffer):
        return int.from_bytes(buffer, signed=True, byteorder='little')
    
    def _rotate_accel(self, x, y, z):
        pose = np.matrix([
            [x],
            [y],
            [z]
        ])
        C = R.from_euler('XYZ', [0, 0, self.internal_offset], degrees=True).as_matrix()
        res = np.matmul(C, pose) # type: ignore
        return (round(res[0, 0], 2), round(res[1, 0], 2), round(res[2, 0], 2))
    
    def _adjusted_yaw(self, yaw):
        new_angle = yaw + self.internal_offset
        if new_angle < -180:
            new_angle = 180 - (abs(new_angle) - 180)
        elif new_angle > 180:
            new_angle = -180 + (abs(new_angle) - 180)
        return round(new_angle, 2)

    def _listen_thread(self):
        while self.active:
            with self.uart_lock:
                buffer = self.uart.read(1)
                if buffer == b'!':
                    self.uart.timeout = None
                    buffer = buffer + self.uart.read(1)
                    # binary message
                    if buffer[1:2] == b'#':
                        # read next valid message into buffer
                        buffer = buffer + self.uart.read(1)
                        msg_len = buffer[-1] - 1
                        buffer = buffer + self.uart.read(msg_len)

                        # check message validity
                        if buffer[-1] != 10 or buffer[-2] != 13:
                            continue

                        # parse based on message type
                        if chr(buffer[3]) == 't':
                            pass # TODO
                        elif chr(buffer[3]) == 'p':
                            checksum = 0
                            for i in range(0, len(buffer[:-4])):
                                checksum = checksum + buffer[i]
                            if hex(checksum)[-2:].upper().encode() != buffer[62:64]:
                                raise RuntimeError("message checksum invalid")
                            with self.val_lock:
                                self.raw_yaw = self._signed_hundredths(buffer[4:6])
                                self.yaw = self._adjusted_yaw(self.raw_yaw)
                                self.roll = self._signed_hundredths(buffer[6:8])
                                self.pitch = self._signed_hundredths(buffer[8:10])
                                self.heading = self._unsigned_hundredths(buffer[12:16])
                                self.altitude = self._q16_16(buffer[12:16])
                                self.fused_heading = self._unsigned_hundredths(buffer[16:18])
                                raw_accel_x = self._signed_thousandths(buffer[18:20]) * 9.80665
                                raw_accel_y = self._signed_thousandths(buffer[20:22]) * 9.80665
                                raw_accel_z = self._signed_thousandths(buffer[22:24]) * 9.80665
                                self.accel_x, self.accel_y, self.accel_z = self._rotate_accel(raw_accel_x, raw_accel_y, raw_accel_z)
                                self.vel_x = self._q16_16(buffer[24:28])
                                self.vel_y = self._q16_16(buffer[28:32])
                                self.vel_z = self._q16_16(buffer[32:36])
                                self.disp_x = self._q16_16(buffer[36:40])
                                self.disp_y = self._q16_16(buffer[40:44])
                                self.disp_z = self._q16_16(buffer[44:48])
                                self.quat_w = self._int(buffer[48:50])
                                self.quat_x = self._int(buffer[50:52])
                                self.quat_y = self._int(buffer[52:54])
                                self.disp_z = self._int(buffer[54:56])
                                self.mpu_temp = self._signed_hundredths(buffer[56:58])
                                self.sensor_status = self._int(buffer[59:60])
                                self.cal_status = self._int(buffer[60:61])
                                self.selftest_status = self._int(buffer[61:62])
            time.sleep(0.005)

    def zero_yaw(self):
        self.internal_offset = -self.raw_yaw

    def close(self):
        self.active = False