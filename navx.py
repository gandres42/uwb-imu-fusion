import serial
import os
from threading import Thread, Lock
import time

class AHRS():
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
    
    def _listen_thread(self):
        while self.active:
            buffer = self.uart.read(1)
            if buffer == b'!':
                self.uart.timeout = None
                buffer = buffer + self.uart.read(1)
                # binary message
                if buffer[1:2] == b'#':
                    buffer = buffer + self.uart.read(1)
                    msg_len = buffer[-1] - 1
                    buffer = buffer + self.uart.read(msg_len)

                    # check message validity
                    if buffer[-1] != 10 or buffer[-2] != 13:
                        print("rip")
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
                            self.yaw = self._signed_hundredths(buffer[4:6])
                            self.roll = self._signed_hundredths(buffer[6:8])
                            self.pitch = self._signed_hundredths(buffer[8:10])
                            self.heading = self._unsigned_hundredths(buffer[12:16])
                            self.altitude = self._q16_16(buffer[12:16])
                            self.fused_heading = self._unsigned_hundredths(buffer[16:18])
                            self.linear_accel_x = self._signed_thousandths(buffer[18:20])
                            self.linear_accel_y = self._signed_thousandths(buffer[20:22])
                            self.linear_accel_z = self._signed_thousandths(buffer[22:24])
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


    def __init__(self, serial_port):
        self.uart = serial.Serial(serial_port, baudrate=48000000, timeout=0)
        self.active = True
        
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.heading = 0
        self.altitude = 0
        self.fused_heading = 0
        self.linear_accel_x = 0
        self.linear_accel_y = 0
        self.linear_accel_z = 0
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

        self.listen_thread = Thread(target=self._listen_thread)
        self.listen_thread.start()
        self.val_lock = Lock()

    def close(self):
        self.active = False