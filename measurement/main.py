import time
from typing import List
from decawave_1001_uart import Decawave1001Driver
from navx import AHRS
from threading import Thread, Lock
from kalman import Kalman
from networktables import NetworkTables
import numpy as np

def list_append(list, n):
    list.append(n)
    return list

NetworkTables.initialize()
nt = NetworkTables.getTable("localization")

imu = AHRS("/dev/ttyACM1")
dwm = Decawave1001Driver("/dev/ttyACM0")
time.sleep(0.1)

try:
    # seed initial IMU buffer for .1s
    # seed UWB interpolation function with first 2 readings
    imu.zero_yaw()
    imu_buffer = []
    dwm_there = list_append(dwm.get_pos().get_position().position(), time.monotonic())
    for i in range(0, int(imu.get_sample_rate()/10)):
        imu_buffer.append((imu.get_accel_x(), imu.get_accel_y(), imu.get_accel_z(), time.monotonic()))
    dwm_here = list_append(dwm.get_pos().get_position().position(), time.monotonic())

    # continuously loop
    while True:
        # if new IMU available:
        if time.monotonic() - imu_buffer[-1][3] >= 0.01:
            # append new value and timestamp to buffer
            imu_buffer.append((imu.get_accel_x(), imu.get_accel_y(), imu.get_accel_z(), time.monotonic()))
            # pop off old value and timestamp
            imu_z = imu_buffer.pop(0)
            
            # create linearly interpolated DWM positions using dwm_there and dwm_here
            dwm_time = imu_z[3] + (imu_z[3] - dwm_there[3])
            dwm_z = [
                np.interp(dwm_time, [dwm_there[3], dwm_here[3]], [dwm_there[0], dwm_here[0]]),
                np.interp(dwm_time, [dwm_there[3], dwm_here[3]], [dwm_there[1], dwm_here[1]]),
                np.interp(dwm_time, [dwm_there[3], dwm_here[3]], [dwm_there[2], dwm_here[2]])
            ]
            # update state
            # push estimated state to networktables
            nt.putNumber('time', time.monotonic())
            nt.putNumber('x', (10 * time.monotonic()) % 100)
            nt.putNumber('y', (10 * time.monotonic()) % 100)
        if time.monotonic() - dwm_here[3] >= 0.1:
            dwm_there = dwm_here
            dwm_here = list_append(dwm.get_pos().get_position().position(), time.monotonic())
except KeyboardInterrupt:
    print("closing sensors")
finally:
    imu.close()
    dwm.close()