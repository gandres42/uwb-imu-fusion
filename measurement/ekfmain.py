import time
from typing import List
from decawave_1001_uart import Decawave1001Driver
from navx import AHRS
from threading import Thread, Lock
from kalman import Nonlinear
from networktables import NetworkTables
import numpy as np

def list_append(list, n):
    list.append(n)
    return np.array(list)


# NetworkTables.initialize()
# nt = NetworkTables.getTable("localization")

dwm = Decawave1001Driver("/dev/ttyACM0")
imu = AHRS("/dev/ttyACM1")
prev_imu = time.monotonic()
prev_dwm = time.monotonic()
time.sleep(0.1)
imu.zero_yaw()
init_pos = dwm.get_pos().get_position().position()
filter = Nonlinear(init_pos[0] * .001, init_pos[1] * .001, init_pos[2] * .001)
while True:
    # if time.monotonic() - prev_imu >= 0.01:
    #     filter.imu_update(imu.get_accel_x(), imu.get_accel_y(), imu.get_accel_z())
    #     prev_imu = time.monotonic()
    if time.monotonic() - prev_dwm > 0.1:
        anchors = dwm.get_loc().get_anchor_distances_and_positions()
        if len(anchors) == 4:
            filter.dwm_update(anchors)
        prev_dwm = time.monotonic()
        print(round(filter.get_x()[0, 0], 3), end=", ")
        print(round(filter.get_x()[1, 0], 3), end=", ")
        print(round(filter.get_x()[2, 0], 3))