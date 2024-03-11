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

np.set_printoptions(edgeitems=30, linewidth=100000, formatter=dict(float=lambda x: "%.2g" % x)) # type: ignore

NetworkTables.initialize()
nt = NetworkTables.getTable("localization")

dwm1 = Decawave1001Driver("/dev/ttyACM0")
dwm2 = Decawave1001Driver("/dev/ttyACM1")
imu = AHRS("/dev/ttyACM2")
prev_imu = time.monotonic()
prev_dwm = time.monotonic()
time.sleep(0.1)
imu.zero_yaw()
init_pos = dwm1.get_pos().get_position().position()
filter = Nonlinear(init_pos[0] * .001, init_pos[1] * .001, init_pos[2] * .001)
anchors = []
while True:
    if time.monotonic() - prev_dwm > 0.1:
        anchors1 = dwm1.get_loc().get_anchor_distances_and_positions()
        anchors2 = dwm2.get_loc().get_anchor_distances_and_positions()
        anchors = anchors1 + anchors2
        position1 = dwm1.get_pos().get_position().position()
        nt.putNumber('dwm_x', position1[0] * .001)
        nt.putNumber('dwm_y', position1[1] * .001)
        prev_dwm = time.monotonic()
    if time.monotonic() - prev_imu >= 0.01:
        filter.dwm_update(anchors, imu.get_accel_x(), imu.get_accel_y(), imu.get_accel_z())
        nt.putNumber('fuse_x', round(filter.get_x()[0, 0], 2))
        nt.putNumber('fuse_y', round(filter.get_x()[1, 0], 2))
        prev_imu = time.monotonic()
    