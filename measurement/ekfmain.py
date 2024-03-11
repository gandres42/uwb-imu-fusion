import time
from typing import List
from dwm1001 import dwm1001
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

dwm1 = dwm1001("/dev/ttyACM0")
dwm2 = dwm1001("/dev/ttyACM1")
imu = AHRS("/dev/ttyACM3")
prev_imu = time.monotonic()
prev_dwm = time.monotonic()
time.sleep(0.1)
# imu.zero_yaw()
init_pos = dwm1.position()
filter = Nonlinear(init_pos.px, init_pos.py, init_pos.pz)
anchors = []
try:
    while True:
        if time.monotonic() - prev_dwm > 0.1:
            anchors1 = dwm1.anchors()
            anchors2 = dwm2.anchors()
            anchors = anchors1 + anchors2
            position1 = dwm1.position()
            nt.putNumber('dwm_x', position1.px)
            nt.putNumber('dwm_y', position1.py)
            prev_dwm = time.monotonic()
        if time.monotonic() - prev_imu >= 0.01:
            print(imu.get_accel_x(), imu.get_accel_y(), imu.get_accel_z(), end="\r")
            print()
            filter.dwm_update(anchors, imu.get_accel_x(), imu.get_accel_y(), imu.get_accel_z())
            nt.putNumber('fuse_x', round(filter.get_x()[0, 0], 2))
            nt.putNumber('fuse_y', round(filter.get_x()[1, 0], 2))
            prev_imu = time.monotonic()
except KeyboardInterrupt:
    dwm1.close()
    dwm2.close()