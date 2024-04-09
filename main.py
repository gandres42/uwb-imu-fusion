import time
from typing import List
from threading import Thread, Lock
from networktables import NetworkTables
import numpy as np
from utils.dwm import dwm1001
from utils.kalman import Nonlinear, Dayton
from utils.navx import ahrs

NetworkTables.initialize()
nt = NetworkTables.getTable("localization")

dwm = dwm1001("/dev/ttyACM0")
imu = ahrs("/dev/ttyACM1")

while not imu.is_calibrated():
    time.sleep(0.05)


prev_imu = time.monotonic()
prev_dwm = time.monotonic()
init_pos = dwm.position()
ekf = Nonlinear(init_pos.px, init_pos.py, init_pos.pz)
anchors = []
try:
    while True:
        pass
        if time.monotonic() - prev_dwm > 0.1:
            anchors = dwm.anchors()
            prev_dwm = time.monotonic()
            ekf.dwm_update(anchors, 
                           imu.get_accel_x(), 
                           imu.get_accel_y(), 
                           imu.get_accel_z())
            nt.putNumber('fuse_x', round(ekf.get_x()[0, 0], 2))
            nt.putNumber('fuse_y', round(ekf.get_x()[1, 0], 2))
            prev_imu = time.monotonic()
except KeyboardInterrupt:
    dwm.close()