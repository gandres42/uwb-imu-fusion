import time
from typing import List

from threading import Thread, Lock

from networktables import NetworkTables
import numpy as np
from py_navx import AHRS
from utils.dwm import dwm1001
from utils.kalman import Nonlinear, Dayton
from utils.navx import i

# imu("/dev/ttyACM0")

# NetworkTables.initialize()
# nt = NetworkTables.getTable("localization")

# dwm = dwm1001("/dev/ttyACM0")
# imu = AHRS("/dev/ttyACM1")
# prev_imu = time.monotonic()
# prev_dwm = time.monotonic()
# time.sleep(0.1)
# init_pos = dwm.position()
# ekf = Nonlinear(init_pos.px, init_pos.py, init_pos.pz)
# lin = Dayton(init_pos.px, init_pos.py)
# px = None
# py = None

# anchors = []
# try:
#     while True:
#         pass
#         if time.monotonic() - prev_dwm > 0.1:
#             anchors = dwm.anchors()
#             position = dwm.position()
#             px = position.px
#             py = position.py
#             prev_dwm = time.monotonic()
#         if time.monotonic() - prev_imu >= 0.01:
#             ekf.dwm_update(anchors, 
#                            imu.get_world_linear_accel_x(), 
#                            imu.get_world_linear_accel_y(), 
#                            imu.get_world_linear_accel_z())
#             nt.putNumber('fuse_x', round(ekf.get_x()[0, 0], 2))
#             nt.putNumber('fuse_y', round(ekf.get_x()[1, 0], 2))
#             prev_imu = time.monotonic()
# except KeyboardInterrupt:
#     dwm.close()

print("sined...")
print("seeled...")
print("delivered...")
print("crossover")