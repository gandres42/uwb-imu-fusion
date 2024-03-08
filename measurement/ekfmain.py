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

filter = Nonlinear(0, 0)
NetworkTables.initialize()
nt = NetworkTables.getTable("localization")

dwm = Decawave1001Driver("/dev/ttyACM0")
anchors = dwm.get_loc().get_anchor_distances_and_positions()
filter.dwm_update(anchors)

# imu = AHRS("/dev/ttyACM1")
# time.sleep(0.1)
# imu.zero_yaw()
# ot = Nonlinear(0, 0)

# try:
#     while True:
#         ot.imu_update(imu.get_accel_x(), imu.get_accel_y(), imu.get_accel_z())
        
#         print()
#         print('------------------------------')
#         print(imu.get_disp_x())
#         print()
#         print(round(ot.get_x()[0, 0], 3))
#         print(round(ot.get_x()[2, 0], 3))
#         print(round(ot.get_x()[4, 0], 3))
#         time.sleep(0.01)
# except KeyboardInterrupt:
#     print("closing sensors")
# finally:
#     imu.close()