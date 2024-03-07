import time
from typing import List
from decawave_1001_uart import Decawave1001Driver
from navx import AHRS
from threading import Thread, Lock
from kalman import Overthruster
from networktables import NetworkTables
import numpy as np

def list_append(list, n):
    list.append(n)
    return np.array(list)

NetworkTables.initialize()
nt = NetworkTables.getTable("localization")

imu = AHRS("/dev/ttyACM1")
time.sleep(0.1)
imu.zero_yaw()

try:
    while True:
        print(imu.get_accel_x())
except KeyboardInterrupt:
    print("closing sensors")
finally:
    imu.close()