#!/usr/bin/env python3

import time
import sys
from py_navx import AHRS, SerialDataType

ahrs = AHRS("/dev/ttyACM1", SerialDataType.kRaw, 100)

while True:
    print(
        ahrs.get_raw_gyro_z()
    )
    time.sleep(0.01)
