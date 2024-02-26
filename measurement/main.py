import navx
import time
from decawave_1001_uart import Decawave1001Driver
from threading import Thread, Lock
from kalman import Kalman
from networktables import NetworkTables
import socket
import os

HOST = '0.0.0.0'
PORT = 9000

active = True
gyron = navx.AHRS("/dev/ttyACM1")
dwm = Decawave1001Driver("/dev/ttyACM0")
ot = None
time.sleep(0.1)
gyron.zero_yaw()

px, py, pz = 0, 0, 0

def fancyprint(num):
    if num >= 0:
        print(" ", end="")
    print(num)

def imu_collection():
    global ot, px, py, pz, sd
    while active:
        if ot is not None:
            ot.update(px, py, gyron.accel_x, gyron.accel_y)
            os.system('cls' if os.name == 'nt' else 'clear')
            fancyprint(ot.get_state()[0, 0])
            fancyprint(ot.get_state()[1, 0])
            fancyprint(ot.get_state()[2, 0])
            fancyprint(ot.get_state()[3, 0])
            fancyprint(ot.get_state()[4, 0])
            fancyprint(ot.get_state()[5, 0])
            time.sleep(0.01)

try:
    # start IMU collection thread, fills buffer with IMU data and timestamps
    imu_thread = Thread(target=imu_collection)
    imu_thread.start()

    # initialize filter with first position reading
    init_pos = dwm.get_loc().get_tag_position().position()
    px = init_pos[0]/1000
    py = init_pos[1]/1000
    py = init_pos[2]/1000
    ot = Kalman(px, py)

    # continuously update UWB position
    while True:
        try:
            new_pos = dwm.get_loc().get_tag_position().position()
            px = new_pos[0]/1000
            py = new_pos[1]/1000
            pz = new_pos[2]/1000
        except Exception as e:
            print(e)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("closing sensors")
    active = False
    gyron.close()
    dwm.close()