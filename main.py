import navx
import time
from decawave_1001_uart import Decawave1001Driver
import os
from threading import Thread, Lock

active = True

gyron = navx.AHRS("/dev/ttyACM0")
dwm = Decawave1001Driver("/dev/ttyACM1")

imu_buffer = []
imu_buffer_lock = Lock()

def imu_thread():
    while active:
        with imu_buffer_lock:
            imu_buffer.append((time.monotonic_ns(), gyron.linear_accel_x, gyron.linear_accel_y, gyron.linear_accel_z))
        time.sleep(0.01)

try:
    gyron_thread = Thread(target=imu_thread)
    gyron_thread.start()
    with open("data.txt", 'w') as f:
        while True:
            os.system('cls' if os.name == 'nt' else 'clear')
            with imu_buffer_lock:
                # print(len(imu_buffer))
                for entry in imu_buffer:
                    f.write(str(('imu', entry)))
                imu_buffer.clear()
            dists = dwm.get_loc().get_anchor_distances_and_positions()
            fancy_dists = []
            for anchor in dists:
                fancy_dists.append((anchor.distance(), anchor.quality_factor(), anchor.position().position()))
            f.write(str(('uwb', time.monotonic_ns(), fancy_dists)))

            time.sleep(0.2)
except KeyboardInterrupt:
    print("closing sensors")
    active = False
    gyron.close()
    dwm.close()