from multiprocessing import Process, Lock, Value, Event
from py_navx import AHRS, SerialDataType
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
import warnings
from ahrs.filters import Madgwick
import math
import imufusion

warnings.simplefilter("ignore", UserWarning)

class angle_filter():
    def __init__(self):
        self.f = KalmanFilter (dim_x=2, dim_z=1)
        self.f.x = np.array([[0.],
                             [0.]])
        self.f.H = np.array([[0, 1]])
        self.f.P *= 1
        self.f.R = .1
        self.f.Q = np.identity(2)
        self.prev_t = time.monotonic_ns()
        self.x = 0

    def update(self, z):
        dt = (time.monotonic_ns() - self.prev_t)/1e9
        self.prev_t = time.monotonic_ns()
        self.f.F = np.array([[1.,dt],
                             [0.,1.]])
        self.f.Q = Q_discrete_white_noise(dim=2, dt=dt, var=.1)
        self.f.predict()
        self.f.update(z)
        return self.f.x[0, 0]


class imu:
    def __init__(self, endpoint):
        self.gx = Value('d', 0)
        self.gy = Value('d', 0)
        self.gz = Value('d', 0)
        self.ax = Value('d', 0)
        self.ay = Value('d', 0)
        self.az = Value('d', 0)
        self.lock = Lock()
        self.kill = Event()
        self.calibrated = Value('i', 0)
        Process(target=self.__update__, args=(endpoint, )).start()

    def __update__(self, endpoint):
        ahrs = imufusion.Ahrs()
        navx = AHRS(endpoint, SerialDataType.kRaw, 200)
        prev_time = time.monotonic_ns()
        offset = 350000 # compensate for lost time, adjust per IMU
        grav_base = np.array([
            [0],
            [0],
            [1]
        ])

        while ahrs.flags.initialising:
            ahrs.update_no_magnetometer(
                np.array([navx.get_raw_gyro_x(), navx.get_raw_gyro_y(), navx.get_raw_gyro_z()]), 
                np.array([navx.get_raw_accel_x(), navx.get_raw_accel_y(), navx.get_raw_accel_z()]), 
                (time.monotonic_ns() - prev_time + offset)/1e9)
            prev_time = time.monotonic_ns()

        self.calibrated.value = 1

        while not self.kill.is_set():            
            ahrs.update_no_magnetometer(
                np.array([navx.get_raw_gyro_x(), navx.get_raw_gyro_y(), navx.get_raw_gyro_z()]), 
                np.array([navx.get_raw_accel_x(), navx.get_raw_accel_y(), navx.get_raw_accel_z()]), 
                (time.monotonic_ns() - prev_time + offset)/1e9)
            prev_time = time.monotonic_ns()
            self.gx.value = ahrs.quaternion.to_euler()[0]
            self.gy.value = ahrs.quaternion.to_euler()[1]
            self.gz.value = ahrs.quaternion.to_euler()[2]
            grav = ahrs.quaternion.to_matrix().T @ grav_base
            self.ax.value = navx.get_raw_accel_x() - grav[0, 0]
            self.ay.value = navx.get_raw_accel_y() - grav[1, 0]
            self.az.value = navx.get_raw_accel_z() - grav[2, 0]
            time.sleep(1/200)

    def get_gyro_x(self):
        with self.lock:
            return round(self.gx.value, 2)
    
    def get_gyro_y(self):
        with self.lock:
            return round(self.gy.value, 2)
    
    def get_gyro_z(self):
        with self.lock:
            return round(self.gz.value, 2)
        
    def get_accel_x(self):
        with self.lock:
            return round(self.ax.value, 2)
        
    def get_accel_y(self):
        with self.lock:
            return round(self.ay.value, 2)
        
    def get_accel_z(self):
        with self.lock:
            return round(self.az.value, 2)
        
    def zero_gyro_z(self):
        with self.lock:
            self.gz.value = 0

    def is_calibrated(self):
        with self.lock:
            if self.calibrated.value == 1:
                return True
            return False


i = imu("/dev/ttyACM1")
print('calibrating')
while not i.is_calibrated():
    time.sleep(.05)

while True:
    print()
    print(i.get_gyro_x())
    print(i.get_gyro_y())
    print(i.get_gyro_z())
    print(i.get_accel_x())
    print(i.get_accel_y())
    print(i.get_accel_z())
    time.sleep(.01)
