from multiprocessing import Process, Lock, Value, Event
from py_navx import AHRS, SerialDataType
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

class angle_filter():
    def __init__(self, dt):
        self.f = KalmanFilter (dim_x=2, dim_z=1)
        self.f.x = np.array([[0.],
                             [0.]])
        self.f.F = np.array([[1.,dt],
                             [0.,1.]])
        self.f.H = np.array([[0, 1]])
        self.f.P *= 10
        self.f.R = .001
        self.f.Q = Q_discrete_white_noise(dim=2, dt=1/200, var=0.1)

    def update(self, z):
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
        self.calibrate(endpoint)
        exit()
        Process(target=self.__update__, args=(endpoint, )).start()

    def calibrate(self, endpoint):
        ahrs = AHRS(endpoint, SerialDataType.kRaw, 200)
        x = []
        y = []
        z = []
        
        while len(x) < 200 or len(y) < 200 or len(z) < 200 or np.var(x) > .0001 or np.var(y) > .0001 or np.var(z) > .0001:
            x.append(ahrs.get_raw_accel_x())
            if len(x) > 200: x.pop(0)
            y.append(ahrs.get_raw_accel_y())
            if len(y) > 200: y.pop(0)
            z.append(ahrs.get_raw_accel_z())
            if len(z) > 200: z.pop(0)
            time.sleep(1/200)

        og = np.array([np.mean(x), np.mean(y), np.mean(z)])
        rg = np.array([0, 0, 1])
        rot, rssd = R.align_vectors([rg], [og])
        print(rot)
        print(rot.as_euler('xyz', degrees=True))

        

    def __update__(self, endpoint):
        ahrs = AHRS(endpoint, SerialDataType.kRaw, 200)
        gx_filter = angle_filter(1/200)
        gy_filter = angle_filter(1/200)
        gz_filter = angle_filter(1/200)

        while not self.kill.is_set():
            with self.lock:
                self.gx.value = gx_filter.update(ahrs.get_raw_gyro_x())
                self.gy.value = gy_filter.update(ahrs.get_raw_gyro_y())
                self.gz.value = gz_filter.update(ahrs.get_raw_gyro_z())
            time.sleep(1/200)

    def get_gyro_x(self):
        with self.lock:
            return self.gx.value
    
    def get_gyro_y(self):
        with self.lock:
            return self.gy.value
    
    def get_gyro_z(self):
        with self.lock:
            return self.gz.value
        
    def zero_gyro_z(self):
        with self.lock:
            self.gz.value = 0


i = imu("/dev/ttyACM1")
# while True:
#     print()
#     print(i.get_gyro_x())
#     print(i.get_gyro_y())
#     print(i.get_gyro_z())
#     time.sleep(.01)
