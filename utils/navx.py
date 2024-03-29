from multiprocessing import Process, Lock, Value, Event
from py_navx import AHRS, SerialDataType
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
import warnings

warnings.simplefilter("ignore", UserWarning)

class angle_filter():
    def __init__(self, dt):
        self.f = KalmanFilter (dim_x=2, dim_z=1)
        self.f.x = np.array([[0.],
                             [0.]])
        self.f.H = np.array([[0, 1]])
        self.f.P *= 1
        self.f.R = .001
        self.f.Q = np.identity(2)
        self.prev_t = time.monotonic_ns()
        self.x = 0

    def update(self, z):
        dt = (time.monotonic_ns() - self.prev_t)/1e9
        self.prev_t = time.monotonic_ns()
        self.f.F = np.array([[1.,dt],
                             [0.,1.]])
        self.f.Q = Q_discrete_white_noise(dim=2, dt=dt, var=.01)
        self.f.predict()
        self.f.update(z)
        return self.f.x[0, 0]
        # return self.x


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
        grav = self.calibrate(endpoint)
        Process(target=self.__update__, args=(endpoint, grav)).start()

    def calibrate(self, endpoint):
        ahrs = AHRS(endpoint, SerialDataType.kProcessedData, 200)
        x = []
        y = []
        z = []
        c_count = 200
        while len(x) < c_count or len(y) < c_count or len(z) < c_count or np.var(x) > .0001 or np.var(y) > .0001 or np.var(z) > .0001:
            x.append(ahrs.get_roll())
            if len(x) > c_count: x.pop(0)
            y.append(ahrs.get_pitch())
            if len(y) > c_count: y.pop(0)
            z.append(ahrs.get_yaw())
            if len(z) > c_count: z.pop(0)
            time.sleep(1/200)
        ahrs.close()
        return [np.mean(x), np.mean(y), np.mean(z)]


    def __update__(self, endpoint, grav):
        ahrs = AHRS(endpoint, SerialDataType.kRaw, 200)
        gx_filter = angle_filter(1/200)
        gy_filter = angle_filter(1/200)
        gz_filter = angle_filter(1/200)
        while not self.kill.is_set():            
            with self.lock:
                self.gx.value = gx_filter.update(ahrs.get_raw_gyro_x()) #- grav[0]
                self.gy.value = gy_filter.update(ahrs.get_raw_gyro_y()) #- grav[1]
                self.gz.value = gz_filter.update(ahrs.get_raw_gyro_z())
                curr_rot = R.from_euler('xyz', [self.gx.value, self.gy.value, self.gz.value], degrees=True).as_matrix()
                grav_offset = np.dot(curr_rot, np.array([0, 0, 1]))
                # print()
                # print(round(self.gx.value, 2), round(self.gy.value, 2), round(self.gz.value, 2))
                # print(grav_offset)
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
        
    def zero_gyro_z(self):
        with self.lock:
            self.gz.value = 0


i = imu("/dev/ttyACM1")
print('calibrating...')
while True:
    print()
    print(i.get_gyro_x())
    print(i.get_gyro_y())
    print(i.get_gyro_z())
    time.sleep(.01)
