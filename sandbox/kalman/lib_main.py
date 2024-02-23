import numpy as np
from var import m_samples
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time

class Kalman:
    def __init__(self, initial_d):
        self.prev_d = initial_d
        self.f = KalmanFilter(dim_x=2, dim_z=1)
        self.f.x = np.array([
            [initial_d],
            [0]
        ])
        self.f.P = np.array([
            [100, 0],
            [0, 100]
        ])
        self.f.F = np.array([
            [1, 1],
            [0, 1]
        ])
        self.f.H = np.matrix([
            [0., 1.]
        ])
        self.R = np.array([
            [0.1996]
        ])
        
        self.f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=.1996)
        self.prev_update = initial_d
    
    def update(self, d):
        z = (d - self.prev_d)
        self.prev_d = d
        self.f.predict()
        self.f.update(z)

    def get_dist(self):
        return self.f.x[0, 0]
        

kalman = Kalman(m_samples[0])
for sample in m_samples:
    kalman.update(sample)
    print(kalman.get_dist())
    # time.sleep(0.01)