import numpy as np
import time
import math
import filterpy
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise

class Dayton:
    def A(self, dt) -> np.ndarray:
        return np.array([
            [1, 0, dt, 0, dt**2/2, 0],
            [0, 1, 0, dt, 0, dt**2/2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
    
    def v(self, dt, jx, jy) -> np.ndarray:
        return np.array([
            [(1/6)*jx*(dt**3)],
            [(1/6)*jy*(dt**3)],
            [(1/2)*jx*(dt**2)],
            [(1/2)*jy*(dt**2)],
            [jx*dt],
            [jx*dt]
        ])
    
    def Q(self, dt):
        return np.array([
            [(dt**6 * self.jx_covar)/36, 0, (dt**5 * self.jx_covar)/12, 0, (dt**4 * self.jx_covar)/6, 0],
            [0, (math.pow(dt, 6) * self.jy_covar)/36, 0, (dt**5 * self.jy_covar)/12, 0, (dt**4 * self.jy_covar)/6],
            [(dt**5 * self.jx_covar)/12, 0, (dt**4 * self.jx_covar)/4, 0, (dt**3 * self.jx_covar)/2, 0],
            [0, (dt**5 * self.jy_covar)/12, 0, (dt**4 * self.jy_covar)/4, 0, (dt**3 * self.jy_covar)/2],
            [(dt**4 * self.jx_covar)/6, 0, (dt**3 * self.jx_covar)/2, 0, (dt**2 * self.jx_covar)/1, 0],
            [0, (dt**4 * self.jy_covar)/6, 0, (dt**3 * self.jy_covar)/2, 0, (dt**2 * self.jy_covar)/1]
        ])


    def __init__(self, init_x, init_y):
        self.x = np.array([
            [init_x],
            [init_y],
            [0],
            [0],
            [0],
            [0]
        ])

        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        self.R = np.array([
            [.0225, 0, 0, 0],
            [0, .0225, 0, 0],
            [0, 0, .2, 0],
            [0, 0, 0, .2]
        ])

        self.P = np.identity(6) * 1

        self.jx_covar = 0.005
        self.jy_covar = 0.005

        self.pax = 0
        self.pay = 0
        self.pt = time.monotonic()

    def update(self, px, py, ax, ay):
        dt = time.monotonic() - self.pt
        self.pt = time.monotonic()
        jx = ax - self.pax
        self.pax = ax
        jy = ay - self.pay
        self.pay = ay

        z = np.array([
            [px],
            [py],
            [ax],
            [ay]
        ])

        P_priori = self.A(dt) @ self.P @ self.A(dt).T + self.Q(dt)
        x_priori = self.A(dt) @ self.x + self.v(dt, jx, jy)
        K = P_priori @ self.H.T @ np.linalg.inv((self.H @ P_priori @ self.H.T) + self.R)
        self.x = x_priori + K @ (z - self.H @ x_priori)
        self.P = (np.identity(6) - K @ self.H) @ P_priori

    def get_state(self):
        return self.x


class Overthruster:
    def __init__(self, init_x, init_y):
        self.x = np.array([
            [init_x],
            [init_y],
            [0],
            [0],
            [0],
            [0]
        ])

        self.P = np.identity(6)

        self.R = np.array([
            [.2, 0],
            [0, .2]
        ])

        self.imu_H = np.array([
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        self.prev_imu = time.monotonic()
        self.prev_dwm = time.monotonic()

    def F(self, dt):
        return np.array([
            [1, 0, dt, 0, (dt**2)/2, 0],
            [0, 1, 0, dt, 0, (dt**2)/2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
    
    def dwm_G(self, dt):
        return np.array([
            [(dt**3)/6, 0],
            [0, (dt**3)/6],
            [(dt**2)/2, 0],
            [0, (dt**2)/2],
            [dt, 0],
            [0, dt]
        ])
    
    def dmw_H(self, dt):
        return np.array([
            # TODO this is going to be a big array
        ])
    
    def dwm_update(self, d1, d2, d3, d4):
        pass    

    def imu_update(self, ax, ay, az):
        dt = time.monotonic() - self.prev_imu
        self.prev_imu = time.monotonic()
        Q = Q_discrete_white_noise(dim=6, dt=dt, var=0.1)
        F = self.F(dt)
        z = np.array([
            [ax],
            [ay]
        ])

        P_prior = F @ self.P @ F.T + Q
        K = P_prior @ self.imu_H @ np.linalg.inv(self.imu_H @ P_prior @ self.imu_H + self.R)
        x_prior = F @ self.x
        self.x = x_prior + K @ (z - self.imu_H @ x_prior)
        self.P = self.P = (np.identity(2) - K * self.imu_H) * P_prior

    def get_x(self):
        return self.x