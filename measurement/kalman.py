import numpy as np
import time
import math
import filterpy
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from decawave_1001_uart import DwmDistanceAndPosition

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

        self.jx_covar = 0.05
        self.jy_covar = 0.05

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

class Nonlinear:
    class Anchor:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    def __init__(self, init_x: float, init_y: float):
        self.x = np.array([
            [init_x],
            [init_y],
            [0],
            [0],
            [0],
            [0]
        ])

        self.P = np.identity(6) * 10

        self.prev_imu = time.monotonic()
        self.prev_dwm = time.monotonic()

        self.anchors = {}

    # def add_anchor()
        
    
    def dwm_update(self, anchors: list[DwmDistanceAndPosition]):
        if len(anchors) == 0:
            return
        
        dt = time.monotonic() - self.prev_dwm
        self.prev_dwm = time.monotonic()

        F = np.array([
            [1, 0, dt, 0, (dt**2)/2, 0],
            [0, 1, 0, dt, 0, (dt**2)/2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        G = np.array([
            [(dt**3)/6, 0],
            [0, (dt**3)/6],
            [(dt**2)/2, 0],
            [0, (dt**2)/2],
            [dt, 0],
            [0, dt]
        ])

        z = []
        for anchor in anchors:
            z.append([anchor.distance() * .001])
        z = np.array(z)

        Hx = []
        x_k = self.x[0, 0]
        y_k = self.x[1, 0]
        for anchor in anchors:
            ax = anchor.position().position()[0] * .001
            ay = anchor.position().position()[1] * .001
            Hx.append([
                (x_k - ax)/(((x_k - ax)**2 + (y_k - ay)**2)**.5),
                (x_k - ay)/(((x_k - ax)**2 + (y_k - ay)**2)**.5),
                0,
                0,
                0,
                0
            ])
        Hx = np.array(Hx)
        print(Hx)
            



    def imu_update(self, ax, ay, az):
        dt = time.monotonic() - self.prev_imu
        self.prev_imu = time.monotonic()

        F = np.array([
            [1, 0, dt, 0, (dt**2)/2, 0],
            [0, 1, 0, dt, 0, (dt**2)/2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        z = np.array([
            [ax],
            [ay]
        ])

        Q = np.array([
            [.1, 0, 0, 0, 0, 0],
            [0, .1, 0, 0, 0, 0],
            [0, 0, .05, 0, 0, 0],
            [0, 0, 0, .05, 0, 0],
            [0, 0, 0, 0, .01, 0],
            [0, 0, 0, 0, 0, .01]
        ])

        H = np.array([
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        R = np.array([
            [1, 0],
            [0, 1]
        ])

        P_prior = F @ self.P @ F.T + Q
        K = P_prior @ H.T @ np.linalg.inv(H @ P_prior @ H.T + R)
        x_prior = F @ self.x
        self.x = x_prior + K @ (z - H @ x_prior)
        self.P = self.P = (np.identity(6) - K @ H) * P_prior

    def get_x(self):
        return self.x