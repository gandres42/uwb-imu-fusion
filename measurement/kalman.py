import numpy as np
import time
import math

class Kalman:
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
            [.1997, 0, 0, 0],
            [0, .1997, 0, 0],
            [0, 0, .05, 0],
            [0, 0, 0, .05]
        ])

        self.P = np.identity(6) * 100

        self.jx_covar = 0.006
        self.jy_covar = 0.006

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
