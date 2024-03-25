import numpy as np
import time
import math
import filterpy
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from dwm1001 import dwm1001, Anchor

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

    def get_x(self):
        return self.x

class Nonlinear:
    class Anchor:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    def __init__(self, init_x: float, init_y: float, init_z: float):
        self.x = np.array([
            [init_x],
            [init_y],
            [init_z],
            [0],
            [0],
            [0],
            [0],
            [0],
            [0]
        ])

        self.P = np.identity(9) * 5
        self.prev_imu = time.monotonic()
        self.prev_dwm = time.monotonic()
        
    
    def dwm_update(self, anchors: list[Anchor], ax, ay, az):
        def z_factory(x, anchors: list[Anchor]):
            eax = x[-3, 0]
            eay = x[-2, 0]
            eaz = x[-1, 0]
            z = []
            for anchor in anchors:
                z.append([((x[0, 0] - anchor.px)**2 + (x[1, 0] - anchor.py)**2 + (x[2, 0] - anchor.pz)**2)**.5])
            z.append([eax])
            z.append([eay])
            z.append([eaz])
            return np.array(z)

        if len(anchors) == 0:
            return
        
        ax = ax * 9.80665
        ay = ay * 9.80665
        az = az * 9.80665
        
        dt = time.monotonic() - self.prev_dwm
        self.prev_dwm = time.monotonic()

        F = np.array([
            [1, 0, 0, dt, 0, 0, (dt**2)/2, 0, 0],
            [0, 1, 0, 0, dt, 0, 0, (dt**2)/2, 0],
            [0, 0, 1, 0, 0, dt, 0, 0, (dt**2)/2],
            [0, 0, 0, 1, 0, 0, dt, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, dt, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, dt],
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1]
        ])

        G = np.array([
            [(dt**3)/6, 0, 0],
            [0, (dt**3)/6, 0],
            [0, 0, (dt**3)/6],
            [(dt**2)/2, 0, 0],
            [0, (dt**2)/2, 0],
            [0, 0, (dt**2)/2],
            [dt, 0, 0],
            [0, dt, 0],
            [0, 0, dt]
        ])

        Q = np.identity(3)


        R = np.identity(len(anchors) + 3) * .19
        R[-1, -1] = 1
        R[-2, -2] = 1
        R[-3, -3] = 1

        z = []
        for anchor in anchors:
            z.append([anchor.dst])
        z.append([ax])
        z.append([ay])
        z.append([az])
        z = np.array(z)

        x_k = self.x[0, 0]
        y_k = self.x[1, 0]
        z_k = self.x[2, 0]
        H = []
        for anchor in anchors:
            H.append([
                (((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) and (x_k - anchor.px)/(((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) or 0,
                (((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) and (y_k - anchor.py)/(((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) or 0,
                (((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) and (z_k - anchor.pz)/(((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) or 0,
                0,
                0,
                0,
                0,
                0,
                0
            ])
        H.append([0, 0, 0, 0, 0, 0, 1, 0, 0])
        H.append([0, 0, 0, 0, 0, 0, 0, 1, 0])
        H.append([0, 0, 0, 0, 0, 0, 0, 0, 1])
        H = np.array(H)
        
        x_prior = F @ self.x
        P_prior = F @ self.P @ F.T + (G @ Q @ G.T)
        K = P_prior @ H.T @ np.linalg.inv(H @ P_prior @ H.T + R)
        self.x = x_prior + K @ (z - z_factory(self.x, anchors))
        self.prev_z = z
        self.P = (np.identity(9) - K @ H) @ P_prior

    def get_x(self):
        return self.x