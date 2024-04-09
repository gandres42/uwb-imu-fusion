import numpy as np
import time
import math
from utils.dwm import Anchor

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

        self.P = np.identity(9)
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
        Q = np.identity(3) * .5

        R = np.identity(len(anchors) + 3) * .197
        R[-1, -1] = .69
        R[-2, -2] = .69
        R[-3, -3] = .69

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
                (((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) and (x_k - anchor.px)/(((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) or 1,
                (((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) and (y_k - anchor.py)/(((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) or 1,
                (((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) and (z_k - anchor.pz)/(((x_k - anchor.px)**2 + (y_k - anchor.py)**2 + (z_k - anchor.pz)**2)**.5) or 1,
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