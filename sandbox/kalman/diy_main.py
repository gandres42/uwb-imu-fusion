import numpy as np
from var import m_samples
import time

class Kalman:
    def __init__(self, initial_d):
        self.x = np.matrix([
            [initial_d],
            [0]
        ])
        self.P = np.matrix([
            [1000000, 0],
            [0, 1000000]
        ])
        self.A = np.matrix([
            [1, 1],
            [0, 1]
        ])
        self.H = np.matrix([
            [0, 1]
        ])
        self.R = np.matrix([
            [.1997]
        ])
        self.Q = np.matrix([
            [.1, 0],
            [0, .1]
        ])
        self.prev_update = initial_d
    
    def update(self, d):
        z = np.matrix([[d - self.prev_update]])
        self.prev_update = d
        P_prior = self.A * self.P * self.A.T
        K = P_prior * self.H.T * np.linalg.inv((self.H * P_prior * self.H.T) + self.R)
        x_prior = self.A * self.x
        self.x = x_prior + K * (z - self.H * x_prior)
        self.P = (np.identity(2) - K * self.H) * P_prior
        print(self.x[0, 0])
        

kalman = Kalman(0)
for sample in m_samples:
    kalman.update(sample)
    # time.sleep(0.01)