import numpy as np
import time
import math

prev_timestamp = None
x = None
P = None
j_var = 0.2

def init(pos_x, pos_y):
    global x, P, prev_timestamp
    x = np.matrix([pos_x, pos_y, 0, 0, 0, 0]).T
    P = np.matrix([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ]) # type: ignore
    prev_timestamp = time.monotonic_ns()
    

def get_state():
    global x
    return x

def update(p_x, p_y, a_x, a_y, timestamp):
    global prev_timestamp, x, P, j_var
    d_t = (prev_timestamp - timestamp) / 1e6
    prev_timestamp = timestamp
    F = np.matrix([
        [1, 0, d_t, 0, math.pow(d_t, 2)/2, 0],
        [0, 1, 0, d_t, 0, math.pow(d_t, 2)/2],
        [0, 0, 1, 0, d_t, 0],
        [0, 0, 0, 1, 0, d_t],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ]) # type: ignore
    
    # TODO revisit jerk variance or remove accel. entirely
    # how do we calculate covariance for sensors?
    # how could we adapt this to not include jerk?
    Q = np.matrix([
        [(math.pow(d_t, 6) * j_var)/36, 0, (math.pow(d_t, 5) * j_var)/12, 0, (math.pow(d_t, 4) * j_var)/6, 0],
        [0, (math.pow(d_t, 6) * j_var)/36, 0, (math.pow(d_t, 5) * j_var)/12, 0, (math.pow(d_t, 4) * j_var)/6],
        [(math.pow(d_t, 5) * j_var)/12, 0, (math.pow(d_t, 4) * j_var)/4, 0, (math.pow(d_t, 3) * j_var)/2, 0],
        [0, (math.pow(d_t, 5) * j_var)/12, 0, (math.pow(d_t, 4) * j_var)/4, 0, (math.pow(d_t, 3) * j_var)/2],
        [(math.pow(d_t, 4) * j_var)/6, 0, (math.pow(d_t, 3) * j_var)/2, 0, (math.pow(d_t, 2) * j_var)/1, 0],
        [0, (math.pow(d_t, 4) * j_var)/6, 0, (math.pow(d_t, 3) * j_var)/2, 0, (math.pow(d_t, 2) * j_var)/1]
    ])
    
    x_ = P*x # type: ignore
    P_ = F*P*F.T + Q # type: ignore
    
    z = np.matrix([p_x, p_y, a_x, a_y]).T
    H = np.matrix([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1],
    ]) # type: ignore

    y = z - H*x_

    R = np.matrix([
        [0.0225, 0, 0, 0],
        [0, 0.0225, 0, 0],
        [0, 0, 0.1, 0],
        [0, 0, 0, 0.1],
    ]) # type: ignore

    S = H*P_*H.T + R
    K = P_*H.T*np.linalg.inv(S)
    x = x_ + K*y
    P = (np.identity(6) - K*H)*P_

