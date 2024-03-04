import kalman
import numpy as np

filter = kalman.Extended([(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3)])
x = np.array([
    [1],
    [1],
    [1],
    [0],
    [0],
    [0]
])
filter.Hx(x)
