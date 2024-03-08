from decawave_1001_uart import DwmDistanceAndPosition, Decawave1001Driver
import numpy as np

def z_factory(x, anchors: list[DwmDistanceAndPosition]):
    px = x[0, 0]
    py = x[1, 0]
    z = []
    for anchor in anchors:
        ax = anchor.position().position()[0] * .001
        ay = anchor.position().position()[1] * .001
        z.append([((x[0, 0] - ax)**2 + (x[1, 0] - ay)**2)**.5])
    return np.array(z)

x = np.array([
    [0],
    [0],
    [0],
    [0],
    [0],
    [0]
])

dwm = Decawave1001Driver("/dev/ttyACM0")
loc = dwm.get_loc().get_anchor_distances_and_positions()
print(state_factory(x, loc))