import time
from networktables import NetworkTables
from utils.dwm import dwm1001
from utils.kalman import Fusion, UWB
from utils.navx import ahrs

NetworkTables.initialize()
nt = NetworkTables.getTable("localization")

dwm = dwm1001("/dev/ttyACM0")
imu = ahrs("/dev/ttyACM1")

while not imu.is_calibrated():
    time.sleep(0.01)


prev_imu = time.monotonic()
prev_dwm = time.monotonic()
init_pos = dwm.position()
plain_ekf = UWB(init_pos.px, init_pos.py, init_pos.pz)
fusion_ekf = Fusion(init_pos.px, init_pos.py, init_pos.pz)
anchors = []
try:
    while True:
        if time.monotonic() - prev_dwm >= 0.1:
            anchors = dwm.anchors()
            prev_dwm = time.monotonic()
        if time.monotonic() - prev_imu >= .005:
            plain_ekf.dwm_update(anchors)
            fusion_ekf.dwm_update(anchors,
                           imu.get_accel_x() * 9.8, 
                           imu.get_accel_y() * 9.8, 
                           imu.get_accel_z() * 9.8)
            prev_imu = time.monotonic()
            nt.putNumber('dwm_x', round(plain_ekf.get_x()[0, 0], 2))
            nt.putNumber('dwm_y', round(plain_ekf.get_x()[1, 0], 2))
            nt.putNumber('fuse_x', round(fusion_ekf.get_x()[0, 0], 2))
            nt.putNumber('fuse_y', round(fusion_ekf.get_x()[1, 0], 2))
            
except KeyboardInterrupt:
    dwm.close()