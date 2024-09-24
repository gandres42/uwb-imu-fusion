## UWB/IMU Fusion
Indoor 3D localization with RF UWB and IMU sensor fusion using an Extended Kalman Filter, implemented in python with a focus on simple setup and use.
### Hardware Integration
The project makes use of two main sensors:
#### Decawave DWM1001-Dev
This sensor is non-negotiable, you'll need this one.  Data is pulled from the sensor over USB using the incuded UART API in the stock PANS firmware.  Simply plug in a sensor and specify the USB endpoint to get started.
#### Inertial Measurement Unit (IMU)
This project was written for use with the Kuaui Labs Navx2 Micro.  As such, all filter tuning has been performed with this sensor in mind.  Replacing the sensor is left to the user, just modify the main loop to change what sensor data is used in the filter.  However, to obtain accurate results the IMU covariance values will need to be updated.
## Filter Design
![](https://lh7-rt.googleusercontent.com/slidesz/AGV_vUcXnCEcinspD_iif3K6a8k0lXE4ZYKAPhA8TNEPPJsXF5Cxb3i7aWFrOO5Ux-Ziyat5fRWuRjJT_j1eYOBircowyrEUCTtVK04x3gM969NijdpHUf21uacVBzDi5G-CnC6KnobLsS3MaEIVtf7LSylhk0NmML0R=s2048?key=xoR_5xepWiR7yxKoPnOZrA)
## Limitations
Non-line-of-sight between anchors and the tracking tag has a significant impact on accuracy.  While this could be improved by using more anchors, PANS firmware is limited to tracking distance from 4 anchors at once.  Further improvement would therefore require writing custom firmware for the DWM1001-Dev, which is a bit beyond the scope of this project.
