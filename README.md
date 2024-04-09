## UWB/IMU Fusion
A collection of scripts for Indoor localization using RF UWB and IMU sensor fusion, implemented in python with a focus on simple setup and use.

### Hardware Integration
The project makes use of two main sensors:

#### Decawave DWM1001-Dev
This sensor is non-negotiable, you'll need this one.  Data is pulled from the sensor over USB using the incuded UART API in the stock PANS firmware.  Simply plug in a sensor and specify the USB endpoint to get started.

#### Inertial Measurement Unit (IMU)
This project was written for use with the Kuaui Labs Navx2 Micro.  As such, all filter tuning has been performed with this sensor in mind.
Replacing the sensor is made as simple as possible, but still left to the user.  To use another IMU, just modify the main loop to change what sensor data is used in the filter.

### Getting Started

### Example
```
```