# GyroCam2

A 2-axis servo-based gimbal for pitch and roll using MPU-6050 as sensor

Download the Kalman libraries from https://github.com/TKJElectronics/KalmanFilter and install to your Arduino/libraries folder.

MPU-6050 connected to I2c pins (A4 and A5) SDA and SCL respectively.

Pitch servo to D10 and Roll servo to D11.

Servos and MPU use 5 volts from Arduino and common ground.

NOTE: You can adjust the resting servo angle from the sketch.

TODO: 
Add pots for servo calibration.
Add magnetometer for yaw (3D)


