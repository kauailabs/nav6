The nav6 includes automatic calibration of the gyroscopes/accelerometers, the yaw offset, and also "Hard Iron" calibration of the magnetometer.  This calibration occurs when power is applied to the nav6, and is subject to certain limitations.

# Gyroscope/Accelerometer Calibration #

The Invensense MPU-6050 Digital Motion Processor (DMP) includes several automatic calibration methods.  The simplest-to-use calibration method is used in the nav6, which will automatically calibrate the gyroscope and accelerometer whenever the nav6 is still for a period of 8 seconds.  Assuming the nav6 is held still after power is applied, the entire calibration process should complete in 20 seconds.

In a typical FIRST FRC competition match, this means that the gyroscope/accelerometers are automatically calibrated at the beginning of a match, between the time when the robot is powered on and when the match actually starts.

# Yaw Offset Calibration #

Additionally, the Yaw offset is automatically calibrated during the nav6 initialization so that whatever direction the "front" of the nav6 board is pointed during this calibration period will be considered "0 degrees".

# Magnetometer Calibration #

## Hard Iron Calibration ##

The HMC5883L Magnetometer includes a built-in "strap driver" circuit which is used to automatically calibrate the gain for each of the X, Y and Z axis magnetometers.  The nav6 firmware automatically calculates and stores these gain values each time the nav6 is powered on.

This calibration performs what is sometimes referred to as "Hard Iron" calibration.

## Soft Iron Calibration ##

Note that the nav6 firmware's automatic magnetometer calibration feature does not include "Soft Iron" calibration which would correct for additional effects due to soft metals that may be near the magnetometer that will tend to impact the magnetometer accuracy.

This is primarily because Soft Iron calibration must be performed after the magnetometer has been installed, since the soft iron effects are typically caused by soft metals in the environment near the sensor.

Soft Iron calibration may be implemented in the robot control software by using the nav6 "Raw" update mode.  In this mode, the IMU yaw angle value and the corresponding magnetometer X and Y values can be correlated and a correction factor can be applied, as described in [this document](http://www.sensorsmag.com/sensors/motion-velocity-displacement/compensating-tilt-hard-iron-and-soft-iron-effects-6475).