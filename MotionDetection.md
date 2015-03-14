Detecting motion/no-motion can be simply detected by determining if a body's linear acceleration exceeds a small threshold.

Using the data from accelerometers, this is not as easy as it seems, _since accelerometer readings contain both acceleration due to gravity as well as acceleration due to a body's motion_.  One method for dealing with this is to use a high-pass filter, which lets quickly-changing information through but blocks information that doesn't change frequently.

However, a more comprehensive and reliable approach is to subtract the acceleration due to gravity (this information can be extracted from the Quaternions provided by the nav6 IMU) from the raw acceleration values from the nav6 IMU.  The result is the "world linear acceleration", representing the actual amount of acceleration due to motion.

The IMUAdvanced class contains an IsMoving() method which returns the result of exactly this calculation.  Therefore, the code for detection motion (or lack thereof) is as simple as:

```
IMUAdvanced * advanced_imu;  // Must be initialized
if ( advanced_imu->IsMoving() ) {
  // Moving
}
else {
  // Not Moving
}
```