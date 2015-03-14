A gyroscope measures the amount of angular rotation about a single axis.  Since the gyroscope measures changes in angular rotation, rather than an  absolute angle, calculation of the actual current angle of that axis is estimated via [numerical integration](http://en.wikipedia.org/wiki/Numerical_integration) rather than an exact measurement.

Any Inertial Measurement Unit (IMU), including the nav6, that integrates a signal from a gyroscope will also accumulate error over time.   Accumulated error is due to several factors, including:

  * [Quantization noise](http://en.wikipedia.org/wiki/Quantization_(signal_processing)) (which occurs when an analog-to-digital converter (ADC) converts a continuous analog value to a discrete integral value)
  * Scale factor error (which occurs due to manufacturing errors causing a specified scale factor [e.g., 256 bits per unit G] to be incorrect)
  * Temperature instability (which occurs when a sensor is more or less sensitive to an input as temperature changes)
  * Bias error (which occurs because the value the sensor reports at 'zero' is not known well enough to 'subtract' that value out during signal processing)

Over time, these errors accumulate leading to greater and greater amounts of error.

Fortunately, an IMU uses other techniques to lessen the amount of error.  Errors in the Pitch and Roll values reported by the IMU tend to be extremely accurate over time since gyroscope values in the pitch/roll axes can be compared to the corresponding values from the accelerometer.  This is because when the IMU is not moving, the accelerometer data reflects only the linear acceleration due to gravity.

Things are not as simple when it comes to correcting for integration error in the Yaw (z) axis, since the accelerometer values in this axis are the same no matter how much z rotation exists.

To deal with this, several different algorithms have been developed, including:

  * Complementary filter
  * Extended Kalman filter (EKF)
  * Direction Cosine Matrix filter (DCM)

_Note:  See the [References](References.md) page for links to more information on these algorithms._

The Complementary and EKF filter algorithms are designed to process 3-axis accelerometer and 3-axis gyroscope values and yield yaw/pitch/roll values.

The Direction Cosine Matrix (DCM) filtering approach is similarly accurate and responsive as the EKF, however it requires information from a 3-axis magnetometer as well to work correctly.

Since the magnetometer on a FIRST FRC robot cannot be trusted due to electromagnetic interference, the DCM algorithm is not suitable in this case.

The complementary filter is a simple approach, and works rather well, however the response time is somewhat slower than the EKF, and the accuracy is somewhat lower.

For these reasons, the EKF is the preferred filtering algorithm to provide the highest performance IMU on a FIRST FRC robot.  However, the EKF algorithm is complex and difficult to understand, making it typically beyond the capabilities of many robotics engineers.  The nav6 circuit board uses the Invensense MPU-6050 IC, and this IC implements a proprietary algorithm which is believed to be an EKF.

Even so, yaw drift on the order of 1 degree per minute is typically observed with the MPU-6050 IMU.

What follows are some tips on how to deal with the yaw drift within the context of a FIRST FRC competition.

# Tips #

1)  Even though the nav6 magnetometer will likely give erroneous readings once the robot motors are energized, the magnetometer can potentially provide a stable reading during the moments before a FRC competition round.  Combined with the 1 degree per minute of drift in the yaw angles, it is possible to calculate the robots absolute orientation and maintain it through a match, subject to an accuracy limit of approximately 3 degrees over the 2 minute, 30 second match.

> yaw drift(degrees) at end of match = yaw drift (~1 degree/minute) x match length (2.5 minute) = **~3 degrees**

2)  Another approach which is supported by the IMU class is to periodically "re-zero" the IMU by applying an offset to the yaw angle reported by the IMU.  To use this approach, when the robot is in the correct orientation, a driver can press a button which causes an offset to be added so that the reported angle at that orientation is 0.