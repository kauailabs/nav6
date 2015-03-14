If you need access to advanced features enabled by the nav6 IMU, such as access to the raw sensor data, world linear acceleration with gravity removed, motion/no-motion detection and access to the sensors temperature data, you can use the IMUAdvanced class instead of the IMU class.

_The IMUAdvanced class is included in both the C++ and the Java libraries._

The capabilities of the IMUAdvanced class do come at a cost - additional computation on the processor executing the IMUAdvanced class, as compared to the IMU class.

The IMUAdvanced class adds the following methods:

float IMUAdvanced::GetWorldLinearAccelX()

float IMUAdvanced::GetWorldLinearAccelY()

float IMUAdvanced::GetWorldLinearAccelZ()

bool  IMUAdvanced::IsMoving()

float IMUAdvanced::GetTempC()