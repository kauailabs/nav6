To integrate the nav6 IMU into cRio-based robot control software, source code for a [C++](https://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Fcrio%2Fc%2B%2B%2Fnav6SimpleRobotExample%253Fstate%253Dclosed) and a [Java](https://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Fcrio%2Fjava) class named **IMU**, which is based upon the WPI Library, is provided.

**You can either choose to [checkout](https://code.google.com/p/nav6/source/checkout) the source code with subversion, or you can choose to download the [latest build](https://nav6.googlecode.com/svn/trunk/nav6.zip) of the libraries.**

_Note:  the example code on this page demonstrates using the C++ version of the IMU class, however the Java version is identical, except that the first word of each Java class method is lower case._

_Note:  if you need access to advanced features enabled by the nav6 IMU, such as access to the raw sensor data, world linear acceleration with gravity removed, motion/no-motion detection and access to the sensors temperature data, you can use the IMUAdvanced class instead of the IMU class._

# Instantiating an IMU object #

To instantiate an IMU object, the IMU constructor must be invoked and provided a pointer to an instance of the WPI Library **SerialPort** class.

```
    // Instantiate Serial Port object

    // Serial settings are:
    // - Baud Rate:  57600
    // - Data Bits:  8
    // - Stop Bits:  1
    // - Parity:     None

    SerialPort *imu_serial_port = new SerialPort(57600);

    // Instantiate IMU object

    short update_rate_hz = 50; // valid values:  4-100
    IMU *imu = new IMU(imu_serial_port, update_rate_hz);

    // Add the IMU to the Live Window
    // Since the IMU inherits from LiveWindowSendable,
    // the live window will automatically be updated with
    // the current Yaw angle

    lw->AddSensor("DriveSystem", "Gyro", imu);

```

# Verifying Communication #

Several methods are provided to help verify proper communication with the nav6 IMU:

> IMU::IsConnected() - indicates whether data is successfully being received from the IMU.  Given the default update rate of 100Hz, the IMU::IsConnected() method should return true within about 10 ms after the IMU object is instantiated.

> IMU::GetUpdateCount() - can be used to determine if new yaw/pitch/roll values have been received from the nav6 IMU since the last time this method was invoked.  Given the default update rate of 100Hz, the value returned from IMU::GetUpdatedCount() should increment once every 10 milliseconds.

> IMU::GetByteCount() - can be used to determine if any bytes are being received over the SerialPort from the nav6.  The value returned from IMU::GetByteCount() indicates the number of bytes received over the serial port since the IMU object was instantiated.

# Calibration State #

> IMU::IsCalibrating() - if this function returns true, the nav6 IMU is performing a gyroscope/accelerometer/magnetometer calibration procedure.  This procedure take 20 seconds, and begins when power is applied to the nav6 IMU.  During this time, the nav6 IMU should be still.  At the end of the calibration period, the nav6 computes a yaw offset which is used until the nav6 is powered on again.

# Accessing Yaw, Pitch, Roll and Compass Heading values #

Four methods are provided to access the most recent Yaw, Pitch, Roll and Compass Heading values from the nav6 IMU:

  * IMU::GetYaw()
  * IMU::GetPitch()
  * IMU::GetRoll()

> These methods return the latest angle from the Digital Motion Processor (DMP).  The returned value is a floating point number from -180.0 to 180.0.

  * IMU::GetCompassHeading()

> The GetCompassHeading() method returns the latest angle from the Magnetometer.  The returned value is a floating point number from 0 to 360.

# Zeroing the Yaw Angle #

To reset the yaw value to the current angle the nav6 IMU is currently pointing, invoke the IMU::ZeroYaw() method.  Once invoked, the IMU::GetYaw() method will return 0 until the IMU is rotated, at which point the changes will be relative to the newly set "zero" heading.

# Using the nav6 IMU data for PID Control #

The IMU Class inherits from the WPI Library PIDSource class, and the current yaw angle is returned from the PIDGet() method.  By using the nav6 IMU as a PID Source, software can be written that would, for instance, [turn the robot to a particular angle](RotateToAngleExample.md).

# Viewing nav6 value/status on the Smart Dashboard #

```
        // Display IMU Status

	bool imu_connected = imu->IsConnected();
	SmartDashboard::PutBoolean( "IMU_Connected", imu_connected);
	SmartDashboard::PutNumber("IMU_Update_Count", imu->GetUpdateCount());
	SmartDashboard::PutNumber("IMU_Byte_Count", imu->GetByteCount());

        // Display IMU Values

	SmartDashboard::PutNumber("PitchAngle", imu->GetPitch());
	SmartDashboard::PutNumber("RollAngle", imu->GetRoll());
```