[![](http://www.kauailabs.com/store/image/data/nav6_Banner3_build_selfguided_robots2.png)](http://www.kauailabs.com/store/index.php?route=product/product&product_id=50)

_NOTE:  the new [navX MXP Robotics Navigation Sensor](https://code.google.com/p/navx) is now available; if you are using a National Instrument RoboRIO, this provides additional features including RoboRIO I/O Expansion and 9-axis headings (AHRS functionality) along with enhanced calibration algorithms and tools._

**The nav6 [inertial measurement unit (IMU)](http://en.wikipedia.org/wiki/Inertial_measurement_unit) brings sophisticated inertial navigation to air, land and sea robots, including the [FIRST Robotics Challenge](http://www.usfirst.org/frc) (FRC).**

nav6 rapidly calculates yaw, pitch, roll and compass heading, as well as linear acceleration, enabling features including:

  * ["Field-oriented" drive mode](FieldOrientedDriveExample.md)
  * [Robot balancing](AutoBalancingExample.md)
  * [Auto-rotation to an absolute or relative angle](RotateToAngleExample.md)
  * [Motion / No-Motion Detection](MotionDetection.md)
  * Bump Detection

## Key Features ##

  * Yaw, Pitch and Roll Angles
  * Tilt-compensated Compass Heading
  * Configurable Update Rate from 4 to 100Hz
  * Automatic Accelerometer/Gyroscope Calibration
  * Automatic Hard-Iron Magnetometer Calibration
  * Libraries and example code for integration onto a FRC Robot, in C++, Java and LabView
  * Completely Open Source:  Source Code, Schematics/Bill of Materials and Enclosure Design files (for 3D printer)
  * Arduino-compatible - programmable with free Arduino IDE

### **_Available now at the [Kauailabs store](http://www.kauailabs.com/store/index.php?route=product/product&product_id=50)._** ###

[![](https://nav6.googlecode.com/svn/trunk/images/nav6_purple_1_small.jpg)](http://www.kauailabs.com/store/index.php?route=product/product&product_id=50)

## Overview ##

The nav6 features the powerful Invensense [MPU-6050](http://invensense.com/mems/gyro/mpu6050.html) IC which includes a 3-axis accelerometer, a 3-axis gyroscope and an on-chip "Digital Motion Processor" (DMP).  The nav6 employs sophisticated motion processing algorithms provided by the DMP included with the MPU-6050.  The result:  highly accurate tip/tilt, and accurate yaw that exhibits minimal drift of approximately 1 degree per minute.  The sensor provides a update rate up to 100Hz, suitable for use in robotic systems.

The nav6 also includes a Honeywell [HMC5883L](http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf) 3-axis magnetometer.  Although the magnetometer compass readings are often unreliable when robot motors are energized, it is useful for establishing absolute orientation at the beginning of a competition.  Combined with the nav6 "pose" and this initial orientation, a robot's absolute orientation throughout a competition match can be maintained.

The nav6 is managed by an Atmel ATMEGA328P microcontroller, which offloads processing from the host.  The nav6 is Arduino-compatible, and the nav6 firmware can be customized via the free Arduino Integrated Development Environment (IDE).

The nav6 has been designed specifically to enable [easy integration](FRCRobotInstallation.md) into a FRC robotics control system:  it's power connection connects directly to an unregulated 12V output on the Power Distribution Board and it's data connection connects directly to the cRio serial port.  Additionally, [source code for easy integration into the FRC cRio controller](https://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Fcrio) is also provided.

## Fully Open Source ##

The nav6 project is completely open-source and includes the following freely-available components:

### _Board schematics/layout_ ###

  * [board schematics](https://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Fschematics) (developed in Eagle 6.4.0, freely available at http://www.cadsoftusa.com/) and bill of materials.

### _Arduino-compatible firmware source code_ ###

  * As the nav6 behaves almost exactly like an Arduino UNO Rev 3 board, it can be programmed via the [Arduino IDE](http://arduino.cc/en/main/software).
  * The [nav6 firmware Source Code](https://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Farduino) can be built and downloaded to the nav6 via the Arduino IDE.

NOTE: To interface the Arduino IDE to the nav6 from a computer without a RS-232 serial port, you can use an inexpensive [USB-to-RS-232 converter cable](http://www.kauailabs.com/store/index.php?route=product/product&product_id=53).

### _cRio client source code_ ###

  * The [nav6 cRio library source code](https://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Fcrio) is compatible with the FRC WPI Library (C++ and Java variants) as well as LabView, which runs on the FIRST robotics cRio and RoboRIO platforms.  This code allows robot control system code to acquire yaw/pitch/roll/compass-heading values from the board in real-time.

### _Enclosure design files_ ###

A custom enclosure to protect and mount the nav6 IMU can be made with a 3D printer using provided [Enclosure](Enclosure.md) design files.

[![](http://www.kauailabs.com/wp-content/uploads/2013/12/cropped-kauailabsbanner_990_x_18012.png)](http://www.kauailabs.com)