**_Will nav6 work with the new National Instruments RoboRio that will be used starting in the 2015 FRC Season?_**

> Yes.  The documentation currently available for the National Instruments RoboRio indicates a RS-232 Serial Port is provided (although a DB9 connector is not used).  Additionally,  the nav6 can be connected to the National Instruments RoboRio via one it its 2 USB host ports.  To accomplish this, a RS-232 to USB converter cable is needed.

> KauaiLabs will release software libraries for the RoboRio soon after the RoboRio is released.

> Note that a RS-232 to USB converter cable is inexpensive, and is also very useful - since with this cable the nav6 IMU can be re-programmed using the Arduino IDE.

**_Did Invensense finally publicly release a description of the DMP (Digital  Motion Processor) and interface specs, or are you using what other people reverse engineered a while ago?_**

> The nav6 firmware uses the officially released Invensense MotionDriver version 5.1.  Kauai Labs has ported this driver to work correctly on the nav6 ATMEGA328 micro-controller.

**_Since many robots use the 1 serial port on the cRIO for CAN, what other interfaces options will be available for NAV6? (I2C)?_**

> For 2014, the RS-232 Serial port interface is the only method for connecting the nav6 to the CRio.

> One possible approach is to replace the existing serial port interface to the CAN bus with an ethernet-based CAN bridge such as the [2CAN](http://www.crosstheroadelectronics.com/2CAN.htm).

> As noted above, the nav6 will work with the new RoboRio (which will used in FIRST robotics starting in 2015) via one of the USB Host interfaces.

**_Aren't the magnetometer (compass heading) readings unreliable when the nav6 is used on a Robot with powerful motors?_**

> Yes, this is correct.  If the nav6 is mounted nearby any energized motors, the magnetometer's ability to measure the (weak) earth's magnetic field is severely diminished.

> However, at the beginning of each FIRST FRC match, the robot is turned on for about a minute before the match begins.  During this time period, the motors are not energized and thus do not add magnetic interference that would disturb the magnetometer readings.

> Magnetometer readings taken at the beginning of a match, when combined with the IMU yaw measurements, can allow the robot's pose to be maintained throughout the match.

**_Why do the Yaw angles provided by the nav6 IMU drift over time?_**

> The short answer is that the yaw angle is calculated by integrating reading from a gyroscope which measures changes in rotation, rather than absolute angles.  Over time, small errors in the rotation measurements build up over time.  The nav6 IMU features sophisticated digital motion processing that limits this error in the yaw angle of about 1 degree per minute.

> For further details, please see the [Yaw Drift](YawDrift.md) wiki page.