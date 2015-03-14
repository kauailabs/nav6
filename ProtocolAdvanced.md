In addition to the Basic nav6 [Protocol](Protocol.md), the nav6 firmware implements several messages which may be useful to more advanced users.

# Advanced Messages #

## Quaternion Data Update Message ##

The Quaternion Data update message communicates the Quaternions calculated by the Digital Motion Procssor, as well as some of the raw sensors readings; this update message may be useful when the Yaw/Pitch/Roll/Compass Heading provides insufficient information:

  * With the quaternions and the raw acceleration data, the world linear acceleration (the true acceleration, with the gravity component removed) can be calculated.  Note that the acceleration raw data has already had calibration applied.

  * Additionally, with the raw magnetometer data, it is possible to implement soft iron calibration.  Note that the raw magnetometer data has already had hard iron calibration applied.


|Byte Offset|Element|Data Type|
|:----------|:------|:--------|
|0 |Quaternion W (15-bits, signed)|16-bit Integer|
|4 |Quaternion X (15-bits, signed)|16-bit Integer|
|8 |Quaternion Y (15-bits, signed)|16-bit Integer|
|12|Quaternion Z (15-bits, signed)|16-bit Integer|
|16|Acceleration X (16-bits, signed)|16-bit Integer|
|20|Acceleration Y (16-bits, signed)|16-bit Integer|
|24|Acceleration Z (16-bits, signed)|16-bit Integer|
|28|Magnetometer X (12 bits, signed)|16-bit Integer|
|32|Magnetometer Y (12 bits, signed)|16-bit Integer|
|36|Magnetometer Z (12 bits, signed)|16-bit Integer|
|40|Temperature (Centigrade degrees)|Float|


## Gyro Data Update Message ##

The Gyro Data update message communicates the raw gyro, accelerometer, magnetometer and temperature data.  This data bypasses the Digital Motion Processor, and allows the individual sensors to be used directly without any intervening processing.  This can allow the following types of use:

  * Access to instantaneous measures of angular velocity in each of the X, Y and Z axes, provided by the tri-axial gyroscopes.  Note that the gyroscope data has already had calibration applied.

  * Additionally, with the raw magnetometer data, it is possible to implement soft iron calibration.  Note that the raw magnetometer data has already had hard iron calibration applied.


|Byte Offset|Element|Data Type|
|:----------|:------|:--------|
|0 |Gyro X (15-bits, signed)|16-bit Integer|
|4 |Gyro Y (15-bits, signed)|16-bit Integer|
|8 |Gyro Z (15-bits, signed)|16-bit Integer|
|12|Acceleration X (16-bits, signed)|16-bit Integer|
|16|Acceleration Y (16-bits, signed)|16-bit Integer|
|20|Acceleration Z (16-bits, signed)|16-bit Integer|
|24|Magnetometer X (12 bits, signed)|16-bit Integer|
|28|Magnetometer Y (12 bits, signed)|16-bit Integer|
|32|Magnetometer Z (12 bits, signed)|16-bit Integer|
|36|Temperature (Centigrade degrees)|Float|


## Stream Configuration Command ##

By default, the nav6 IMU begins transmitting YPR Updates upon power up.  The Stream Configuration Command is sent to the nav6 in order to change the type of Streaming Update message the nav6 will transmit to the client.


|Byte Offset|Element|Data Type|
|:----------|:------|:--------|
|0 |Stream Type (YPR or Raw)|8-bit ASCII Character|
|1 |Update Rate (Hz) - Valid range:  4-100|8-bit Integer|



|Stream Type|Description|
|:----------|:----------|
|'y'|Yaw, Pitch, Roll & Compass Heading Update|
|'q'|Quaternion Data Update|
|'g'|Gyro Data Update|


## Stream Configuration Response ##

Whenever a Stream Configuration Command is received by the nav6, the nav6 responds by sending a Stream Configuration Response message, which is formatted as follows:


|Byte Offset|Element|Data Type|
|:----------|:------|:--------|
|0 |Stream Type (YPR or Raw)|8-bit ASCII Character|
|1 |Gyroscope Full Scale Range (Degrees/sec)|16-bit Integer|
|5 |Accelerometer Full Scale Range (G)|16-bit Integer|
|9 |Update Rate (Hz)|16-bit Integer|
|13|Calibrated Yaw Offset (Degrees)|Float|
|20|Calibrated Quaternion W Offset (15-bits, signed)|16-bit Integer|
|24|Calibrated Quaternion X Offset (15-bits, signed)|16-bit Integer|
|28|Calibrated Quaternion Y Offset (15-bits, signed)|16-bit Integer|
|32|Calibrated Quaternion Z Offset (15-bits, signed)|16-bit Integer|
|36|Flags|16-bit Integer|



|Flag value|Desription|
|:---------|:---------|
|0, 1|Startup Calibration in progress|
|2 |Startup Calibration complete|
