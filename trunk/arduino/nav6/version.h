#define NAV6_FIRMWARE_VERSION_MAJOR 	2
#define NAV6_FIRMWARE_VERSION_MINOR		1

// Version Log
//
// 1.0:  Initial Release (5/10/2013)
//
// 2.0:  Redesigned to use the Invensense Motion Driver 5.1
//       rather than the MPU6050 library from the I2CDevLib.
//
// 2.1:  Added "Raw" update mode, tilt compensation for compass
//       and diagnostic enhancements.
//     
//       Added I2C Bus reset logic to the setup() initialization
//       code.
//
// - Known issues
//
// None.