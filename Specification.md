The nav6 circuit board and official firmware provide inertial and magnetic measurements, with a range, accuracy and update rate as described on this page.

_Note that accuracy values are only valid after a start-up [calibration](SensorCalibration.md) period, during which time the nav6 circuit board must be held still._

## nav6 Key Features ##

  * Yaw, Pitch and Roll Angles
  * Tilt-compensated Compass Heading
  * Configurable Update Rate from 4 to 100Hz
  * Automatic Accelerometer/Gyroscope Calibration
  * Automatic Hard-Iron Magnetometer Calibration

## nav6 Electrical/Interface Specifications ##


|Voltage:                  |6-16V DC|
|:-------------------------|:-------|
|Current Consumption:      |200 millamps|
|Communications Interface: |RS-232|
|Power Connector:          |JST PH 2-pin connector|
|RS-232 Connector:         |DB9, Female|


## nav6 Microcontroller Specifications ##


|Model:                    |Atmel ATMEGA328P|
|:-------------------------|:---------------|
|Clock Frequency:          |16 Mhz|


## nav6 Official Firmware Key Specifications ##


|Startup Calibration Period:     |20 seconds|
|:-------------------------------|:---------|
|Gyro Sensitivity                |+/- 2000 degrees/sec|
|Accel Sensitivity               |+/- 2 g|
|Magnetometer Sensitivity:       |1.3 Gauss|
|Gyro Low-pass Filter Bandwidth: |42Hz|
|Yaw angle accuracy:             |~1 degree of drift/minute|
|Serial Port Update Rate:        |4-100 Hz|
|Digital Motion Processor (DMP) Update Rate:                |4-100 Hz|
|Magnetometer Raw Update Rate:   |15 Hz|


## MPU-6050 Key Specifications ##


|Gyro Sensitivity Range             |+/- 250,500,1000 or 2000 degrees/sec|
|:----------------------------------|:-----------------------------------|
|Accelerometer Sensitivity Range    |+/- 2,4,8 or 16 g|
|Motion Processor Update Rate       |Up to 1KHz|
|ADC Resolution                     |16 bits|


## HMC5883L Key Specifications ##


|Angular accuracy:                   |1-2 degrees|
|:-----------------------------------|:----------|
|ADC Resolution                      |12 bits|
