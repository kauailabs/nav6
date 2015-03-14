The nav6 Factory Test Procedure verifies correct operation of the circuit board and it's key components.  The nav6 Factory Test Procedure is performed in the factory to verify initial correct operation, and may be run at a later point in time to re-verify correct option.

# Test Procedure #

  1. Ensure the “nav6 Factory Test” software is running on the nav6 circuit board.  See the [Customization](Customization.md) page for instructions on loading the Factory Test firmware onto the nav6.
> 2)  Apply power (6V – 12V DC) to the nav6 Power connector.  This connector is a 2-pin JST PH connector.
    * Verify that the two Red power LEDs are on and solidly-lit.
    * If the two Red power LEDs are not on or solidly lit, either a valid power source is not connected to the power connector, or one of the voltage regulators is malfunctioning.
> 3) After power is applied, the nav6 bootloader will quickly flash the “STATUS” Led 3 times.
    * If these quick flashes are not seen during startup, this indicates the ATMEGA328 microcontroller is not correctly installed, the bootloader is not loaded, or is malfunctioning.
> 4) Next, after a brief delay of a second or so, the Nav6 “STATUS” Led will quickly flash 3 more times.  This indicates the Factory Test is beginning.
    * If the “STATUS” Led does not, for a second time, quickly flash 3 times at startup, this indicates the “nav6 Factory Test” software is not correctly installed on the ATMEGA328 processor.
> 5) The “nav6 Factory Test” software will next verify communication on the I2C bus with and correct operation of the following components:
    * MPU-6050 Gyro/Accelerometer
    * HMC5883L 3-axis Magnetometer
> 6) The “Nav6 Factory Test” software also provides detailed operational status via its serial port.
    * During the test, the green “TX” LED will flash whenever messages are transmitted over the serial port.
    * To view the detailed messages, connect a RS-232 to USB converter cable to a PC and connect a serial terminal using the settings:
> > > Baud Rate:  57600
> > > Data Bits:  8
> > > Parity:  None
> > > Stop Bits:  1

> 7) When the Factory Test completes, the result will be displayed on the Nav6 Status LED.
    * If successful, the Status LED will be steady on.
    * If unsuccessful, the Status LED will repeatedly flash quickly a number of times, then pausing a few seconds before repeating the sequence of flashes.  The number of flashes corresponds to an error code, as follows:
| Code	| Error	| Typical Cause |
|:-----|:------|:--------------|
|1	|MPU-6050 not responding on I2C bus	|MPU-6050 missing, incorrectly installed, or malfunctioning.  Or, I2C bus failure.|
|2	|HMC5883L not responding on I2C bus	|HMC5883L missing, incorrectly installed, or malfunctioning.  Or, I2C bus failure.|
|3	|MPU-6050 Initialization failed	|MPU-6050 malfunctioning|
|4	|MPU-6050 Gyro Test Failed	|MPU-6050 on-chip Gyroscope(s) malfunctioning|
|5	|MPU-6050 Accelerometer Test Failed	|MPU-6050 on-chip Accelerometer(s) malfunctioning|
|6	|HMC5883L Not responding on I2C bus	|HMC5883L malfunctioning|
|7	|HMC5883L Calibration Failed	|HMC5883L malfunctioning|
|8	|MPU-6050 Interrupt not Received	|Interrupt Line between MPU-6050 and ATMEGA328 not connected; MPU-6050 malfunctioning|
|9	|HMC5883L Interrupt not Received	|Interrupt Line between HMC5883L and ATMEGA328 not connected; HMC5883L malfunctioning|

8) NOTE:  In certain circumstances, the Factory test will "hang" after step 4.  This occurs because one of the devices on the I2C buses is in a bad state, or the I2C bus communication is "flaky" (as when a solder connection on the circuit board is less than ideal).  When this "hang" occurs, all of the green LEDs on the nav6 will be off and will not light up again until the device is reset.