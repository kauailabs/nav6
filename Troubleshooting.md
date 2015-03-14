This page includes trouble-shooting tips for the nav6 when used with the RoboRio and the !CRio.

# RoboRio #

## USB:  Verify USB Serial Port Binding ##

If connecting the RoboRio to the nav6 via one of the !RoboRIO USB host ports (using a USB-serial adapter cable), verify that the nav6's USB interface is correctly bound to !RoboRIO Serial Port ASRL3:INTR, as shown below.

This information is displayed in the RoboRio's webdashboard.  For instance, if you have connected your RoboRio to your PC via USB cable (connected to the RoboRio's USB Device Port), the web dashboard can be viewed within a web browser (in this case, at http://172.22.11.2).

![https://navx-mxp.googlecode.com/svn/trunk/images/roborio_webdashboard_ports.png](https://navx-mxp.googlecode.com/svn/trunk/images/roborio_webdashboard_ports.png)

## RS-232:  Verify RoboRIO Console Out is disabled ##

If you are connecting the nav6 to the RoboRio's RS-232 port, ensure that the "Console Out" feature of the RoboRio is disabled.  If the "Console Out" feature of the !Roborio is enabled, no other communication of the RS-232 port is permitted.

This setting can be edited via the [RoboRio Web Dashboard](http://wpilib.screenstepslive.com/s/4485/m/24166/l/262266-roborio-webdashboard)

Note that this setting can also be changed via the RoboRio imaging tool. Note that if you ever have to reimage the roboRIO, the imaging tool will automatically turn the console out back on.

## RS-232: Verify TX, RX and GND are connected correctly ##

TX on the nav6 side connects to RX on the roborio side, and vice versa. Additionally,  GND on the nav6's RS-232 connector must be connected to GND on the roborio's RS-232 connector.

To verify that the nav6 is sending data, look at the green TX led on the nav6 circuit board.  During startup, the RoboRio nav6 library will send a message to the nav6, so during this time you should also see a very brief flash of the RX led on the nav6 circuit board.

# CRio #

1) **_Ensure the CRio CONSOLE OUT Dipswitch is off_**

If the CRio CONSOLE OUT Dipswitch is ON, the Serial Port will be used exclusively by the CRio to output debug/status messages to a serial terminal.  In this case, the Serial Port cannot be used to communicate to the nav6 Open Source IMU.

2) **_Verify your Serial Cable is straight-through (not a null modem cable)_**

Pin 2 on the CRio Serial Port Connector must be connected to Pin 2 on the nav6 Serial Port Connector.  CRio Serial Port Connector Pin 3 must be connected to nav6 Serial Port Connector Pin 3.

Some Serial Cables, known as "null-modem" cables, will connect Pin 2 on one side to Pin 3 on the other, and will connect Pin 3 on one side to Pin 2 on the other.

Attempting to use a null-modem cable, or any other cable that does not connect Pins 2 and 3 "straight through" will likely result in frustration, and is not recommended.

3) **_Select an update rate that ensures sufficient CPU bandwidth._**

On certain platforms, such as a CRio running Java, running the nav6 at a high update rate (e.g., 100Hz) may result in excessive CPU utilization; if this is the case, reduce the update rate to an appropriate setting for your environment.

The FRC Driver Station's "Chart" Tab can be used to monitor the CPU consumption, providing feedback that will help if tuning is required.

4) **_Ensure the "Black Jaguar Serial Bridge" CAN Plugin is not installed._**

This can be verified by running the FRC cRIO Imaging tool, and verifying that the CAN Driver Plugin is not configured to load the Black Jaguar Serial Bridge.  The Black Jaguar Serial Bridge uses the cRIO Serial Port, and the nav6 requires exclusive use of the Serial Port in order to function correctly.