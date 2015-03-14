# Overview #

In order to communicate sensor data to a client (e.g., a cRio robot controller) the nav6 IMU software uses a custom protocol.  This protocol defines messages which are sent between the IMU and the client over a serial interface, and also includes an error detection capability to ensure corrupted data is not used by the client.

In order to enhance readability during debugging, the nav6 IMU protocol uses ASCII characters.

[Source code](https://code.google.com/p/nav6/source/browse/trunk/arduino/nav6/IMUProtocol.h) that implements the nav6 IMU protocol is provided to simplify adding support for the nav6 IMU protocol to a software project.  The C++ version of the protocol source code is contained within a single .h file; the Java version is contained within a .jar file (the source is also available for this Jar file).  As of the date of this writing, this source code is portable to both the Arduino platform and the cRio processor.

# Message Structure #

Each nav6 IMU protocol message has the following structure:


|Start of Message|Message ID|Message Body|Message Termination|
|:---------------|:---------|:-----------|:------------------|
|1 byte|1 byte|length is message-type dependent|4 bytes|


## Data Type Formatting ##

In keeping with the ASCII encoding of the nav6 IMU protocol, the following encoding is used for message elements:


|Data Type|Encoding|Example|
|:--------|:-------|:------|
|Float|(Sign)(100s)(10s)(1s).(10ths)(100ths)|'-132.96'|
|8-bit Integer|(HighNibble)(LowNibble)|'E9'|
|16-bit Integer|(HighByte,HighNibble)(HighByte,LowNibble)(LowByte,HighNibble)(LowByte,LowNibble)|'1A0F'|


# Start of Message #

Each IMU message begins with "start of message" indicator (a '!' character), which indicates that the following bytes contain a message.

# Message ID #

Immediately following the "start of message", a one-byte Message ID character, which may be one of the following:


|ID|Message Type|
|:-|:-----------|
|'y'|Yaw/Pitch/Roll/Compass Heading|


# Message Body #

## Yaw/Pitch/Roll/Compass Heading Update Message ##

The Yaw/Pitch/Roll/Compass Heading Update message indicates the nav6 IMU's current pose and heading, in units of degrees, as follows:


|Byte Offset|Element|Data Type|
|:----------|:------|:--------|
|0 |Yaw (degrees from -180 to 180)|Float|
|7 |Pitch (degrees from -180 to 180)|Float|
|14|Roll (degrees from -180 to 180)|Float|
|21|Compass Heading (degrees from 0 to 360)|Float|


# Message Termination #

The final four bytes of each IMU protocol message contain an unsigned 8-bit checksum followed by a carriage return and then a line feed character.

## Checksum ##

The checksum is calculated by adding each byte of the message except the bytes within the Message Termination itself.  The checksum is accumulated within an 8-bit unsigned byte.

## New Line ##

The carriage return and line feed characters are present at the end of the message so that when the message is displayed in a console window, a new line will be inserted in the console at the end of the message.