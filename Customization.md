The nav6 IMU is fully open source, both hardware and software.  If you are a software developer and want to change the behavior of the IMU, you can modify the source code for the IMU by following these instructions.

NOTE: To interface the Arduino IDE to the nav6 from a computer without a RS-232 serial port, you can use an inexpensive [USB-to-RS-232 converter cable](http://www.kauailabs.com/store/index.php?route=product/product&product_id=53).

# Process #

1) Download the free [Arduino IDE](http://arduino.cc/en/Main/Software) to your development computer.

2) Install subversion on your development computer and [check out the IMU source code](https://code.google.com/p/nav6/source/checkout).

3) Modify the source code in the Arduino IDE to include your desired new functionality.

4) Connect the nav6 IMU to your development computer via the Serial Port.

5) In the Arduino IDE, select the Serial Port corresponding to the nav6 IMU via the Tools->Serial Port menu item.

6) In the Arduino IDE, set the Board type to "Arduino UNO".

7) Compile and upload your latest code to the nav6 via the File->Upload menu item.

**_Unlike the Arduino UNO board, you'll need to press the reset button on the nav6 circuit board just before starting the upload to the nav6. This approach was taken to ensure the nav6 is not reset accidentally during normal use._**

_**As soon as the upload starts, press the "RESET" button on the Nav6.  Pressing the RESET button will cause the bootloader software to execute.  If the upload starts within 2 seconds of pressing the RESET button, the upload should succeed.   You'll see the RX and TX LEDs blink as the sketch is uploaded.  If the RESET button is pushed too early, or too late, simply attempt the upload and press the RESET button again.**_

**NOTES:**

  * At any time you can re-load the latest [latest nav6 IMU firmware](http://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Farduino/nav6) using this same process.

  * Alternatively, if you suspect the nav6 may be malfunctioning, you may load and run the ["Factory Test" firmware](http://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Farduino/nav6FactoryTest).