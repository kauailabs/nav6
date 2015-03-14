The nav6UI user interface application is a great way to visualize the data provided by the nav6 IMU.  This application is developed in the Processing 2.0 language and is cross-platform.

To run the nav6UI, the nav6 IMU must be connected to a PC (Windows, Mac or Linux) via a serial cable or a USB/Serial converter cable.

_Note:  if you are running Windows, you will need to also install ensure that a USB driver is installed, in order to communicate over USB to the nav6 IMU.  This installation is most easily accomplished by installing the Arduino IDE, which also installs the correct USB driver on your computer.  For details, please see the [Arduino for Windows driver installation instructions](http://arduino.cc/en/guide/windows#toc4)._

![https://nav6.googlecode.com/svn/trunk/images/nav6UI.png](https://nav6.googlecode.com/svn/trunk/images/nav6UI.png)

To build the nav6UI, follow these steps:

  * Download and install the free [Processing development environment](https://processing.org/download/).

  * Checkout the nav6 source code.

  * Copy the contents of the nav6 source code's processing directory to <User Directory>\Processing directory.

  * Open the Processing IDE and then open the nav6UI sketch via the File->Sketchbook menu.

  * Compile/Run the nav6UI by selecting the Sketch->Run menu.

If your computer has more than one serial port, you must run the nav6UI from the command-line and specify the serial port to use as a command-line parmeter, following these steps:

  * Export the nav6UI application via the File->Export Application menu item.

  * Locate the output of the Export Application operation.  A batch or script file (depending upon your operating system) will have been created.  Edit the script file to add the following to the end of the java line, as follows:

> java -Djava.ext.dirs=lib -Djava.library.path=lib nav6UI **port=COM12**