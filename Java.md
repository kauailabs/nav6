## RoboRIO ##

A Java software library and example code for integrating the nav6 into a FRC Robot is available.

[Source code](https://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Froborio%2Fjava) and [Online help](http://www.kauailabs.com/onlinedocs/nav6/java) for this Java library are available online.

**You can either choose to [checkout](https://code.google.com/p/nav6/source/checkout) the source code with subversion, or you can choose to download the [latest build](https://nav6.googlecode.com/svn/trunk/nav6.zip) of the libraries.**

## CRio ##
A Java software library and example code for integrating the nav6 into a FRC Robot is available.

[Source code](https://code.google.com/p/nav6/source/browse/#svn%2Ftrunk%2Fcrio%2Fjava) and [Online help](http://www.kauailabs.com/onlinedocs/nav6/java) for this Java library are available online.

**You can either choose to [checkout](https://code.google.com/p/nav6/source/checkout) the source code with subversion, or you can choose to download the [latest build](https://nav6.googlecode.com/svn/trunk/nav6.zip) of the libraries.**

### Installing the nav6 Library and getting started ###

1) First, install the latest nav6.jar file:

> https://nav6.googlecode.com/svn/trunk/nav6.zip

> In this zip file, extract the nav6.jar file, which is in the zip at /crio/java/build/nav6.jar.

> Place the nav6.jar file in your {user directory}/sunspotfrcsdk/lib directory (the same directory where the wpilibj.jar file is already present).

> The sunspotfrcsdk/lib directory is typically at C:\Users\{YourName}\sunspotfrcsdk\lib

2) Add references to the nav6 java library classpath to your robot project.

> From within the Netbeans IDE, select your project in the project tree, then:

> Project -> Properties

> Java Sources Classpath

> Add JAR/Folder

> {user directory}\sunspotfrcsdk\lib\nav6.jar

3) Initialize the nav6, and output information to the dashboard, as shown in the nav6SimpleRobotExample.  This verifies correct basic operation.

4) Finally, integrate the nav6 into your robot software by using the values from the nav6.  There are some examples discussed in the "Application Examples" section.