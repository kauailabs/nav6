Connecting the nav6 IMU to the cRio processor on a FIRST FRC robot is a very simple process and can be accomplished in just a few minutes.

The following are needed to accomplish this integration:

  * nav6 IMU Circuit Board
  * 2-pin JST "PH" style connector and wire harness
  * DB9 Male-to-female straight-through serial cable
  * A protective enclosure
  * Screws or other mounting hardware

_Note that the DB9 Male-to-female cable is a "straight-through" cable, NOT a "crossover" cable._

<img src='https://nav6.googlecode.com/svn/trunk/images/FRCMounting.png' height='700px' width='400px'>

<h1>Electrical Connections</h1>

First, connect the nav6 Power wire harness to one of the unregulated 12-volt connections on the power distribution board.  The nav6 consumes approximately 200 milliamps of current and thus can be connected to any of the 12-Volt power distribution board connections.  The nav6 requires a  minimum voltage of 5.5 volts.<br>
<br>
Next, connect the male DB9 connector to the nav6 circuit board, and the female DB9 connector to the cRio.<br>
<br>
<h1>Mounting</h1>

The nav6 IMU should be mounted such that it is firmly attached to the robot chassis.  <i><b>The quality of this mounting will be directly reflected in the quality of the inertial measurements provided by the nav6.</b></i>  To ensure quality, carefully follow these guidelines:<br>
<br>
<ul><li>The nav6 should be tightly mounted; it should be a part of the chassis mass, and should move exactly as the chassis moves.  Avoid mounting the nav6 in an area of the chassis that might be flexible, as this could introduce vibration to the IMU that does not represent the chassis inertial properties.</li></ul>

<ul><li>The nav6 IMU should be mounted in the center of the chassis, which ensures the origin of the yaw/pitch/roll axes truly represent the chassis center.</li></ul>

<ul><li>Be sure to understand the <a href='Orientation.md'>orientation</a> of the nav6 IMU, relative to the chassis.</li></ul>

<ul><li>Housing the nav6 IMU in some form of <a href='Enclosure.md'>protective enclosure</a> is highly recommended, to protect it from damage.  This should both protect the circuit board from damage, and provide strain relief for the cables that connect to the nav6 IMU.</li></ul>

Once installation is complete, <a href='IMUClass.md'>integration into the robot control software</a> can begin.