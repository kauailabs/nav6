Automatically rotating a robot to an angle using the nav6 IMU and a PIDController is simple and can be very useful.  Example source code for rotating to a given angle is shown below:

```

void turnToAngle( double angle_degrees ) {

  IMU *imu;         // Initialized previously; PID Controller Input
  PIDRotate rotate; // This is the PID Controller Output

  // Create the PIDController object.   Note that the DRIVE_P/I/D
  // values will need to be determined by the user.

  PIDController turn(DRIVE_P, DRIVE_I, DRIVE_D, imu, &rotate);

  // Configure the PID Controller settings

  turn.setTolerance(2.0); // OnTarget -> within 2% of desired angle
  turn.setSetpoint(angle_degrees);
  turn.setContinuous();
  turn.enable();

  // Until the target angle is reached, command the drive system
  // to rotate by the amount specified by the PID Controller.
  // This example assumes an omnidirectional drive system, and
  // turns "in-place" (by specifying no motion in X/Y axes). 

  while(!turn.onTarget()) {
    double x = 0.0;
    double y = 0.0;
    double rot = rotate.getValue();
    drive( x, y, rot);
  }

  // Clean up 

  turn.disable();
  delete turn;
}

// This simple class is a PIDOutput, and thus it receives
// the angle the PIDController has calculated should be
// output to the drive system.

public class PIDRotate implements PIDOutput {

  private double value;

  public PIDRotate() {
    value = 0.00;
  }

  public void pidWrite(double output) {
    value = output;
  }

  public double getValue() {
    return value;
  }
}

```

