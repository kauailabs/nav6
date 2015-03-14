The example below demonstrates how to implement automatic balance of a robot.  In this example, if the nav6 IMU indicates the current pitch angle exceeds a threshold, an "automatic balancing mode" is entered until the nav6 IMU indicates the current pitch angle is below another threshold.

During "automatic balancing mode", the drive system's Y axis is driven in the reverse direction with a magnitude equal to the sign of the pitch angle.

```
double pitchAngle, rollAngle, yawAngle, compassHeading;                

// Thresholds for determining if currently off-balance/on-balance.

const double offBalancePitchGyroAngleThresholdDegrees = 10;
const double onBalancePitchGyroAngleThresholdDegrees  = 5;
                
pitchAngleDegrees = imu->GetPitch();
                
// Determine whether assisted balance mode is in effect;
if ( !m_bAutoBalanceMode && ( pitchAngle >= offBalancePitchGyroAngleThresholdDegrees ) ) {
  m_bAutoBalanceMode = true;
}
else if ( m_bAutoBalanceMode && ( pitchAngle <= (-onBalancePitchGyroAngleThresholdDegrees))) {
  m_bAutoBalanceMode = false;
}

if ( m_bAutoBalanceMode ) {

  // Control drive system automatically, 
  // driving in reverse direction of pitch angle,
  // with a magnitude based upon the angle
  
  float pitchAngleRadians = pitchAngleDegrees * (3.1415926 / 180.0);
  float driveRate = sin(pitchAngleRadians) * -1;

  float x = 0.0;
  float y = driveRate;
  float rot = 0.0;

  drive->DoDrive( x, y, rot );
}
else {
  // control drive system based on inputs from joystick
}

```