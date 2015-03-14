An easy-to-use, highly-maneuverable drive system is at the heart of a successful [FIRST Robotics Challenge (FRC)](http://www.usfirst.org/frc) robot.  Omnidirectional drive systems (e.g., omniwheel, mecanum, swerve) are typically employed to provide enhanced maneuverability, yet introduce a new challenge:  _increased driving complexity, related to the requirement that the driver “put their head in the robot”_.  The nav6 IMU can be used to add Field-Oriented Drive to omnidirectional drive robots - reducing driver training time, enabling driver/shooter delegation in turret-less shooting robotis, and allowing drivers to fully focus on the game rather than the drive system.

![http://www.kauailabs.com/store/image/data/OmniDirectionalDrive.png](http://www.kauailabs.com/store/image/data/OmniDirectionalDrive.png)

Omnidirectional drive systems provide drivers a highly-maneuverable robot that can move in the Y axis (forward-backward), X-axis (strafe), and Z axis (rotating about it's center axis).  Each degree of freedom is independent, meaning that the overall robot motion is comprised of a “mix” of motion in each of the X, Y and Z axes, control of which is easily provided with a 3-degree of freedom joystick.
This resulting maneuverability is quite useful during FRC competitions to avoid other robots, pick up and place game pieces, line up for shooting to a target, etc.

Yet the driver who remains in a fixed position is now presented a new challenge:  when the driving joystick is pushed forward, the robot does not necessarily move forward with respect to the field – rather it moves forward with respect to the robot.  This forces the driver to develop the skill of “placing their head in the robot” and performing the angular transformation mentally.  This skill can take quite awhile to develop meaning that rookie drivers face an uphill climb before they can be productive team contributors.  Additionally, the mental energy involved in field-to-robot rotational transformations reduces the driver's cognitive ability to focus other game-related tactical tasks, as evidenced by drivers who are so intently focused on driving that their response to their teammates is diminished.  Moreover, when the driver does not have a clear line of sight to the robot, the “head in the robot” becomes even more challenging.

Solving this challenge is conceptually straightforward.  First, the current angle (θ) of rotation between the head of the field, and the head of the robot must be measured; secondly, the joystick X/Y coordinates are transformed by θ, as shown in following C++ code:

```
        // Get Current Joystick values

	double rotate = pstick->GetTwist();
	double y = pstick->GetY();
	double x = pstick->GetX();
	
	y = -y;	// Invert joystick (by default, joystick "forward" is negative)

        float strafe = x;
        float fwd = y;
        float rcw = rotate;

        float pi = 3.1415926;
        
        // Field-oriented drive - Adjust input angle for gyro offset angle
        
        float curr_gyro_angle_degrees = 0;
        if ( imu->IsConnected() ) 
        {
            curr_gyro_angle_degrees = imu->GetYaw();

            // Update dashboard with latest IMU values

            SmartDashboard::PutNumber(  "IMU_Yaw",       imu_angle_degrees);
            SmartDashboard::PutNumber( "IMU Yaw Offset", imu->GetYawOffset());
                
        }

        float curr_gyro_angle_radians = curr_gyro_angle_degrees * pi/180;       
        
        float temp = fwd * cos( curr_gyro_angle_radians ) + strafe * sin( curr_gyro_angle_radians);

        strafe = -fwd * sin( curr_gyro_angle_radians ) + strafe * cos( curr_gyro_angle_radians );

        fwd = temp;

        // At this point, the strafe and fwd vectors have been
        // adjusted, and can be provided to the drive system.
```

_NOTE:  For details on field-oriented drive algorithms, please see this [excellent post on Chief Delphi by Ether](http://www.chiefdelphi.com/media/papers/2390) which provides a wealth of helpful, well written information on implementing field-oriented drive on various types of drive systems._