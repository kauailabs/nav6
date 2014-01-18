/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import com.kauailabs.nav6.frc.IMU; 
//import com.kauailabs.nav6.frc.IMUAdvanced; Comment this in to use "Advanced" features
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.visa.VisaException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    
    SerialPort serial_port;
    IMU imu;  // Alternatively, use IMUAdvanced for advanced features
    
    public RobotTemplate() {
        try {
            serial_port = new SerialPort(57600);
            
            // You can add a second parameter to modify the 
            // update rate (in hz) from 4 to 100.  The default is 100.
            // To help reduce CPU load, set it to 50.
            
            // You can also use the IMUAdvanced class for advanced
            // features.
            
            imu = new IMU(serial_port,(byte)50);
            //imu = new IMUAdvanced(serial_port,(byte)50);
        } catch (VisaException ex) {
            ex.printStackTrace();
        }
        LiveWindow.addSensor("IMU", "Gyro", imu);
    }
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {

        while (isOperatorControl() && isEnabled()) {
            SmartDashboard.putBoolean("IMU_Connected", imu.isConnected());
            SmartDashboard.putNumber("IMU_Yaw", imu.getYaw());
            SmartDashboard.putNumber("IMU_Pitch", imu.getPitch());
            SmartDashboard.putNumber("IMU_Roll", imu.getRoll());
            SmartDashboard.putNumber("IMU_CompassHeading", imu.getCompassHeading());
            SmartDashboard.putNumber("IMU_Update_Count", imu.getUpdateCount());
            SmartDashboard.putNumber("IMU_Byte_Count", imu.getByteCount());
            // If you are using the IMUAdvanced class, you can also access the following
            // additional functions, at the expense of some extra processing
            // that occurs on the CRio processor
//            SmartDashboard.putNumber("IMU_Accel_X", imu.getWorldLinearAccelX());
//            SmartDashboard.putNumber("IMU_Accel_Y", imu.getWorldLinearAccelY());
//            SmartDashboard.putBoolean("IMU_IsMoving", imu.isMoving());
//            SmartDashboard.putNumber("IMU_Temp_C", imu.getTempC());
            Timer.delay(0.01);
        }
     }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
}
