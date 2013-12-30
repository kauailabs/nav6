/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import com.kauailabs.nav6.frc.IMU;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.SerialPort;
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
    IMU imu;
    
    public RobotTemplate() {
        try {
            serial_port = new SerialPort(57600);
            imu = new IMU(serial_port);
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

        SmartDashboard.putBoolean("IMU_Connected", imu.isConnected());
        SmartDashboard.putNumber("IMU_Yaw", imu.getYaw());
        SmartDashboard.putNumber("IMU_Pitch", imu.getPitch());
        SmartDashboard.putNumber("IMU_Roll", imu.getRoll());
        SmartDashboard.putNumber("IMU_CompassHeading", imu.getCompassHeading());
	SmartDashboard.putNumber("IMU_Update_Count", imu.getUpdateCount());
	SmartDashboard.putNumber("IMU_Byte_Count", imu.getByteCount());
        
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
}
