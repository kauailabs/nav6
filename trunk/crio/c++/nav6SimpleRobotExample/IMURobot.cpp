#include "WPILib.h"
#include "IMU.h"
#include "IMUAdvanced.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	NetworkTable *table;
	IMUAdvanced *imu;
	SerialPort *serial_port;

public:
	RobotDemo()
	{
	}

	void RobotInit() {
		
		table = NetworkTable::GetTable("datatable");
		serial_port = new SerialPort(57600);
        uint8_t update_rate_hz = 50;
        imu = new IMUAdvanced(serial_port,update_rate_hz);
        if ( imu ) {
        	LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", imu);
        }
	}
	
	/**
	 * 
	 * Wait for 10 seconds
	 */
	void Autonomous(void)
	{
		Wait(2.0); 				//    for 10 seconds
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		while (IsOperatorControl())
		{
			SmartDashboard::PutBoolean( "IMU_Connected", imu->IsConnected());
			SmartDashboard::PutNumber("IMU_Yaw", imu->GetYaw());
			SmartDashboard::PutNumber("IMU_Pitch", imu->GetPitch());
			SmartDashboard::PutNumber("IMU_Roll", imu->GetRoll());
			SmartDashboard::PutNumber("IMU_CompassHeading", imu->GetCompassHeading());
			SmartDashboard::PutNumber("IMU_Update_Count", imu->GetUpdateCount());
			SmartDashboard::PutNumber("IMU_Byte_Count", imu->GetByteCount());

			SmartDashboard::PutNumber("IMU_Accel_X", imu->GetWorldLinearAccelX());
			SmartDashboard::PutNumber("IMU_Accel_Y", imu->GetWorldLinearAccelY());
			SmartDashboard::PutBoolean("IMU_IsMoving", imu->IsMoving());
			SmartDashboard::PutNumber("IMU_Temp_C", imu->GetTempC());

			Wait(0.2);				// wait for a while
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}

};

START_ROBOT_CLASS(RobotDemo);

