/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs. All Rights Reserved.							  */
/* Based upon the Open Source WPI Library released by FIRST robotics.         */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/


#include "NetworkCommunication/UsageReporting.h"
#include "Timer.h"
#include "WPIErrors.h"
#include "LiveWindow/LiveWindow.h"
#include <time.h>
#include "IMU.h"
#include "IMUProtocol.h"
#include "Synchronized.h"

static SEM_ID cIMUStateSemaphore = semBCreate (SEM_Q_PRIORITY, SEM_FULL);   
static int update_count = 0;
static int byte_count = 0;

static bool stop = false;

/*** Internal task.
 * 
 * Task which retrieves yaw/pitch/roll updates from the IMU, via the
 * SerialPort.
 **/ 

static char protocol_buffer[256];

static void imuTask(IMU *imu) 
{
	stop = false;
	SerialPort *pport = imu->GetSerialPort();
	pport->SetReadBufferSize(512);
	pport->SetTimeout(1.0);
	pport->EnableTermination('\n');
	pport->Flush();
	pport->Reset();

	float yaw = 0.0;
	float pitch = 0.0;
	float roll = 0.0;	
	
	while (!stop)
	{ 
//		INT32 bytes_received = pport->GetBytesReceived();
//		if ( bytes_received > 0 )
		{
			UINT32 bytes_read = pport->Read( protocol_buffer, sizeof(protocol_buffer) );
			if ( bytes_read > 0 )
			{
				byte_count += bytes_read;
				UINT32 i = 0;
				// Scan the buffer looking for valid packets
				while ( i < bytes_read )
				{
					int bytes_remaining = bytes_read - i;
					int packet_length = IMUProtocol::decodeYPRUpdate( &protocol_buffer[i], bytes_remaining, yaw, pitch, roll ); 
					if ( packet_length > 0 )
					{
						update_count++;
						imu->SetYawPitchRoll(yaw,pitch,roll);
						i += packet_length;
					}
					else // current index is not the start of a valid packet; increment
					{
						i++;
					}
				}
			}
		}
	}
}

IMU::IMU( SerialPort *pport ) :
	m_task ("IMU", (FUNCPTR)imuTask,Task::kDefaultPriority+1)  
{
	yaw = 0.0;
	pitch = 0.0;
	roll = 0.0;
	pserial_port = pport;
	pserial_port->Reset();
	InitIMU();
	m_task.Start((UINT32)this);
}

/**
 * Initialize the IMU.
 */
void IMU::InitIMU()
{
	// The IMU serial port configuration is 8 data bits, no parity, one stop bit. 
	// No flow control is used.
	// Conveniently, these are the defaults used by the WPILib's SerialPort class.
	//
	// In addition, the WPILib's SerialPort class also defaults to:
	//
	// Timeout period of 5 seconds
	// Termination ('\n' character)
	// Transmit immediately
	InitializeYawHistory();
	yaw_offset = 0;
}

/**
 * Delete the IMU.
 */
IMU::~IMU()
{
	m_task.Stop();
}

void IMU::Restart()
{
	stop = true;
	pserial_port->Reset();
	m_task.Stop();
	
	pserial_port->Reset();
	InitializeYawHistory();
	update_count = 0;
	byte_count = 0;
	m_task.Restart();
}

bool IMU::IsConnected()
{
	double time_since_last_update = Timer::GetPPCTimestamp() - this->last_update_time;
	return time_since_last_update <= 1.0;
}

double IMU::GetByteCount()
{
	return byte_count;
}
double IMU::GetUpdateCount()
{
	return update_count;
}


void IMU::ZeroYaw()
{
	yaw_offset = GetAverageFromYawHistory();
}


/**
 * Return the yaw angle in degrees.
 * 
 * This angle increases as the robot spins to the right.
 * 
 * This angle ranges from -180 to 180 degrees.
 */
float IMU::GetYaw( void )
{
	Synchronized sync(cIMUStateSemaphore);
	double yaw = this->yaw;
	yaw -= yaw_offset;
	if ( yaw < -180 ) yaw += 360;
	if ( yaw > 180 ) yaw -= 360;
	return yaw;
}

float IMU::GetPitch( void )
{
	Synchronized sync(cIMUStateSemaphore);
	return this->pitch;
}

float IMU::GetRoll( void )
{
	Synchronized sync(cIMUStateSemaphore);
	return this->roll;
}

/**
 * Get the angle in degrees for the PIDSource base object.
 * 
 * @return The angle in degrees.
 */
double IMU::PIDGet()
{
	return GetYaw();
}

void IMU::UpdateTable() {
	if (m_table != NULL) {
		m_table->PutNumber("Value", GetYaw());
	}
}

void IMU::StartLiveWindowMode() {
	
}

void IMU::StopLiveWindowMode() {
	
}

std::string IMU::GetSmartDashboardType() {
	return "Gyro";
}

void IMU::InitTable(ITable *subTable) {
	m_table = subTable;
	UpdateTable();
}

ITable * IMU::GetTable() {
	return m_table;
}

void IMU::SetYawPitchRoll(float yaw, float pitch, float roll)
{
	{
		Synchronized sync(cIMUStateSemaphore);
		
		this->yaw = yaw;
		this->pitch = pitch;
		this->roll = roll;
	}
	UpdateYawHistory(this->yaw);
}

void IMU::InitializeYawHistory()
{
	for ( int i = 0; i < YAW_HISTORY_LENGTH; i++ )
	{
		yaw_history[i] = 0;
	}
	next_yaw_history_index = 0;
	last_update_time = 0.0;
}

void IMU::UpdateYawHistory(float curr_yaw )
{
	if ( next_yaw_history_index >= YAW_HISTORY_LENGTH )
	{
		next_yaw_history_index = 0;
	}
	yaw_history[next_yaw_history_index] = curr_yaw;
	last_update_time = Timer::GetPPCTimestamp();
	next_yaw_history_index++;
}

double IMU::GetAverageFromYawHistory()
{
	double yaw_history_sum = 0.0;
	for ( int i = 0; i < YAW_HISTORY_LENGTH; i++ )
	{
		yaw_history_sum += yaw_history[i];
	}	
	double yaw_history_avg = yaw_history_sum / YAW_HISTORY_LENGTH;
	return yaw_history_avg;
}
