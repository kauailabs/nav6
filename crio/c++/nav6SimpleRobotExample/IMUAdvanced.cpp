/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs. All Rights Reserved.							  */
/*                                                                            */
/* Created in support of Team 2465 (Kauaibots).  Go Thunderchicken!           */
/*                                                                            */
/* Based upon the Open Source WPI Library released by FIRST robotics.         */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkCommunication/UsageReporting.h"
#include "Timer.h"
#include "WPIErrors.h"
#include "LiveWindow/LiveWindow.h"
#include <time.h>
#include "IMUAdvanced.h"
#include "IMUProtocol.h"
#include "Synchronized.h"
#include <math.h>

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

static void imuAdvancedTask(IMUAdvanced *imu) 
{
	stop = false;
	SerialPort *pport = imu->GetSerialPort();
	pport->SetReadBufferSize(512);
	pport->SetTimeout(1.0);
	pport->EnableTermination('\n');
	pport->Flush();
	pport->Reset();

	int16_t q1, q2, q3, q4;
	int16_t accel_x, accel_y, accel_z;
	int16_t mag_x, mag_y, mag_z;
	float temp_c;
	char stream_type;
	uint16_t gyro_fsr_dps, accel_fsr_g, update_rate_hz;
	uint16_t q1_offset, q2_offset, q3_offset, q4_offset;
	float yaw_offset_degrees;
	uint16_t flags;
	
    // Give the nav6 circuit a few seconds to initialize, then send the stream configuration command.
    Wait(2.0);
    int cmd_packet_length = IMUProtocol::encodeStreamCommand( protocol_buffer, STREAM_CMD_STREAM_TYPE_QUATERNION, imu->update_rate_hz ); 
   
    pport->Write( protocol_buffer, cmd_packet_length );
    pport->Flush();
    pport->Reset();	
	
	while (!stop)
	{ 
//		INT32 bytes_received = pport->GetBytesReceived();
//		if ( bytes_received > 0 )
		{
			UINT32 bytes_read = pport->Read( protocol_buffer, sizeof(protocol_buffer) );
			if ( bytes_read > 0 )
			{
				int packets_received = 0;
				byte_count += bytes_read;
				UINT32 i = 0;
				// Scan the buffer looking for valid packets
				while ( i < bytes_read )
				{
					byte_count += bytes_read;
					int bytes_remaining = bytes_read - i;
					int packet_length = IMUProtocol::decodeQuaternionUpdate( &protocol_buffer[i], bytes_remaining, 
							q1,q2,q3,q4,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,temp_c ); 
					if ( packet_length > 0 )
					{
						packets_received++;
						update_count++;
						imu->SetRaw(q1,q2,q3,q4,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,temp_c);
						i += packet_length;
					}
					else 
					{
						packet_length = IMUProtocol::decodeStreamResponse( &protocol_buffer[i], bytes_remaining, stream_type,
								  gyro_fsr_dps, accel_fsr_g, update_rate_hz,
								  yaw_offset_degrees, 
								  q1_offset, q2_offset, q3_offset, q4_offset,
								  flags );
						if ( packet_length > 0 ) 
						{
							packets_received++;
							imu->SetStreamResponse( stream_type, 
									  gyro_fsr_dps, accel_fsr_g, update_rate_hz,
									  yaw_offset_degrees, 
									  q1_offset, q2_offset, q3_offset, q4_offset,
									  flags );
							i += packet_length;
						}
						else // current index is not the start of a valid packet we're interested in; increment
						{
							i++;
						}
					}
				}
	            if ( ( packets_received == 0 ) && ( bytes_read == 256 ) ) {
	                // No packets received and 256 bytes received; this
	                // condition occurs in the SerialPort.  In this case,
	                // reset the serial port.
	            	pport->Reset();
	            }				
			}
			else {
				double start_wait_timer = Timer::GetFPGATimestamp();
				// Timeout
				int bytes_received = pport->GetBytesReceived();
				while ( !stop && ( bytes_received == 0 ) ) {
					Wait(1.0/imu->update_rate_hz);
					bytes_received = pport->GetBytesReceived();
				}
                if ( !stop && (bytes_received > 0 ) ) {
                    if ( (Timer::GetFPGATimestamp() - start_wait_timer ) > 1.0 ) {
                    	Wait(2.0);
                        int cmd_packet_length = IMUProtocol::encodeStreamCommand( protocol_buffer, STREAM_CMD_STREAM_TYPE_QUATERNION, imu->update_rate_hz ); 
                       
                        pport->Write( protocol_buffer, cmd_packet_length );
                        pport->Flush();
                        pport->Reset();
                    }
                }
			}
            
		}
	}
}

IMUAdvanced::IMUAdvanced( SerialPort *pport, uint8_t update_rate_hz ) :
	IMU(pport,true, update_rate_hz)
{
	yaw_offset_degrees = 0;
	accel_fsr_g = 2;
	gyro_fsr_dps = 2000;
	m_task = new Task("IMUAdvanced", (FUNCPTR)imuAdvancedTask,Task::kDefaultPriority+1); 
	m_task->Start((UINT32)this);
}

IMUAdvanced::~IMUAdvanced() {
	
}

void IMUAdvanced::InitWorldLinearAccelHistory()
{
	for ( int i = 0; i > WORLD_LINEAR_ACCEL_HISTORY_LENGTH; i++ )
	{
		world_linear_accel_history[i] = 0.0;
	}
	next_world_linear_accel_history_index = 0;
	world_linear_acceleration_recent_avg = 0.0;	
}

void IMUAdvanced::UpdateWorldLinearAccelHistory( float x, float y, float z )
{
	if ( next_world_linear_accel_history_index >= WORLD_LINEAR_ACCEL_HISTORY_LENGTH )
	{
		next_world_linear_accel_history_index = 0;
	}
	world_linear_accel_history[next_world_linear_accel_history_index] = fabs(x) + fabs(y);
	next_world_linear_accel_history_index++;
}

float IMUAdvanced::GetAverageFromWorldLinearAccelHistory()
{
	float world_linear_accel_history_avg = 0.0;
	for ( int i = 0; i < WORLD_LINEAR_ACCEL_HISTORY_LENGTH; i++ )
	{
		world_linear_accel_history_avg += world_linear_accel_history[i];
	}
	world_linear_accel_history_avg /= WORLD_LINEAR_ACCEL_HISTORY_LENGTH;
	return world_linear_accel_history_avg;
}


/**
 * Initialize the IMU.
 */
void IMUAdvanced::InitIMU()
{
	IMU::InitIMU();
	InitWorldLinearAccelHistory();
	
	// set the nav6 into "Quaternion" update mode
	
	int packet_length = IMUProtocol::encodeStreamCommand( protocol_buffer, STREAM_CMD_STREAM_TYPE_QUATERNION, update_rate_hz ); 
	pserial_port->Write( protocol_buffer, packet_length );
}

float IMUAdvanced::GetWorldLinearAccelX()
{
	Synchronized sync(cIMUStateSemaphore);
	return this->world_linear_accel_x;
}

float IMUAdvanced::GetWorldLinearAccelY()
{
	Synchronized sync(cIMUStateSemaphore);
	return this->world_linear_accel_y;	
}

float IMUAdvanced::GetWorldLinearAccelZ()
{
	Synchronized sync(cIMUStateSemaphore);
	return this->world_linear_accel_z;	
}

bool  IMUAdvanced::IsMoving()
{
	Synchronized sync(cIMUStateSemaphore);
	return (GetAverageFromWorldLinearAccelHistory() >= 0.01);
}

bool IMUAdvanced::IsCalibrating()
{
	Synchronized sync(cIMUStateSemaphore);
	uint16_t calibration_state = this->flags & NAV6_FLAG_MASK_CALIBRATION_STATE;
	return (calibration_state != NAV6_CALIBRATION_STATE_COMPLETE);
}

float IMUAdvanced::GetTempC()
{
	Synchronized sync(cIMUStateSemaphore);
	return this->temp_c;
}

void IMUAdvanced::SetRaw( int16_t quat1, int16_t quat2, int16_t quat3, int16_t quat4,
					int16_t accel_x, int16_t accel_y, int16_t accel_z,
					int16_t mag_x, int16_t mag_y, int16_t mag_z,
					float temp_c)
{
	{
		Synchronized sync(cIMUStateSemaphore);

		float q[4];				// Quaternion from IMU
		float gravity[3];		// Gravity Vector
		//float euler[3];			// Classic euler angle representation of quaternion
		float ypr[3];			// Angles in "Tait-Bryan" (yaw/pitch/roll) format
		float yaw_degrees;
		float pitch_degrees;
		float roll_degrees;
		float linear_acceleration_x;
		float linear_acceleration_y;
		float linear_acceleration_z;
        float q2[4];
        float q_product[4];
		float world_linear_acceleration_x;
		float world_linear_acceleration_y;
		float world_linear_acceleration_z;
        
		// Convert 15-bit signed quaternion integral values to floats
		q[0] = ((float)quat1) / 16384.0f;
		q[1] = ((float)quat2) / 16384.0f;
		q[2] = ((float)quat3) / 16384.0f;
        q[3] = ((float)quat4) / 16384.0f;
        
        // Fixup any quaternion values out of range
        for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];

        // below calculations are necessary for calculation of yaw/pitch/roll, 
        // and tilt-compensated compass heading
        
        // calculate gravity vector
        gravity[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
        gravity[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
        gravity[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
        
        // calculate Euler angles
        // This code is here for reference, and is commented out for performance reasons
        //euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
        //euler[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);
        //euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);

        // calculate yaw/pitch/roll angles
        ypr[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
        ypr[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
        ypr[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));

        // Convert yaw/pitch/roll angles to degreess, and remove calibrated offset
        
        yaw_degrees = ypr[0] * (180.0/3.1415926); 
        pitch_degrees = ypr[1] * (180.0/3.1415926); 
        roll_degrees = ypr[2] * (180.0/3.1415926); 
         
        // Subtract offset, and handle potential 360 degree wrap-around
        yaw_degrees -= yaw_offset_degrees;
        if ( yaw_degrees < -180 ) yaw_degrees += 360;
        if ( yaw_degrees > 180 ) yaw_degrees -= 360;

        // calculate linear acceleration by 
        // removing the gravity component from raw acceleration values
        // Note that this code assumes the acceleration full scale range
        // is +/- 2 degrees
         
        linear_acceleration_x = (float)(((float)accel_x) / (32768.0 / (float)accel_fsr_g)) - gravity[0];
        linear_acceleration_y = (float)(((float)accel_y) / (32768.0 / (float)accel_fsr_g)) - gravity[1];
        linear_acceleration_z = (float)(((float)accel_z) / (32768.0 / (float)accel_fsr_g)) - gravity[2]; 
        
        // Calculate world-frame acceleration
        
        q2[0] = 0;
        q2[1] = linear_acceleration_x;
        q2[2] = linear_acceleration_y;
        q2[3] = linear_acceleration_z;
        
        // Rotate linear acceleration so that it's relative to the world reference frame
        
        // http://www.cprogramming.com/tutorial/3d/quaternions.html
        // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
        // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
        // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1
    
        // P_out = q * P_in * conj(q)
        // - P_out is the output vector
        // - q is the orientation quaternion
        // - P_in is the input vector (a*aReal)
        // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])

        // calculate quaternion product
        // Quaternion multiplication is defined by:
        //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
        //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
        //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
        //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
        
        q_product[0] = q[0]*q2[0] - q[1]*q2[1] - q[2]*q2[2] - q[3]*q2[3];  // new w
        q_product[1] = q[0]*q2[1] + q[1]*q2[0] + q[2]*q2[3] - q[3]*q2[2];  // new x
        q_product[2] = q[0]*q2[2] - q[1]*q2[3] + q[2]*q2[0] + q[3]*q2[1];  // new y 
        q_product[3] = q[0]*q2[3] + q[1]*q2[2] - q[2]*q2[1] + q[3]*q2[0];  // new z

        float q_conjugate[4];
        
        q_conjugate[0] = q[0];            
        q_conjugate[1] = -q[1];            
        q_conjugate[2] = -q[2];            
        q_conjugate[3] = -q[3];            

        float q_final[4];
        
        q_final[0] = q_product[0]*q_conjugate[0] - q_product[1]*q_conjugate[1] - q_product[2]*q_conjugate[2] - q_product[3]*q_conjugate[3];  // new w
        q_final[1] = q_product[0]*q_conjugate[1] + q_product[1]*q_conjugate[0] + q_product[2]*q_conjugate[3] - q_product[3]*q_conjugate[2];  // new x
        q_final[2] = q_product[0]*q_conjugate[2] - q_product[1]*q_conjugate[3] + q_product[2]*q_conjugate[0] + q_product[3]*q_conjugate[1];  // new y 
        q_final[3] = q_product[0]*q_conjugate[3] + q_product[1]*q_conjugate[2] - q_product[2]*q_conjugate[1] + q_product[3]*q_conjugate[0];  // new z

        world_linear_acceleration_x = q_final[1];
        world_linear_acceleration_y = q_final[2];
        world_linear_acceleration_z = q_final[3];
        
        // Calculate tilt-compensated compass heading
        
        float inverted_pitch = -ypr[1];
        float roll = ypr[2];
        
        float cos_roll = cos(roll);
        float sin_roll = sin(roll);
        float cos_pitch = cos(inverted_pitch);
        float sin_pitch = sin(inverted_pitch);
        
        float MAG_X = mag_x * cos_pitch + mag_z * sin_pitch;
        float MAG_Y = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch;
        float tilt_compensated_heading_radians = atan2(MAG_Y,MAG_X);
        float tilt_compensated_heading_degrees = tilt_compensated_heading_radians * (180.0 / 3.1415926);
        
        // Adjust compass for board orientation,
        // and modify range from -180-180 to
        // 0-360 degrees
      
        tilt_compensated_heading_degrees -= 90.0;
        if ( tilt_compensated_heading_degrees < 0 ) {
          tilt_compensated_heading_degrees += 360; 
        }

		this->yaw = yaw_degrees;
		this->pitch = pitch_degrees;
		this->roll = roll_degrees;
		this->compass_heading = tilt_compensated_heading_degrees;
        
		this->world_linear_accel_x = world_linear_acceleration_x;
		this->world_linear_accel_y = world_linear_acceleration_y;
		this->world_linear_accel_z = world_linear_acceleration_z;
		this->temp_c = temp_c;
		
		UpdateYawHistory(this->yaw);
		UpdateWorldLinearAccelHistory( world_linear_acceleration_x, world_linear_acceleration_y, world_linear_acceleration_z);
	}	
}

void IMUAdvanced::SetStreamResponse( char stream_type, 
								uint16_t gyro_fsr_dps, uint16_t accel_fsr_g, uint16_t update_rate_hz,
								float yaw_offset_degrees, 
								uint16_t q1_offset, uint16_t q2_offset, uint16_t q3_offset, uint16_t q4_offset,
								uint16_t flags )
{
	{
		Synchronized sync(cIMUStateSemaphore);
		this->yaw_offset_degrees = yaw_offset_degrees;
		this->flags = flags;
		this->accel_fsr_g = accel_fsr_g;
		this->gyro_fsr_dps = gyro_fsr_dps;
		this->update_rate_hz = update_rate_hz;
	}		
}

double IMUAdvanced::GetByteCount()
{
	return byte_count;
}
double IMUAdvanced::GetUpdateCount()
{
	return update_count;
}
