/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs. All Rights Reserved.							  */
/*                                                                            */
/* Created in support of Team 2465 (Kauaibots).  Go Thunderchicken!           */
/*                                                                            */
/* Based upon the Open Source WPI Library released by FIRST robotics.         */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef IMUADVANCED_H_
#define IMUADVANCED_H_

#include "IMU.h"
/**
 * Use the IMU to retrieve a Yaw/Pitch/Roll measurement.
 * 
 * This utilizes the Kauai Labs Nav6 IMU.
 * 
 * This IMU interfaces to the CRio processor via a Serial port.
 * 
 * This is the "advanced" version of the IMU class, which uses the
 * nav6 "Raw" messages, adding these additional features:
 * 
 * - calculate world-frame linear acceleration (gravity-removed)
 * - monitor the temperature of the Invensense sensor
 * - detection of motion/no-motion
 * 
 * Note that these additional features come at the expense of some
 * additional processing.
 */

#define WORLD_LINEAR_ACCEL_HISTORY_LENGTH 10

class IMUAdvanced : public IMU
{
public:

	IMUAdvanced( SerialPort *pport );
	virtual ~IMUAdvanced();
	
	virtual float GetWorldLinearAccelX();
	virtual float GetWorldLinearAccelY();
	virtual float GetWorldLinearAccelZ();
	virtual bool  IsMoving();
	virtual bool  IsCalibrating();
	virtual float GetTempC();
		
	void SetRaw( uint16_t q1, uint16_t q2, uint16_t q3, uint16_t q4,
					uint16_t accel_x, uint16_t accel_y, uint16_t accel_z,
					uint16_t mag_x, uint16_t mag_y, uint16_t mag_z,
					float temp_c);
	void SetStreamResponse( char stream_type, 
							uint16_t gyro_fsr_dps, uint16_t accel_fsr_g, uint16_t update_rate_hz,
							float yaw_offset_degrees, 
							uint16_t q1_offset, uint16_t q2_offset, uint16_t q3_offset, uint16_t q4_offset,
							uint16_t flags );
	
private:
	void InitIMU();
	void InitWorldLinearAccelHistory();
	void UpdateWorldLinearAccelHistory( float x, float y, float z );
	float GetAverageFromWorldLinearAccelHistory();

	float   yaw_offset_degrees;
	float   world_linear_accel_x;
	float   world_linear_accel_y;
	float   world_linear_accel_z;
	float   temp_c;
	uint16_t accel_fsr_g;
	uint16_t gyro_fsr_dps;
	uint16_t update_rate_hz;
	uint16_t flags;
	float 	world_linear_accel_history[WORLD_LINEAR_ACCEL_HISTORY_LENGTH];
	int 	next_world_linear_accel_history_index;
	float	world_linear_acceleration_recent_avg;

};
#endif
