/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs. All Rights Reserved.							  */
/*                                                                            */
/* Created in support of Team 2465 (Kauaibots).  Go Thunderchicken!           */
/*                                                                            */
/* Based upon the Open Source WPI Library released by FIRST robotics.         */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef IMU_H_
#define IMU_H_

#include "SensorBase.h"
#include "PIDSource.h"
#include "LiveWindow/LiveWindowSendable.h"
#include "SerialPort.h"
#include "Task.h"
/**
 * Use the IMU to retrieve a Yaw/Pitch/Roll measurement.
 * 
 * This utilizes the Kauai Labs Nav6 IMU.
 * 
 * This IMU interfaces to the CRio processor via a Serial port.
 */

#define YAW_HISTORY_LENGTH 10

class IMU : public SensorBase, public PIDSource, public LiveWindowSendable
{
protected:
	SerialPort *pserial_port;
	IMU(SerialPort *pport, bool internal, uint8_t update_rate_hz);
	void InitIMU();
	
public:

	IMU( SerialPort *pport, uint8_t update_rate_hz = 100 );
	virtual ~IMU();
	virtual float GetPitch();	// Pitch, in units of degrees (-180 to 180)
	virtual float GetRoll();	// Roll, in units of degrees (-180 to 180)
	virtual float GetYaw();		// Yaw, in units of degrees (-180 to 180)
	virtual float GetCompassHeading(); // CompassHeading, in units of degrees (0 to 360)
	
	bool IsConnected();
	void ZeroYaw();
	
	// PIDSource interface, returns yaw component in units of degrees (-180 to 180)
	double PIDGet();
	
	void UpdateTable();
	void StartLiveWindowMode();
	void StopLiveWindowMode();
	std::string GetSmartDashboardType();
	void InitTable(ITable *subTable);
	ITable * GetTable();

	SerialPort *GetSerialPort() { return pserial_port; }
	void SetYawPitchRoll(float yaw, float pitch, float roll, float compass_heading);
	double GetYawOffset() { return yaw_offset; }
	double GetByteCount();
	double GetUpdateCount();
	void Restart();

	uint8_t update_rate_hz;	
	
private:
	void InitializeYawHistory();
	double GetAverageFromYawHistory();

protected:

	void UpdateYawHistory(float curr_yaw );
	
	Task *	m_task;
	float 	yaw;
	float 	pitch; 
	float 	roll;
	float   compass_heading;
	float 	yaw_history[YAW_HISTORY_LENGTH];
	int 	next_yaw_history_index;
	double 	last_update_time;
	double 	yaw_offset;
    
	ITable *m_table;
};
#endif
