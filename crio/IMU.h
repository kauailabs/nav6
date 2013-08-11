/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
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
 * Use the angle sensor to read an absolute angle measurement.
 * 
 * This utilizes the Avagotech AEAT-6012-A06 12-bit Absolute Magnetic Encoder.
 * 
 * This sensor requires two DigitalOutput pin (Chip Select, Clock)
 * 
 * This sensor also requires one DigitalInput pin (Data)
 */

#define YAW_HISTORY_LENGTH 10

class IMU : public SensorBase, public PIDSource, public LiveWindowSendable
{
	SerialPort *pserial_port;
	
public:

	IMU( SerialPort *pport );
	virtual ~IMU();
	virtual float GetPitch();	// Pitch, in units of degrees (-180 to 180)
	virtual float GetRoll();	// Roll, in units of degrees (-180 to 180)
	virtual float GetYaw();		// Yaw, in units of degrees (-180 to 180)
	
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
	void SetYawPitchRoll(float yaw, float pitch, float roll);
	double GetYawOffset() { return yaw_offset; }
	double GetByteCount();
	double GetUpdateCount();
	void Restart();
	
private:
	void InitIMU();
	void InitializeYawHistory();
	void UpdateYawHistory(float curr_yaw );
	double GetAverageFromYawHistory();

    Task 	m_task;
	float 	yaw;
	float 	pitch; 
	float 	roll;
	float 	yaw_history[YAW_HISTORY_LENGTH];
	int 	next_yaw_history_index;
	double 	last_update_time;
	double 	yaw_offset;
    
	ITable *m_table;
};
#endif