/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs 2013. All Rights Reserved.                        */
/*                                                                            */
/* Created in support of Team 2465 (Kauaibots).  Go Thunderchicken!           */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. Any        */
/* modifications to this code must be accompanied by the nav6_License.txt file*/ 
/* in the root directory of the project.                                      */
/*----------------------------------------------------------------------------*/

package com.kauailabs.nav6.frc;

import com.kauailabs.nav6.IMUProtocol;
import com.kauailabs.nav6.frc.BufferingSerialPort;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.visa.VisaException;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author Scott
 */
public class IMU extends SensorBase implements PIDSource, LiveWindowSendable, Runnable {

    static final int YAW_HISTORY_LENGTH = 10;
    static final byte DEFAULT_UPDATE_RATE_HZ = 100;

    BufferingSerialPort serial_port;
    volatile float yaw;
    volatile float pitch;
    volatile float roll;
    volatile float compass_heading;
    float yaw_history[];
    int next_yaw_history_index;
    double last_update_time;
    double yaw_offset;
    ITable m_table;
    Thread m_thread;
    protected byte update_rate_hz;

    volatile int update_count = 0;
    volatile int byte_count = 0;

    boolean stop = false;
    char protocol_buffer[];

    public IMU(BufferingSerialPort serial_port, byte update_rate_hz) {
        this.update_rate_hz = update_rate_hz;
        this.serial_port = serial_port;
        yaw_history = new float[YAW_HISTORY_LENGTH];
        protocol_buffer = new char[256];
        yaw = (float) 0.0;
        pitch = (float) 0.0;
        roll = (float) 0.0;
        try {
            serial_port.reset();
        } catch (VisaException ex) {
            ex.printStackTrace();
        }
        initIMU();
        m_thread = new Thread(this);
        m_thread.start();        
    }
    
    public IMU(BufferingSerialPort serial_port) {
        this(serial_port,DEFAULT_UPDATE_RATE_HZ);
    }

    protected void initIMU() {
        // The IMU serial port configuration is 8 data bits, no parity, one stop bit. 
        // No flow control is used.
        // Conveniently, these are the defaults used by the WPILib's SerialPort class.
        //
        // In addition, the WPILib's SerialPort class also defaults to:
        //
        // Timeout period of 5 seconds
        // Termination ('\n' character)
        // Transmit immediately
        initializeYawHistory();
        yaw_offset = 0;

        // set the nav6 into "YPR" update mode
	byte stream_command_buffer[] = new byte[256];
	int packet_length = IMUProtocol.encodeStreamCommand( stream_command_buffer, (byte)IMUProtocol.STREAM_CMD_STREAM_TYPE_YPR, update_rate_hz ); 
        try {
            serial_port.write( stream_command_buffer, packet_length );
        } catch (VisaException ex) {
        }
    }

    private void initializeYawHistory() {

        for (int i = 0; i < YAW_HISTORY_LENGTH; i++) {
            yaw_history[i] = 0;
        }
        next_yaw_history_index = 0;
        last_update_time = 0.0;

    }

    private void setYawPitchRoll(float yaw, float pitch, float roll, float compass_heading) {
        //synchronized (this) { // synchronized block
            this.yaw = yaw;
            this.pitch = pitch;
            this.roll = roll;
            this.compass_heading = compass_heading;
        //}
        updateYawHistory(this.yaw);
    }

    private void updateYawHistory(float curr_yaw) {
        if (next_yaw_history_index >= YAW_HISTORY_LENGTH) {
            next_yaw_history_index = 0;
        }
        yaw_history[next_yaw_history_index] = curr_yaw;
        last_update_time = Timer.getFPGATimestamp();
        next_yaw_history_index++;
    }

    private double getAverageFromYawHistory() {
        double yaw_history_sum = 0.0;
        for (int i = 0; i < YAW_HISTORY_LENGTH; i++) {
            yaw_history_sum += yaw_history[i];
        }
        double yaw_history_avg = yaw_history_sum / YAW_HISTORY_LENGTH;
        return yaw_history_avg;
    }

    // Pitch, in units of degrees (-180 to 180)
    public float getPitch() {
        //synchronized (this) { // synchronized block
            return pitch;
        //}
    }

    public float getRoll() {
        //synchronized (this) { // synchronized block
            return roll;
        //}
    }

    public float getYaw() {
        float calculated_yaw;
        //synchronized (this) { // synchronized block
            calculated_yaw = this.yaw;
        //}
        calculated_yaw -= yaw_offset;
        if (calculated_yaw < -180) {
            calculated_yaw += 360;
        }
        if (calculated_yaw > 180) {
            calculated_yaw -= 360;
        }
        return calculated_yaw;
    }

    public float getCompassHeading() {

        //synchronized (this) { // synchronized block
            return compass_heading;
        //}
    }

    public void zeroYaw() {
        yaw_offset = getAverageFromYawHistory();
    }

    public boolean isConnected() {
        double time_since_last_update = Timer.getFPGATimestamp() - this.last_update_time;
        return time_since_last_update <= 1.0;
    }

    public double getByteCount() {
        return byte_count;
    }

    public double getUpdateCount() {
        return update_count;
    }

    public double pidGet() {
        return getYaw();
    }

    public void updateTable() {
        if (m_table != null) {
            m_table.putNumber("Value", getYaw());
        }
    }

    public void startLiveWindowMode() {
    }

    public void stopLiveWindowMode() {
    }

    public void initTable(ITable itable) {
        m_table = itable;
        updateTable();
    }

    public ITable getTable() {
        return m_table;
    }

    public String getSmartDashboardType() {
        return "Gyro";
    }

    public void run() {
        int last_err_code = 0;
        stop = false;
        try {
            serial_port.setReadBufferSize(512);
            serial_port.setTimeout(5.0);
            serial_port.enableTermination('\n');
            serial_port.flush();
            serial_port.reset();
        } catch (VisaException ex) {
            ex.printStackTrace();
        }

        IMUProtocol.YPRUpdate update = new IMUProtocol.YPRUpdate();
        update.yaw = (float) 0.0;
        update.pitch = (float) 0.0;
        update.roll = (float) 0.0;
        update.compass_heading = (float) 0.0;

        byte[] remaining_data = new byte[256];

        // Give the nav6 circuit a few seconds to initialize, then send the stream configuration command.
        Timer.delay(2.0);
	int cmd_packet_length = IMUProtocol.encodeStreamCommand( remaining_data, (byte)IMUProtocol.STREAM_CMD_STREAM_TYPE_YPR, update_rate_hz ); 
        try {
            serial_port.write( remaining_data, cmd_packet_length );
            serial_port.flush();
            serial_port.reset();
        } catch (VisaException ex) {
        }
        
        while (!stop) {
            try {
                int packets_received = 0;
                byte[] received_data = serial_port.read(256);
                int bytes_read = received_data.length;
                if (bytes_read > 0) {
                    byte_count += bytes_read;
                    int i = 0;
                    // Scan the buffer looking for valid packets
                    while (i < bytes_read) {
                        int bytes_remaining = bytes_read - i;
                        System.arraycopy(received_data, i, remaining_data, 0, bytes_remaining);
                        int packet_length = IMUProtocol.decodeYPRUpdate(remaining_data, bytes_remaining, update);
                        if (packet_length > 0) {
                            packets_received++;
                            update_count++;
                            setYawPitchRoll(update.yaw, update.pitch, update.roll, update.compass_heading);
                            i += packet_length;
                        } else // current index is not the start of a valid packet; increment
                        {
                            i++;
                        }
                    }
                    if ( ( packets_received == 0 ) && ( bytes_read == 256 ) ) {
                        // No packets received and 256 bytes received; this
                        // condition occurs in the Java SerialPort.  In this case,
                        // reset the serial port.
                        serial_port.reset();
                    }
                }
            } catch (VisaException ex) {
                // ex.hasCode() value of 17 == Timeout
                int error_code = ex.hashCode();
                int x = error_code;
            }
        }
    }

}
