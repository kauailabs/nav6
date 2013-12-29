/* ============================================
Nav6 source code is placed under the MIT license
Copyright (c) 2013 Kauai Labs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

package com.kauailabs.nav6;

public class IMUProtocol {

    final static byte PACKET_START_CHAR = '!';
    final static int PROTOCOL_FLOAT_LENGTH = 7;
    final static int CHECKSUM_LENGTH = 2;
    final static int TERMINATOR_LENGTH = 2;

    // Yaw/Pitch/Roll (YPR) Update Packet - e.g., !y[yaw][pitch][roll][checksum][cr][lf]

    final static byte MSGID_YPR_UPDATE = 'y';
    final static int YPR_UPDATE_MESSAGE_LENGTH = 34; 	
    final static int YPR_UPDATE_YAW_VALUE_INDEX = 2;
    final static int YPR_UPDATE_PITCH_VALUE_INDEX = 9;
    final static int YPR_UPDATE_ROLL_VALUE_INDEX = 16;
    final static int YPR_UPDATE_COMPASS_VALUE_INDEX = 23;
    final static int YPR_UPDATE_CHECKSUM_INDEX = 30;
    final static int YPR_UPDATE_TERMINATOR_INDEX = 32;

    // Raw Data Update Packet - e.g., !r[q1][q2][q3][q4][accelx][accely][accelz][magx][magy][magz][checksum][cr][lf]

    final static byte MSGID_RAW_UPDATE = 'r';
    final static int RAW_UPDATE_MESSAGE_LENGTH =    	53;      
    final static int RAW_UPDATE_QUAT1_VALUE_INDEX =    2;
    final static int RAW_UPDATE_QUAT2_VALUE_INDEX =    6;
    final static int RAW_UPDATE_QUAT3_VALUE_INDEX = 	10;
    final static int RAW_UPDATE_QUAT4_VALUE_INDEX = 	14;
    final static int RAW_UPDATE_ACCEL_X_VALUE_INDEX =  18;
    final static int RAW_UPDATE_ACCEL_Y_VALUE_INDEX =  22;
    final static int RAW_UPDATE_ACCEL_Z_VALUE_INDEX =  26;	
    final static int RAW_UPDATE_MAG_X_VALUE_INDEX = 	30;
    final static int RAW_UPDATE_MAG_Y_VALUE_INDEX =    34;
    final static int RAW_UPDATE_MAG_Z_VALUE_INDEX =    38;
    final static int RAW_UPDATE_TEMP_VALUE_INDEX = 	42;
    final static int RAW_UPDATE_CHECKSUM_INDEX =       49;
    final static int RAW_UPDATE_TERMINATOR_INDEX =     51;

    // EnableStream Command Packet - e.g., !S[stream type][checksum][cr][lf]

    final static byte MSGID_STREAM_CMD = 'S';
    final static int STREAM_CMD_MESSAGE_LENGTH = 7;
    final static int STREAM_CMD_STREAM_TYPE_YPR = MSGID_YPR_UPDATE;
    final static int STREAM_CMD_STREAM_TYPE_RAW = MSGID_RAW_UPDATE;
    final static int STREAM_CMD_STREAM_TYPE_INDEX = 2;
    final static int STREAM_CMD_CHECKSUM_INDEX = 3;
    final static int STREAM_CMD_TERMINATOR_INDEX = 5;

    // EnableStream Response Packet - e.g., !s[stream type][gyro full scale range][accel full scale range][update rate hz][yaw_offset_degrees][flags][checksum][cr][lf]
    final static byte MSG_ID_STREAM_RESPONSE = 's';
    final static int STREAM_RESPONSE_MESSAGE_LENGTH = 30;
    final static int STREAM_RESPONSE_STREAM_TYPE_INDEX = 2;
    final static int STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE = 3;
    final static int STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE = 7;
    final static int STREAM_RESPONSE_UPDATE_RATE_HZ = 11;
    final static int STREAM_RESPONSE_YAW_OFFSET_DEGREES = 15;
    final static int STREAM_RESPONSE_FLAGS = 22;
    final static int STREAM_RESPONSE_CHECKSUM_INDEX = 26;
    final static int STREAM_RESPONSE_TERMINATOR_INDEX = 28;

    final static int IMU_PROTOCOL_MAX_MESSAGE_LENGTH = RAW_UPDATE_MESSAGE_LENGTH;

    static public class YPRUpdate {
        public float yaw;
        public float pitch;
        public float roll;
        public float compass_heading;
    }
    
    static public class StreamResponse {
        public byte stream_type;
        public short gyro_fsr_dps;
        public short accel_fsr_g;
        public short update_rate_hz;
        public float yaw_offset_degrees;
        public short flags;
    }
    
    static public class RawUpdate {
        public short q1;
        public short q2;
        public short q3;
        public short q4;
        public short accel_x;
        public short accel_y;
        public short accel_z;
        public short mag_x;
        public short mag_y;
        public short mag_z;
        public float temp_c;
    }
    
    public static int encodeStreamCommand( byte[] protocol_buffer, byte stream_type)
    {
      // Header
      protocol_buffer[0] = PACKET_START_CHAR;
      protocol_buffer[1] = MSGID_STREAM_CMD;

      // Data
      protocol_buffer[STREAM_CMD_STREAM_TYPE_INDEX] = stream_type;

      // Footer
      encodeTermination( protocol_buffer, STREAM_CMD_MESSAGE_LENGTH, STREAM_CMD_MESSAGE_LENGTH - 4 );

      return STREAM_CMD_MESSAGE_LENGTH;
    }

    public static int decodeStreamResponse( byte[] buffer, int length, 
                                            StreamResponse r )
    {
      if ( length < STREAM_RESPONSE_MESSAGE_LENGTH ) return 0;
      if ( ( buffer[0] == PACKET_START_CHAR ) && ( buffer[1] == MSG_ID_STREAM_RESPONSE ) )
      {
        if ( !verifyChecksum( buffer, STREAM_RESPONSE_CHECKSUM_INDEX ) ) return 0;

        r.stream_type		= buffer[2];
        r.gyro_fsr_dps   	= decodeProtocolUint16( buffer, STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE );
        r.accel_fsr_g   	= decodeProtocolUint16( buffer, STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE );
        r.update_rate_hz 	= decodeProtocolUint16( buffer, STREAM_RESPONSE_UPDATE_RATE_HZ );
        r.yaw_offset_degrees 	= decodeProtocolFloat(  buffer, STREAM_RESPONSE_YAW_OFFSET_DEGREES );
        r.flags			= decodeProtocolUint16( buffer, STREAM_RESPONSE_FLAGS );
      }
      return STREAM_RESPONSE_MESSAGE_LENGTH;
    }

    public static int decodeStreamCommand( byte[] buffer, int length, byte stream_type )
    {
      if ( length < STREAM_CMD_MESSAGE_LENGTH ) return 0;
      if ( ( buffer[0] == '!' ) && ( buffer[1] == MSGID_STREAM_CMD ) )
      {
        if ( !verifyChecksum( buffer, STREAM_CMD_CHECKSUM_INDEX ) ) return 0;

        stream_type = buffer[STREAM_CMD_STREAM_TYPE_INDEX];
      }
      return STREAM_CMD_MESSAGE_LENGTH;
    }

    public static int decodeYPRUpdate( byte[] buffer, int length, YPRUpdate u )
    {
      if ( length < YPR_UPDATE_MESSAGE_LENGTH ) return 0;
      if ( ( buffer[0] == '!' ) && ( buffer[1] == 'y' ) )
      {
        if ( !verifyChecksum( buffer, YPR_UPDATE_CHECKSUM_INDEX ) ) return 0;

        u.yaw   = decodeProtocolFloat( buffer, YPR_UPDATE_YAW_VALUE_INDEX );
        u.pitch = decodeProtocolFloat( buffer, YPR_UPDATE_PITCH_VALUE_INDEX );
        u.roll  = decodeProtocolFloat( buffer, YPR_UPDATE_ROLL_VALUE_INDEX );
        u.compass_heading = decodeProtocolFloat( buffer, YPR_UPDATE_COMPASS_VALUE_INDEX );
      }
      return YPR_UPDATE_MESSAGE_LENGTH;
    }

    public static int decodeRawUpdate( byte[] buffer, int length, 
                                        RawUpdate u )
    {
      if ( length < RAW_UPDATE_MESSAGE_LENGTH ) return 0;
      if ( ( buffer[0] == PACKET_START_CHAR ) && ( buffer[1] == MSGID_RAW_UPDATE ) )
      {
        if ( !verifyChecksum( buffer, RAW_UPDATE_CHECKSUM_INDEX ) ) return 0;

        u.q1        = decodeProtocolUint16( buffer, RAW_UPDATE_QUAT1_VALUE_INDEX );
        u.q2        = decodeProtocolUint16( buffer, RAW_UPDATE_QUAT2_VALUE_INDEX );
        u.q3        = decodeProtocolUint16( buffer, RAW_UPDATE_QUAT3_VALUE_INDEX );
        u.q4        = decodeProtocolUint16( buffer, RAW_UPDATE_QUAT4_VALUE_INDEX );
        u.accel_x   = decodeProtocolUint16( buffer, RAW_UPDATE_ACCEL_X_VALUE_INDEX );
        u.accel_y   = decodeProtocolUint16( buffer, RAW_UPDATE_ACCEL_Y_VALUE_INDEX );
        u.accel_z   = decodeProtocolUint16( buffer, RAW_UPDATE_ACCEL_Z_VALUE_INDEX );
        u.mag_x     = decodeProtocolUint16( buffer, RAW_UPDATE_MAG_X_VALUE_INDEX );
        u.mag_y     = decodeProtocolUint16( buffer, RAW_UPDATE_MAG_Y_VALUE_INDEX );
        u.mag_z     = decodeProtocolUint16( buffer, RAW_UPDATE_MAG_Z_VALUE_INDEX );
        u.temp_c    = decodeProtocolFloat(  buffer, RAW_UPDATE_TEMP_VALUE_INDEX );
      }
      return RAW_UPDATE_MESSAGE_LENGTH;
    }

    public static void encodeTermination( byte[] buffer, int total_length, int content_length )
    {
      if ( ( total_length >= (CHECKSUM_LENGTH + TERMINATOR_LENGTH) ) && ( total_length >= content_length + (CHECKSUM_LENGTH + TERMINATOR_LENGTH) ) )
      {
        // Checksum 
        byte checksum = 0;
        for ( int i = 0; i < content_length; i++ )
        {
          checksum += buffer[i];
        }
        // convert checksum to two ascii bytes
        
        byteToHex(checksum,buffer,content_length);
        // Message Terminator
        buffer[content_length + CHECKSUM_LENGTH + 0] = '\r';
        buffer[content_length + CHECKSUM_LENGTH + 1] = '\n';
      }
    }

    final protected static byte[] hexArray = 
        new byte[] {    (byte)'0', (byte)'1', (byte)'2', (byte)'3',
                        (byte)'4', (byte)'5', (byte)'6', (byte)'7',
                        (byte)'8', (byte)'9', (byte)'A', (byte)'B',
                        (byte)'C', (byte)'D', (byte)'E', (byte)'F' };

    public static void byteToHex(byte thebyte, byte[] dest, int offset) {
        int v = thebyte & 0xFF;
        dest[offset + 0] = hexArray[v >> 4];
        dest[offset + 1] = hexArray[v & 0x0F];
    }    
    
    public static short decodeProtocolUint16( byte[] uint16_string, int offset )
    {
            short decoded_uint16 = 0;
            int shift_left = 12;
            for ( int i = offset + 0; i < offset + 4; i++ ) 
            {
                byte digit = (byte) (uint16_string[i] <= '9' ? uint16_string[i] - '0' : ((uint16_string[i] - 'A') + 10));
                decoded_uint16 += (((short)digit) << shift_left);
                shift_left -= 4;
            }
            return decoded_uint16;  
    }  

    public static boolean verifyChecksum( byte[] buffer, int content_length )
    {
        // Calculate Checksum
        byte checksum = 0;
        for ( int i = 0; i < content_length; i++ )
        {
          checksum += buffer[i];
        }

        // Decode Checksum
        byte decoded_checksum = decodeChecksum( buffer, content_length );

        return ( checksum == decoded_checksum );
    }

    public static byte decodeChecksum( byte[] checksum, int offset )
    {
        byte first_digit = (byte) (checksum[0 + offset] <= '9' ? checksum[0 + offset] - '0' : ((checksum[0 + offset] - 'A') + 10));
        byte second_digit = (byte) (checksum[1 + offset] <= '9' ? checksum[1 + offset] - '0' : ((checksum[1 + offset] - 'A') + 10));
        byte decoded_checksum = (byte) ((first_digit * 16) + second_digit);
        return decoded_checksum;  
    }  

    public static float decodeProtocolFloat( byte[] buffer, int offset )
    {
      String float_string = new String(buffer,offset,PROTOCOL_FLOAT_LENGTH);
      return Float.parseFloat(float_string);
    }
}
