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

#ifndef _IMU_PROTOCOL_H_
#define _IMU_PROTOCOL_H_

#define PACKET_START_CHAR '!'
#define PROTOCOL_FLOAT_LENGTH 7
#define CHECKSUM_LENGTH 2
#define TERMINATOR_LENGTH 2

// Yaw/Pitch/Roll (YPR) Update Packet - e.g., !y[yaw][pitch][roll][checksum][cr][lf]

#define MSGID_YPR_UPDATE 'y'
#define YPR_UPDATE_MESSAGE_LENGTH 34 	
						//       where yaw, pitch, roll are floats
						//		 where checksum is 2 ascii-bytes of HEX checksum (all bytes before checksum)
#define YPR_UPDATE_YAW_VALUE_INDEX 2
#define YPR_UPDATE_PITCH_VALUE_INDEX 9
#define YPR_UPDATE_ROLL_VALUE_INDEX 16
#define YPR_UPDATE_COMPASS_VALUE_INDEX 23
#define YPR_UPDATE_CHECKSUM_INDEX 30
#define YPR_UPDATE_TERMINATOR_INDEX 32

// Raw Data Update Packet - e.g., !r[q1][q2][q3][q4][accelx][accely][accelz][magx][magy][magz][checksum][cr][lf]

#define MSGID_RAW_UPDATE 'r'
#define RAW_UPDATE_MESSAGE_LENGTH  				53      
#define RAW_UPDATE_QUAT1_VALUE_INDEX  			 2
#define RAW_UPDATE_QUAT2_VALUE_INDEX  			 6
#define RAW_UPDATE_QUAT3_VALUE_INDEX 			10
#define RAW_UPDATE_QUAT4_VALUE_INDEX 			14
#define RAW_UPDATE_ACCEL_X_VALUE_INDEX  		18
#define RAW_UPDATE_ACCEL_Y_VALUE_INDEX  		22
#define RAW_UPDATE_ACCEL_Z_VALUE_INDEX  		26	
#define RAW_UPDATE_MAG_X_VALUE_INDEX			30
#define RAW_UPDATE_MAG_Y_VALUE_INDEX            34
#define RAW_UPDATE_MAG_Z_VALUE_INDEX            38
#define RAW_UPDATE_TEMP_VALUE_INDEX				42
#define RAW_UPDATE_CHECKSUM_INDEX               49
#define RAW_UPDATE_TERMINATOR_INDEX             51

// EnableStream Command Packet - e.g., !S[stream type][checksum][cr][lf]

#define MSGID_STREAM_CMD 'S'
#define STREAM_CMD_MESSAGE_LENGTH 7
#define STREAM_CMD_STREAM_TYPE_YPR MSGID_YPR_UPDATE
#define STREAM_CMD_STREAM_TYPE_RAW MSGID_RAW_UPDATE
#define STREAM_CMD_STREAM_TYPE_INDEX 2
#define STREAM_CMD_CHECKSUM_INDEX 3
#define STREAM_CMD_TERMINATOR_INDEX 5

// EnableStream Response Packet - e.g., !s[stream type][gyro full scale range][accel full scale range][update rate hz][yaw_offset_degrees][flags][checksum][cr][lf]
#define MSG_ID_STREAM_RESPONSE 's'
#define STREAM_RESPONSE_MESSAGE_LENGTH 30
#define STREAM_RESPONSE_STREAM_TYPE_INDEX 2
#define STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE 3
#define STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE 7
#define STREAM_RESPONSE_UPDATE_RATE_HZ 11
#define STREAM_RESPONSE_YAW_OFFSET_DEGREES 15
#define STREAM_RESPONSE_FLAGS 22
#define STREAM_RESPONSE_CHECKSUM_INDEX 26
#define STREAM_RESPONSE_TERMINATOR_INDEX 28

#include <stdio.h>
#include <stdlib.h>

#define IMU_PROTOCOL_MAX_MESSAGE_LENGTH RAW_UPDATE_MESSAGE_LENGTH

class IMUProtocol
{

public:

static int encodeYPRUpdate( char *protocol_buffer, float yaw, float pitch, float roll, float compass_heading )
{
  // Header
  protocol_buffer[0] = PACKET_START_CHAR;
  protocol_buffer[1] = MSGID_YPR_UPDATE;
  
  // Data
  encodeProtocolFloat( yaw,    &protocol_buffer[YPR_UPDATE_YAW_VALUE_INDEX] );
  encodeProtocolFloat( pitch,  &protocol_buffer[YPR_UPDATE_PITCH_VALUE_INDEX] );
  encodeProtocolFloat( roll,    &protocol_buffer[YPR_UPDATE_ROLL_VALUE_INDEX] );
  encodeProtocolFloat( compass_heading, &protocol_buffer[YPR_UPDATE_COMPASS_VALUE_INDEX] );
  
  // Footer
  encodeTermination( protocol_buffer, YPR_UPDATE_MESSAGE_LENGTH, YPR_UPDATE_MESSAGE_LENGTH - 4 );

  return YPR_UPDATE_MESSAGE_LENGTH;
}

static int encodeRawUpdate( char *protocol_buffer, 
							uint16_t q1, uint16_t q2, uint16_t q3, uint16_t q4, 
							uint16_t accel_x, uint16_t accel_y, uint16_t accel_z,
							int16_t mag_x, int16_t mag_y, int16_t mag_z,
							float temp_c )
{
  // Header
  protocol_buffer[0] = PACKET_START_CHAR;
  protocol_buffer[1] = MSGID_RAW_UPDATE;
  
  // Data
  encodeProtocolUint16( q1,    				&protocol_buffer[RAW_UPDATE_QUAT1_VALUE_INDEX] );
  encodeProtocolUint16( q2,    				&protocol_buffer[RAW_UPDATE_QUAT2_VALUE_INDEX] );
  encodeProtocolUint16( q3,    				&protocol_buffer[RAW_UPDATE_QUAT3_VALUE_INDEX] );
  encodeProtocolUint16( q4,    				&protocol_buffer[RAW_UPDATE_QUAT4_VALUE_INDEX] );
  encodeProtocolUint16( accel_x,			&protocol_buffer[RAW_UPDATE_ACCEL_X_VALUE_INDEX] );
  encodeProtocolUint16( accel_y,			&protocol_buffer[RAW_UPDATE_ACCEL_Y_VALUE_INDEX] );
  encodeProtocolUint16( accel_z,			&protocol_buffer[RAW_UPDATE_ACCEL_Z_VALUE_INDEX] );
  encodeProtocolUint16( (uint16_t)mag_x,	&protocol_buffer[RAW_UPDATE_MAG_X_VALUE_INDEX] );
  encodeProtocolUint16( (uint16_t)mag_y,	&protocol_buffer[RAW_UPDATE_MAG_Y_VALUE_INDEX] );
  encodeProtocolUint16( (uint16_t)mag_z,	&protocol_buffer[RAW_UPDATE_MAG_Z_VALUE_INDEX] );
  encodeProtocolFloat(  temp_c,				&protocol_buffer[RAW_UPDATE_TEMP_VALUE_INDEX] );
  
  // Footer
  encodeTermination( protocol_buffer, RAW_UPDATE_MESSAGE_LENGTH, RAW_UPDATE_MESSAGE_LENGTH - 4 );

  return RAW_UPDATE_MESSAGE_LENGTH;
}

static int encodeStreamCommand( char *protocol_buffer, char stream_type)
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

static int encodeStreamResponse( char *protocol_buffer, char stream_type, 
									uint16_t gyro_fsr_dps, uint16_t accel_fsr_g, uint16_t update_rate_hz, float yaw_offset_degrees, uint16_t flags)
{
  // Header
  protocol_buffer[0] = PACKET_START_CHAR;
  protocol_buffer[1] = MSG_ID_STREAM_RESPONSE;
  
  // Data
  protocol_buffer[STREAM_RESPONSE_STREAM_TYPE_INDEX] = stream_type;
  encodeProtocolUint16( gyro_fsr_dps, &protocol_buffer[STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE] );
  encodeProtocolUint16( accel_fsr_g, &protocol_buffer[STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE] );
  encodeProtocolUint16( update_rate_hz, &protocol_buffer[STREAM_RESPONSE_UPDATE_RATE_HZ] );
  encodeProtocolFloat(  yaw_offset_degrees, &protocol_buffer[STREAM_RESPONSE_YAW_OFFSET_DEGREES] );
  encodeProtocolUint16(  flags, &protocol_buffer[STREAM_RESPONSE_FLAGS] );
 
  // Footer
  encodeTermination( protocol_buffer, STREAM_RESPONSE_MESSAGE_LENGTH, STREAM_RESPONSE_MESSAGE_LENGTH - 4 );

  return STREAM_RESPONSE_MESSAGE_LENGTH;
}

static int decodeStreamResponse( char *buffer, int length, 
									char& stream_type, uint16_t& gyro_fsr_dps, uint16_t& accel_fsr_g, uint16_t& update_rate_hz,
									float& yaw_offset_degrees, uint16_t& flags )
{
  if ( length < STREAM_RESPONSE_MESSAGE_LENGTH ) return 0;
  if ( ( buffer[0] == PACKET_START_CHAR ) && ( buffer[1] == MSG_ID_STREAM_RESPONSE ) )
  {
    if ( !verifyChecksum( buffer, STREAM_RESPONSE_CHECKSUM_INDEX ) ) return 0;

	stream_type			= buffer[2];
    gyro_fsr_dps   		= decodeProtocolUint16( &buffer[STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE] );
    accel_fsr_g   		= decodeProtocolUint16( &buffer[STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE] );
    update_rate_hz 		= decodeProtocolUint16( &buffer[STREAM_RESPONSE_UPDATE_RATE_HZ] );
    yaw_offset_degrees 	= decodeProtocolFloat( &buffer[STREAM_RESPONSE_YAW_OFFSET_DEGREES] );
    flags				= decodeProtocolUint16( &buffer[STREAM_RESPONSE_FLAGS] );
  }
  return STREAM_RESPONSE_MESSAGE_LENGTH;
}


static int decodeStreamCommand( char *buffer, int length, char& stream_type )
{
  if ( length < STREAM_CMD_MESSAGE_LENGTH ) return 0;
  if ( ( buffer[0] == '!' ) && ( buffer[1] == MSGID_STREAM_CMD ) )
  {
    if ( !verifyChecksum( buffer, STREAM_CMD_CHECKSUM_INDEX ) ) return 0;

    stream_type = buffer[STREAM_CMD_STREAM_TYPE_INDEX];
  }
  return STREAM_CMD_MESSAGE_LENGTH;
}

static int decodeYPRUpdate( char *buffer, int length, float& yaw, float& pitch, float& roll, float& compass_heading )
{
  if ( length < YPR_UPDATE_MESSAGE_LENGTH ) return 0;
  if ( ( buffer[0] == '!' ) && ( buffer[1] == 'y' ) )
  {
    if ( !verifyChecksum( buffer, YPR_UPDATE_CHECKSUM_INDEX ) ) return 0;

    yaw   = decodeProtocolFloat( &buffer[YPR_UPDATE_YAW_VALUE_INDEX] );
    pitch = decodeProtocolFloat( &buffer[YPR_UPDATE_PITCH_VALUE_INDEX] );
    roll  = decodeProtocolFloat( &buffer[YPR_UPDATE_ROLL_VALUE_INDEX] );
	compass_heading = decodeProtocolFloat( &buffer[YPR_UPDATE_COMPASS_VALUE_INDEX] );
  }
  return YPR_UPDATE_MESSAGE_LENGTH;
}

static int decodeRawUpdate( char *buffer, int length, 
							uint16_t& q1, uint16_t& q2, uint16_t& q3, uint16_t& q4,
							uint16_t& accel_x, uint16_t& accel_y, uint16_t& accel_z,
							int16_t& mag_x, int16_t& mag_y, int16_t& mag_z,
							float& temp_c )
{
  if ( length < RAW_UPDATE_MESSAGE_LENGTH ) return 0;
  if ( ( buffer[0] == PACKET_START_CHAR ) && ( buffer[1] == MSGID_RAW_UPDATE ) )
  {
    if ( !verifyChecksum( buffer, RAW_UPDATE_CHECKSUM_INDEX ) ) return 0;

    q1   	= decodeProtocolUint16( &buffer[RAW_UPDATE_QUAT1_VALUE_INDEX] );
    q2   	= decodeProtocolUint16( &buffer[RAW_UPDATE_QUAT2_VALUE_INDEX] );
    q3   	= decodeProtocolUint16( &buffer[RAW_UPDATE_QUAT3_VALUE_INDEX] );
    q4   	= decodeProtocolUint16( &buffer[RAW_UPDATE_QUAT4_VALUE_INDEX] );
    accel_x	= decodeProtocolUint16( &buffer[RAW_UPDATE_ACCEL_X_VALUE_INDEX] );
    accel_y	= decodeProtocolUint16( &buffer[RAW_UPDATE_ACCEL_Y_VALUE_INDEX] );
    accel_z	= decodeProtocolUint16( &buffer[RAW_UPDATE_ACCEL_Z_VALUE_INDEX] );
    mag_x	= (int16_t)decodeProtocolUint16( &buffer[RAW_UPDATE_MAG_X_VALUE_INDEX] );
    mag_y	= (int16_t)decodeProtocolUint16( &buffer[RAW_UPDATE_MAG_Y_VALUE_INDEX] );
    mag_z	= (int16_t)decodeProtocolUint16( &buffer[RAW_UPDATE_MAG_Z_VALUE_INDEX] );
	temp_c  = decodeProtocolFloat(  &buffer[RAW_UPDATE_TEMP_VALUE_INDEX] );
  }
  return RAW_UPDATE_MESSAGE_LENGTH;
}

protected:

static void encodeTermination( char *buffer, int total_length, int content_length )
{
  if ( ( total_length >= (CHECKSUM_LENGTH + TERMINATOR_LENGTH) ) && ( total_length >= content_length + (CHECKSUM_LENGTH + TERMINATOR_LENGTH) ) )
  {
    // Checksum 
    unsigned char checksum = 0;
    for ( int i = 0; i < content_length; i++ )
    {
      checksum += buffer[i];
    }
    // convert checksum to two ascii bytes
    sprintf(&buffer[content_length], "%02X", checksum);
    // Message Terminator
    sprintf(&buffer[content_length + CHECKSUM_LENGTH], "%s","\r\n");
  }
}

// Formats a float as follows
//
// e.g., -129.235
//
// "-129.24"
//
// e.g., 23.4
//
// "+023.40"

static void encodeProtocolFloat( float f, char* buff )
{
  int temp1 = abs((int)((f - (int)f) * 100));
  if ( f < 0 ) buff[0] = '-'; else buff[0] = ' ';
  sprintf(&buff[1],"%03d.%02d", abs((int)f), temp1);
}

static void encodeProtocolUint16( uint16_t value, char* buff )
{
  sprintf(&buff[0],"%04X", value );
}

static uint16_t decodeProtocolUint16( char *uint16_string )
{
	uint16_t decoded_uint16 = 0;
	unsigned int shift_left = 12;
	for ( int i = 0; i < 4; i++ ) 
	{
		unsigned char digit = uint16_string[i] <= '9' ? uint16_string[i] - '0' : ((uint16_string[i] - 'A') + 10);
		decoded_uint16 += (((uint16_t)digit) << shift_left);
		shift_left -= 4;
	}
	return decoded_uint16;  
}  


static bool verifyChecksum( char *buffer, int content_length )
{
    // Calculate Checksum
    unsigned char checksum = 0;
    for ( int i = 0; i < content_length; i++ )
    {
      checksum += buffer[i];
    }

    // Decode Checksum
    unsigned char decoded_checksum = decodeChecksum( &buffer[content_length] );
    
    return ( checksum == decoded_checksum );
}

static unsigned char decodeChecksum( char *checksum )
{
	unsigned char first_digit = checksum[0] <= '9' ? checksum[0] - '0' : ((checksum[0] - 'A') + 10);
	unsigned char second_digit = checksum[1] <= '9' ? checksum[1] - '0' : ((checksum[1] - 'A') + 10);
	unsigned char decoded_checksum = (first_digit * 16) + second_digit;
	return decoded_checksum;  
}  

static float decodeProtocolFloat( char *buffer )
{
  char temp[PROTOCOL_FLOAT_LENGTH+1];
  for ( int i = 0; i < PROTOCOL_FLOAT_LENGTH; i++ )
  {
    temp[i] = buffer[i];
  }
  temp[PROTOCOL_FLOAT_LENGTH] = 0;
  return atof(temp);
}

};

#endif // _IMU_PROTOCOL_H_
