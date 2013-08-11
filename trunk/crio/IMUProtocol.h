#ifndef _IMU_PROTOCOL_H_
#define _IMU_PROTOCOL_H_

#define PACKET_START_CHAR '!'
#define PROTOCOL_FLOAT_LENGTH 7
#define CHECKSUM_LENGTH 2
#define TERMINATOR_LENGTH 2

// Yaw/Pitch/Roll (YPR) Update Packet

#define MSGID_YPR_UPDATE 'y'
#define YPR_UPDATE_MESSAGE_LENGTH 27 	// e.g., !y[yaw][pitch][roll][checksum][cr][lf]
						//       where yaw, pitch, roll are floats
						//		 where checksum is 2 ascii-bytes of HEX checksum (all bytes before checksum)
#define YPR_UPDATE_YAW_VALUE_INDEX 2
#define YPR_UPDATE_PITCH_VALUE_INDEX 9
#define YPR_UPDATE_ROLL_VALUE_INDEX 16
#define YPR_UPDATE_CHECKSUM_INDEX 23
#define YPR_UPDATE_TERMINATOR_INDEX 25

#include <stdio.h>
#include <stdlib.h>

class IMUProtocol
{

public:

static int encodeYPRUpdate( char *protocol_buffer, float yaw, float pitch, float roll )
{
  // Header
  protocol_buffer[0] = PACKET_START_CHAR;
  protocol_buffer[1] = MSGID_YPR_UPDATE;
  
  // Data
  encodeProtocolFloat( yaw,    &protocol_buffer[YPR_UPDATE_YAW_VALUE_INDEX] );
  encodeProtocolFloat( pitch,  &protocol_buffer[YPR_UPDATE_PITCH_VALUE_INDEX] );
  encodeProtocolFloat( roll,    &protocol_buffer[YPR_UPDATE_ROLL_VALUE_INDEX] );
  
  // Footer
  encodeTermination( protocol_buffer, YPR_UPDATE_MESSAGE_LENGTH, YPR_UPDATE_MESSAGE_LENGTH - 4 );

  return YPR_UPDATE_MESSAGE_LENGTH;
}

static int decodeYPRUpdate( char *buffer, int length, float& yaw, float& pitch, float& roll )
{
  if ( length < YPR_UPDATE_MESSAGE_LENGTH ) return 0;
  if ( ( buffer[0] == '!' ) && ( buffer[1] == 'y' ) )
  {
    if ( !verifyChecksum( buffer, YPR_UPDATE_CHECKSUM_INDEX ) ) return 0;

    yaw   = decodeProtocolFloat( &buffer[YPR_UPDATE_YAW_VALUE_INDEX] );
    pitch = decodeProtocolFloat( &buffer[YPR_UPDATE_PITCH_VALUE_INDEX] );
    roll  = decodeProtocolFloat( &buffer[YPR_UPDATE_ROLL_VALUE_INDEX] );
    return YPR_UPDATE_MESSAGE_LENGTH;
  }
  return 0;
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