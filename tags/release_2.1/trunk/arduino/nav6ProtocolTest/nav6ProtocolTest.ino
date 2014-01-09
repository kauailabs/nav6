#include "Wire.h"
#include "IMUProtocol.h"

void setup()
{
 Serial.begin(57600);
 Serial.println("Init"); 
}

char protocol_buff[IMU_PROTOCOL_MAX_MESSAGE_LENGTH];

float yaw = 1.06;
float pitch = 162.1;
float roll = -13.28;
float compass_heading = 17.5;

uint16_t q1 = 0xAAAA;
uint16_t q2 = 0xBBBB;
uint16_t q3 = 0xCCCC;
uint16_t q4 = 0xDDDD;

uint16_t a_x = 0x0101;
uint16_t a_y = 0x2323;
uint16_t a_z = 0x4545;

int16_t m_x = 0x6767;
int16_t m_y = 0x8989;
int16_t m_z = 0xEFEF;

float temp_c = 36.5;

void loop()
{
  int num_bytes = IMUProtocol::encodeYPRUpdate( protocol_buff, yaw, pitch, roll, compass_heading );
  Serial.write((byte *)protocol_buff,num_bytes);
  delay(1000);
  float decodedyaw, decodedpitch, decodedroll, decodedcompassheading;
  if ( IMUProtocol::decodeYPRUpdate( protocol_buff, num_bytes, decodedyaw, decodedpitch, decodedroll, decodedcompassheading ) )
  {
    Serial.print("Decoded.  Yaw:  "  );
    Serial.print(decodedyaw);
    Serial.print(" Pitch:  ");
    Serial.print(decodedpitch);
    Serial.print(" Roll:  " );
    Serial.println(decodedroll);
    Serial.print(" Heading:  " );
    Serial.println(decodedcompassheading);
  }
  else
  {
    Serial.println("Error Decoding YPR");
  }
  num_bytes = IMUProtocol::encodeRawUpdate(protocol_buff,q1,q2,q3,q4,a_x,a_y,a_z,m_x,m_y,m_z,temp_c);
  Serial.write((byte *)protocol_buff,num_bytes);
  delay(1000);
  uint16_t decoded_q1, decoded_q2, decoded_q3, decoded_q4, decoded_ax, decoded_ay, decoded_az;
  int16_t decoded_mx, decoded_my, decoded_mz;
  float decoded_temp_c;
  if ( IMUProtocol::decodeRawUpdate( protocol_buff, num_bytes, 
                                      decoded_q1, decoded_q2, decoded_q3, decoded_q4,
                                      decoded_ax, decoded_ay, decoded_az,
                                      decoded_mx, decoded_my, decoded_mz,
                                      decoded_temp_c ) )
  {
    Serial.print("Decoded.  Q1:  "  );
    Serial.print(decoded_q1,HEX);
    Serial.print(" Q2:  ");
    Serial.print(decoded_q2,HEX);
    Serial.print(" Q3:  ");
    Serial.print(decoded_q3,HEX);
    Serial.print(" Q4:  ");
    Serial.print(decoded_q4,HEX);
    Serial.print(" AX:  ");
    Serial.print(decoded_ax,HEX);
    Serial.print(" AY:  ");
    Serial.print(decoded_ay,HEX);
    Serial.print(" AZ:  ");
    Serial.print(decoded_az,HEX);
    Serial.print(" MX:  ");
    Serial.print((uint16_t)decoded_mx,HEX);
    Serial.print(" MY:  ");
    Serial.print((uint16_t)decoded_my,HEX);
    Serial.print(" MZ:  ");
    Serial.print((uint16_t)decoded_mz,HEX);
    Serial.print(" Temp (C):  ");
    Serial.print(decoded_temp_c);
    Serial.println();
  }
  else
  {
    Serial.println("Error Decoding RAW");
  }
  num_bytes = IMUProtocol::encodeStreamCommand(protocol_buff,MSGID_RAW_UPDATE);
  Serial.write((byte *)protocol_buff,num_bytes);
  delay(1000);
  char decoded_stream_id;
  if ( IMUProtocol::decodeStreamCommand( protocol_buff, num_bytes, decoded_stream_id ) )
  {
    Serial.print("Stream type:  ");
    Serial.println(decoded_stream_id);
  }
  else
  {
    Serial.println("Error Decoding Stream Command");
  } 
  num_bytes = IMUProtocol::encodeStreamResponse(protocol_buff,MSGID_RAW_UPDATE,2000,2,100,123.45,(uint16_t)(0.1*16384),(uint16_t)(0.2*16384),(uint16_t)(0.3*16384),(uint16_t)(0.4*16384),0xABCD);
  Serial.write((byte *)protocol_buff,num_bytes);
  delay(1000);
  uint16_t decoded_gyro_fsr_dps, decoded_accel_fsr_g, decoded_update_rate_hz, decoded_flags;
  float decoded_yaw_offset_degrees;
  if ( IMUProtocol::decodeStreamResponse( protocol_buff, num_bytes, decoded_stream_id,decoded_gyro_fsr_dps, decoded_accel_fsr_g, decoded_update_rate_hz, 
        decoded_yaw_offset_degrees, decoded_q1, decoded_q2, decoded_q3, decoded_q4, decoded_flags ) )
  {
    Serial.print("Stream type:  ");
    Serial.print(decoded_stream_id);
    Serial.print(" Gyro FSR (DPS):  ");
    Serial.print(decoded_gyro_fsr_dps);
    Serial.print(" Accel FSR (G):  ");
    Serial.print(decoded_accel_fsr_g);
    Serial.print(" Update Rate (Hz):  ");
    Serial.print(decoded_update_rate_hz);
    Serial.print(" Yaw Offset (Degrees):  ");
    Serial.print(decoded_yaw_offset_degrees);
    Serial.print(" Q1:  ");
    Serial.print(decoded_q1);
    Serial.print(" Q2:  ");
    Serial.print(decoded_q2);
    Serial.print(" Q3:  ");
    Serial.print(decoded_q3);
    Serial.print(" Q4:  ");
    Serial.print(decoded_q4);
    Serial.print(" Flags:  ");
    Serial.print(decoded_flags);
    Serial.println();
  }
  else
  {
    Serial.println("Error Decoding Stream Command");
  } 
  
}

