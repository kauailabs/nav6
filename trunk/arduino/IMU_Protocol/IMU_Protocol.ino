#include "IMUProtocol.h"

void setup()
{
 Serial.begin(57600);
 Serial.println("Init"); 
}

char protocol_buff[256];

float yaw = 1.06;
float pitch = 162.1;
float roll = -13.28;
float compass_heading = 17.5;

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
    Serial.println("Error Decoding");
  }
}

