#include "IMUProtocol.h"

void setup()
{
 Serial1.begin(57600);
 Serial1.println("Init"); 
}

char protocol_buff[256];

float yaw = 1.06;
float pitch = 162.1;
float roll = -13.28;

void loop()
{
  int num_bytes = IMUProtocol::encodeYPRUpdate( protocol_buff, yaw, pitch, roll );
  Serial1.write((byte *)protocol_buff,num_bytes);
  delay(1000);
  float decodedyaw, decodedpitch, decodedroll;
  if ( IMUProtocol::decodeYPRUpdate( protocol_buff, num_bytes, decodedyaw, decodedpitch, decodedroll ) )
  {
    Serial1.print("Decoded.  Yaw:  "  );
    Serial1.print(decodedyaw);
    Serial1.print(" Pitch:  ");
    Serial1.print(decodedpitch);
    Serial1.print(" Roll:  " );
    Serial1.println(decodedroll);
  }
  else
  {
    Serial1.println("Error Decoding");
  }
}

