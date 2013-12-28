/* =============================================================================
Nav6 source code is placed under the MIT license

Copyright (c) 2013 Kauai Labs

Portions of this work are based upon the I2C Dev Library by Jeff Rowberg
(www.i2cdevlib.com) which is open-source licensed under the MIT
License.  This work is also based upon the Arduino software
library which is licensed under a Creative Commons license.

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
=============================================================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev, MPU6050, HMC5883L must be installed as libraries, or else the 
// .cpp/.h files for these classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include <HMC5883LCalibratable.h>

HMC5883LCalibratable compass;
MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the ATMEGA328
   external interrupt #0 pin, and the HMC5883L's INT pin being connected to
   the ATMEGA328 external interrupt #1 pin.
  
   In the Arduino Library, this corresponds to
   digital I/O pin 2 and pin 3, respectively.
 * ========================================================================= */

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#define SDA_PIN A4
#define SCL_PIN A5

#define OUTPUT_IMU_PROTOCOL

#ifdef OUTPUT_IMU_PROTOCOL
#include "IMUProtocol.h"
char protocol_buffer[64];
#endif

#define LED_PIN 13
boolean blinkState = false;

// MPU control/status vars

boolean dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;       // holds actual interrupt status byte from MPU
uint8_t devStatus;          // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;         // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];     // FIFO storage buffer

// orientation/motion vars

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                INTERRUPT SERVICE ROUTINES                ===
// ================================================================

volatile boolean mpuInterrupt = false;         // true if MPU interrupt occurs
void dmpDataReady() {
    mpuInterrupt = true;
}

volatile boolean compass_data_ready = false;
void compassDataAvailable() {
  compass_data_ready = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    // reset i2c bus
/*    pinMode(SDA_PIN,INPUT);
    pinMode(SCL_PIN,OUTPUT);
    
    // Clock through up to 1000 bits
    for ( int i = 0; i < 1000; i++ ) 
    {
      digitalWrite(SCL_PIN,HIGH);
      digitalWrite(SCL_PIN,LOW);
      digitalWrite(SCL_PIN,HIGH);
    }
    // send a stop
    digitalWrite(SDA_PIN,HIGH);
    digitalWrite(SDA_PIN,LOW);
*/

  // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    // Disable internal I2C pull-ups
    cbi(PORTC, 4);
    cbi(PORTC, 5);



    // initialize serial communication
    Serial.begin(57600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize devices
    Serial.println(F("Init I2C devices..."));
    mpu.initialize();
    compass.initialize();
    
    uint8_t gain = compass.getGain();    
    Serial.print("Default Compass Gain:  ");
    Serial.println(gain);
    Serial.println("Calibrating Compass...   ");
    Serial.flush();
    delay(10);
    if ( compass.calibrate(gain,100) ) {
      Serial.println("Success");
    }
    else {
      Serial.println("Failed");
    }
    
    compass.initialize();
    // Set Compass to "Continuous" mode
    // This makes reading data more efficient, at the expense of some 
    // additional current draw.
    // enable the compass interrupt
    attachInterrupt(1,compassDataAvailable,RISING);
    // Initiate reading from magnetometer, this should trigger an interrupt.
    compass.setMode(HMC5883L_MODE_CONTINUOUS);
    compass.getHeadingX();
    compass.setMode(HMC5883L_MODE_CONTINUOUS);

    // Verify connection to MPU
    Serial.println(F("Connecting to MPU..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connected") : F("MPU6050 connection failed"));

    // load firmware to and configure the MPU-6050 DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    if (devStatus == 0) {

        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling ISR (Arduino EXT INT 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows 
        // when the MPU Interrupt next occurs.
        Serial.println(F("DMP ready! Awaiting 1st interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Init failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
}

#define MPU_CALIBRATION_STATE_CALIBRATING 0 // Waiting for MPU to complete internal calibration
#define MPU_CALIBRATION_STATE_ACCUMULATE  1 // Accumulate Yaw/Pitch/Roll offsets
#define MPU_CALIBRATION_STATE_COMPLETE    2 // Normal Operation

int calibration_state = MPU_CALIBRATION_STATE_CALIBRATING;
                            
int accumulator_count = 0;

float calibrated_x_offset = 0.0;
float calibrated_y_offset = 0.0;
float calibrated_z_offset = 0.0;

float x_accumulator = 0.0;
float y_accumulator = 0.0;
float z_accumulator = 0.0;

// angle in radians = angle in degrees * Pi / 180 
const float degrees_to_radians = M_PI / 180.0;
// angle in degrees = angle in radians * 180 / Pi
const float radians_to_degrees = 180.0 / M_PI;

float compass_heading_radians = 0;
float compass_heading_degrees = 0;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

#define STARTUP_CALIBRATION_DELAY_MS        19000
#define CALIBRATED_OFFSET_AVERAGE_PERIOD_MS  1000

void loop() {
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    boolean accumulate = false;
    if ( calibration_state == MPU_CALIBRATION_STATE_CALIBRATING )
    {
      if ( millis() >= STARTUP_CALIBRATION_DELAY_MS )
      {
        calibration_state = MPU_CALIBRATION_STATE_ACCUMULATE;
      }
    }
    if ( calibration_state == MPU_CALIBRATION_STATE_ACCUMULATE )
    {
      accumulate = true;
      if ( millis() >= (STARTUP_CALIBRATION_DELAY_MS + CALIBRATED_OFFSET_AVERAGE_PERIOD_MS) )
      {
        accumulate = false;
        calibrated_x_offset = x_accumulator / accumulator_count;
        calibrated_y_offset = 0; // y_accumulator / accumulator_count;
        calibrated_z_offset = 0; // z_accumulator / accumulator_count;
        calibration_state = MPU_CALIBRATION_STATE_COMPLETE;
      }
      else
      {
        accumulator_count++;
      }
    }

    // Read compass heading data if it has been updated recently.

    if ( compass_data_ready ) 
    {
      // Read latest heading from compass
      // Note that the compass heading is tilt compensated based upon
      // previous pitch/roll readings from the MPU
      compass_heading_radians = compass.compassHeadingTiltCompensatedRadians(-ypr[1], ypr[2]);
      compass_heading_degrees = compass_heading_radians * radians_to_degrees;
      
      // Adjust compass for board oreintation,
      // and modify range from -180-180 to
      // 0-360 degrees
      
      compass_heading_degrees -= 90.0;
      if ( compass_heading_degrees < 0 ) {
        compass_heading_degrees += 360; 
      }
      compass_data_ready = false;
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // if quaternions are desired, they may be acquired as follows:

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        // Display readable euler angles (degrees) as follows:

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            
            float x = euler[0] * radians_to_degrees;
            float y = euler[1] * radians_to_degrees;
            float z = euler[2] * radians_to_degrees;

            if ( accumulate )
            {
              x_accumulator += x;
              y_accumulator += y;
              z_accumulator += z;
            }

            x -= calibrated_x_offset;
            y -= calibrated_y_offset;
            z -= calibrated_z_offset;
            if ( x < -180 ) x += 360;
            if ( x > 180 ) x -= 360;
            if ( y < -180 ) y += 360;
            if ( y > 180 ) y -= 360;
            if ( z < -180 ) z += 360;
            if ( z > 180 ) z -= 360;

            Serial.print("euler\t");
            Serial.print(x);
            Serial.print("\t");
            Serial.print(y);
            Serial.print("\t");
            Serial.println(z);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            float x = ypr[0] * radians_to_degrees;
            float y = ypr[1] * radians_to_degrees;
            float z = ypr[2] * radians_to_degrees;

            if ( accumulate )
            {
              x_accumulator += x;
              y_accumulator += y;
              z_accumulator += z;
            }

            x -= calibrated_x_offset;
            y -= calibrated_y_offset;
            z -= calibrated_z_offset;
            if ( x < -180 ) x += 360;
            if ( x > 180 ) x -= 360;
            if ( y < -180 ) y += 360;
            if ( y > 180 ) y -= 360;
            if ( z < -180 ) z += 360;
            if ( z > 180 ) z -= 360;

            Serial.print("ypr\t");
            Serial.print(x);
            Serial.print("\t");
            Serial.print(y);
            Serial.print("\t");
            Serial.println(z);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        #ifdef OUTPUT_IMU_PROTOCOL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            float yaw_deg = ypr[0] * radians_to_degrees;
            float pitch_deg = ypr[1] * radians_to_degrees;
            float roll_deg = ypr[2] * radians_to_degrees;

            if ( accumulate )
            {
              x_accumulator += yaw_deg;
              y_accumulator += pitch_deg;
              z_accumulator += roll_deg;
            }

            yaw_deg -= calibrated_x_offset;
            pitch_deg -= calibrated_y_offset;
            roll_deg -= calibrated_z_offset;
            if ( yaw_deg < -180 ) yaw_deg += 360;
            if ( yaw_deg > 180 ) yaw_deg -= 360;
            if ( pitch_deg < -180 ) pitch_deg += 360;
            if ( pitch_deg > 180 ) pitch_deg -= 360;
            if ( roll_deg < -180 ) roll_deg += 360;
            if ( roll_deg > 180 ) roll_deg -= 360;

            int num_bytes = IMUProtocol::encodeYPRUpdate(protocol_buffer, yaw_deg, pitch_deg, roll_deg,compass_heading_degrees);     
            Serial.write((unsigned char *)protocol_buffer, num_bytes);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
