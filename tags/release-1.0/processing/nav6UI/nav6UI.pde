// nav6 IMU User Interface/Demonstration software demonstration
// 12/31/2013 by Kauai Labs (scott@kauailabs.com)
// 
// Based upon the MPU-6050 Demonstration software developed
// 6/20/2012 by Jeff Rowberg <jeff@rowberg.net>
/* ============================================
This softwware is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Additions Copyright (c) 2013 Scott Libert

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

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;
import com.kauailabs.nav6.*;

import java.util.*;

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
//    (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

// NOTE:  Also requires the nav6 library to be installed in order to run properly.
// 1. Download from http://code.google.com/p/nav6
// 2. Place the nav6.jar library in the libraries subdirectory 

ToxiclibsSupport gfx;

Serial port;                         // The serial port
byte[] protocol_buffer = new byte[256];  // Buffer for received packets
int serialCount = 0;                 // current packet byte position
int aligned = 0;
int interval = 0;

float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];

float current_temp_c = 0.0;

float tilt_compensated_heading_degrees = 0.0;

float yaw_degrees = 0.0;
float pitch_degrees = 0.0;
float roll_degrees = 0.0;

float linear_acceleration_x = 0.0;
float linear_acceleration_y = 0.0;
float linear_acceleration_z = 0.0;

float world_linear_acceleration_x;
float world_linear_acceleration_y;
float world_linear_acceleration_z;
            
static final int ACCEL_HISTORY_SIZE = 10;
float world_acceleration_history[] = new float[ACCEL_HISTORY_SIZE];
int world_acceleration_history_index = 0;
float world_acceleration_recent_avg = 0.0;

int last_update_ms = 0;

int updateRateHz = 0;
int gyroFSRDPS = 0;
int accelFSRG = 0;
float yaw_offset_degrees = 0.0;
short nav6_flags;

boolean initial_calibration_in_process = false;

boolean multiple_serial_ports = false;
boolean serial_port_open_error = false;
String attempted_open_serial_port_name = "";
String opened_port_name = "";

// this table is used to store all command line parameters
// in the form: name=value
static Hashtable params=new Hashtable();
 
// here we overwrite PApplet's main entry point (for application mode)
// we're parsing all commandline arguments and copy only the relevant ones
 
static public void main(String args[]) {
  String[] newArgs=new String[args.length+1];
  /*********************************************************
  /* IMPORTANT: replace this with the name of your sketch  *
  /*********************************************************/
  newArgs[0]="nav6UI";
 
  for(int i=0; i<args.length; i++) {
    newArgs[i+1]=args[i];
    if (args[i].indexOf("=")!=-1) {
      String[] pair=split(args[i], '=');
      params.put(pair[0],pair[1]);
    }
  }
  // pass on to PApplet entry point
  PApplet.main(newArgs);
}
 

void enableRawUpdateMode() {

    int length = IMUProtocol.encodeStreamCommand(protocol_buffer, (byte) IMUProtocol.STREAM_CMD_STREAM_TYPE_RAW);
    if (length != 0) {
      byte[] stream_command = new byte[length];
      arrayCopy(protocol_buffer,0,stream_command,0,length);
      port.write(stream_command);
      println(new String(protocol_buffer));
    }
    else {
      println("Error encoding stream command.");
    }
}

void setup() {
    // 500px square viewport using OpenGL rendering
    size(500, 500, OPENGL);
    gfx = new ToxiclibsSupport(this);

    print("Display Height:  ");
    println(height);
    print("Display Width:  ");
    println(width);

    // setup lights and antialiasing
    lights();
    smooth();
  
    print("Enumerating Serial Ports...");
    // display serial port list for debugging/clarity
    try {
      println(Serial.list());
    } catch (Exception e){
      println("No valid serial ports.  Exiting...");
      exit();
    }    
    print("Done Enumerating Serial Ports.");

    String theportname = "";
    
    if ( Serial.list().length < 1 ) {
      println("No valid serial ports.");
      return;    
    }
    else if ( Serial.list().length > 1 ) {
      
      if ( params.size() < 1 ) { 
        multiple_serial_ports = true;
        println("Use command line to specify serial port to use");
        println("Syntax:  port=<PortNumber>");
        return;
      }
      else {
        // select the port based upon the provided parameter
        String[] portnames = Serial.list();
        String portname = (String)params.get("port");
        print("Using command-line Specified port ");
        println(portname);
        for ( int i = 0; i < portnames.length; i++ ) {
          if ( portnames[i].equals(portname) ) {
            theportname = portnames[i];
            break;
          }
        }
      }
      
    }
    else {
      theportname = Serial.list()[0];
    }
    
     print("Opening serial port ");
     println(theportname);
    // get the first available port (use EITHER this OR the specific port code below)
    String portName = theportname;
    
    // get a specific serial port (use EITHER this OR the first-available code above)
    //String portName = "COM4";
    
    // open the serial port
    try {
      port = new Serial(this, portName, 57600);
      opened_port_name = portName;
      port.setDTR(false);
    }
    catch(Exception ex) {
      print("Error opening serial port ");
      println(portName);
      attempted_open_serial_port_name = portName;
      serial_port_open_error = true;
      return;
    }
    
    int lf=10;
    port.bufferUntil(lf);
    
    // Give the nav6 a few seconds to start up.
    delay(3000);
    
    // Send command to nav6 requesting streaming data in 'raw' format
    enableRawUpdateMode();    
    
}

void draw() {
    
  
    // black background
    background(0);
    
    if ( port == null ) {
      textSize(26);
      fill(255,0,0);
      if ( multiple_serial_ports ) {
        text("Multiple Serial ports available.",(width/2)-150,height/2);
        text(" Specify port on command line.",(width/2)-150,height/2+30);
        text("   port=<PORTNAME>",(width/2)-150,height/2+60);
      }
      else if ( serial_port_open_error ) {
        text("Unable to open serial port " + attempted_open_serial_port_name,(width/2)-200,height/2);
      }
      else {
        text("No Serial Ports available",(width/2)-150,height/2);
      }
      return;
    }
    
    if ( initial_calibration_in_process ) {
      text("Calibrating...",(width/2)-50,(height/2)-100);
    }
    
    textSize(32); 
    text("nav6", 20, 30); 
    textSize(16);
    text("Open-source IMU",20,50);
    String gyrofsr = "Gyro Range:  +/- " + gyroFSRDPS + " Deg/sec";
    text(gyrofsr,20,70);
    String accelfsr = "Accel Range:  +/- " + accelFSRG + " G";
    text(accelfsr,20,90);
    String updaterate = "Update Rate:  " + updateRateHz + " Hz";
    text(updaterate,20,110);
    
    int timeout_ms = 2000;
    if ( ( millis() - last_update_ms ) >= timeout_ms ) {
      textSize(26);
      fill(255,0,0);
      text("Disconnected",(width/2)-100,(height/2)-100);
    }
    else {
      textSize(16);
      text("Connected",width-100,30);
      text(opened_port_name,width-100,50);
    }
    
    textSize(15);
    fill(255,0,0);

    String linear_accel_x = "Accel X:  " + nfp(world_linear_acceleration_x,1,2) + " G";
    text(linear_accel_x,20,height-60);
    String linear_accel_y = "Accel Y:  " + nfp(world_linear_acceleration_y,1,2) + " G";
    text(linear_accel_y,20,height-40);
      
    String temp = "Temp:  " + current_temp_c;
    temp += " C";
    text(temp,20,height-20);
    
    String yaw     = "Yaw     :  " + nfp(yaw_degrees,3,2);
    text(yaw,width-140, height-80);
    String pitch   = "Pitch    :  " + nfp(pitch_degrees,3,2);
    text(pitch,width-140, height-60);
    String roll    = "Roll      :  " + nfp(roll_degrees,3,2);
    text(roll,width-140, height-40);
    String heading = "Heading:  " + nfs(tilt_compensated_heading_degrees,3,2);
    text(heading,width-140, height-20);
    
    String motion_state = "";
    if ( world_acceleration_recent_avg >= 0.01 ) {
      motion_state = "Moving";
    }
    text(motion_state,(width/2)-30,height-20);
    
    // translate everything to the middle of the viewport
    pushMatrix();
    translate(width / 2, height / 2);

    // 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
    // ...and other weirdness I haven't figured out yet
    //rotateY(-ypr[0]);
    //rotateZ(-ypr[1]);
    //rotateX(-ypr[2]);

    // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
    // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
    // different coordinate system orientation assumptions between Processing
    // and InvenSense DMP)
    float[] axis = quat.toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);

    // draw main body in red
    fill(255, 0, 0, 200);
    box(10, 10, 200);
    
    // draw front-facing tip in blue
    fill(0, 0, 255, 200);
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();
    
    // draw wings and tail fin in green
    fill(0, 255, 0, 200);
    beginShape(TRIANGLES);
    vertex(-100,  2, 30); vertex(0,  2, -80); vertex(100,  2, 30);  // wing top layer
    vertex(-100, -2, 30); vertex(0, -2, -80); vertex(100, -2, 30);  // wing bottom layer
    vertex(-2, 0, 98); vertex(-2, -30, 98); vertex(-2, 0, 70);  // tail left layer
    vertex( 2, 0, 98); vertex( 2, -30, 98); vertex( 2, 0, 70);  // tail right layer
    endShape();
    beginShape(QUADS);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex( 100, 2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100, 2,  30);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2, -30, 98); vertex(-2, -30, 98);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    vertex(-2, -30, 98); vertex(2, -30, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    endShape();
    
    popMatrix();
}

// The serialEvent() method will be invoked whenever one or more
// serial characters are received at the moment when a 
// line feed character (which terminates a nav6 message)
// is received.

IMUProtocol.RawUpdate raw_update = new IMUProtocol.RawUpdate();
IMUProtocol.YPRUpdate ypr_update = new IMUProtocol.YPRUpdate();
IMUProtocol.StreamResponse stream_response = new IMUProtocol.StreamResponse();

void serialEvent(Serial port) {
    
  try {
  
    boolean found_start_of_message = false;
    while ( port.available() > 0 ) {
      if ( port.readBytesUntil((int)'!',protocol_buffer) > 0 ) {
        int msg_len = port.readBytesUntil((int) '\n',protocol_buffer);
        if (msg_len > 0 ) {
          
          byte[] full_message = new byte[msg_len+1];
          full_message[0] = '!';
          arrayCopy(protocol_buffer,0,full_message,1,msg_len);
          
          print(new String(full_message));
          
          int decode_length = IMUProtocol.decodeRawUpdate(full_message, full_message.length, raw_update);
          if (decode_length != 0) {
            
            last_update_ms = millis();
            
            q[0] = ((float)raw_update.q1) / 16384.0f;
            q[1] = ((float)raw_update.q2) / 16384.0f;
            q[2] = ((float)raw_update.q3) / 16384.0f;
            q[3] = ((float)raw_update.q4) / 16384.0f;
            for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i]; // ???
            
            // set our toxilibs quaternion to new data
            quat.set(q[0], q[1], q[2], q[3]);
            
            current_temp_c = raw_update.temp_c;
            
            // below calculations are necessary for calculation of yaw/pitch/roll, 
            // and tilt-compensated compass heading
            
            // calculate gravity vector
            gravity[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
            gravity[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
            gravity[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
            // calculate Euler angles
            euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
            euler[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);
            euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
  
            // calculate yaw/pitch/roll angles
            ypr[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
            ypr[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
            ypr[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
             
            yaw_degrees = ypr[0] * (180.0/3.1415926); 
            pitch_degrees = ypr[1] * (180.0/3.1415926); 
            roll_degrees = ypr[2] * (180.0/3.1415926); 
             
            // Subtract offset, and handle potential 360 degree wrap-around
            yaw_degrees -= yaw_offset_degrees;
            if ( yaw_degrees < -180 ) yaw_degrees += 360;
            if ( yaw_degrees > 180 ) yaw_degrees -= 360;
             
            // calculate linear acceleration by 
            // removing the gravity component (+1g = +4096 in standard DMP FIFO packet)
             
            linear_acceleration_x = (((float)raw_update.accel_x) / 16384.0) - gravity[0];
            linear_acceleration_y = (((float)raw_update.accel_y) / 16384.0) - gravity[1];
            linear_acceleration_z = (((float)raw_update.accel_z) / 16384.0) - gravity[2]; 
            
            float q2[] = new float[4];
            q2[0] = 0;
            q2[1] = linear_acceleration_x;
            q2[2] = linear_acceleration_y;
            q2[3] = linear_acceleration_z;
            
            // Rotate linear acceleration so that it's relative to the world reference frame
            
            // http://www.cprogramming.com/tutorial/3d/quaternions.html
            // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
            // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
            // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1
        
            // P_out = q * P_in * conj(q)
            // - P_out is the output vector
            // - q is the orientation quaternion
            // - P_in is the input vector (a*aReal)
            // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])

            
            float[] q_product = new float[4];
            
            // calculate quaternion product
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            
            q_product[0] = q[0]*q2[0] - q[1]*q2[1] - q[2]*q2[2] - q[3]*q2[3];  // new w
            q_product[1] = q[0]*q2[1] + q[1]*q2[0] + q[2]*q2[3] - q[3]*q2[2];  // new x
            q_product[2] = q[0]*q2[2] - q[1]*q2[3] + q[2]*q2[0] + q[3]*q2[1];  // new y 
            q_product[3] = q[0]*q2[3] + q[1]*q2[2] - q[2]*q2[1] + q[3]*q2[0];  // new z

            float[] q_conjugate = new float[4];
            
            q_conjugate[0] = q[0];            
            q_conjugate[1] = -q[1];            
            q_conjugate[2] = -q[2];            
            q_conjugate[3] = -q[3];            

            float[] q_final = new float[4];
            
            q_final[0] = q_product[0]*q_conjugate[0] - q_product[1]*q_conjugate[1] - q_product[2]*q_conjugate[2] - q_product[3]*q_conjugate[3];  // new w
            q_final[1] = q_product[0]*q_conjugate[1] + q_product[1]*q_conjugate[0] + q_product[2]*q_conjugate[3] - q_product[3]*q_conjugate[2];  // new x
            q_final[2] = q_product[0]*q_conjugate[2] - q_product[1]*q_conjugate[3] + q_product[2]*q_conjugate[0] + q_product[3]*q_conjugate[1];  // new y 
            q_final[3] = q_product[0]*q_conjugate[3] + q_product[1]*q_conjugate[2] - q_product[2]*q_conjugate[1] + q_product[3]*q_conjugate[0];  // new z

            world_linear_acceleration_x = q_final[1];
            world_linear_acceleration_y = q_final[2];
            world_linear_acceleration_z = q_final[3];
             
            world_acceleration_history[world_acceleration_history_index] = abs(world_linear_acceleration_x) + abs(world_linear_acceleration_y);
            world_acceleration_history_index = (world_acceleration_history_index + 1) % ACCEL_HISTORY_SIZE;
            float world_acceleration_history_sum = 0;
            for ( int i = 0; i < ACCEL_HISTORY_SIZE; i++ ) {
              world_acceleration_history_sum += world_acceleration_history[i];
            }  
            world_acceleration_recent_avg = world_acceleration_history_sum / ACCEL_HISTORY_SIZE;
             
            // Calculate tilt-compensated compass heading
            
            float inverted_pitch = -ypr[1];
            float roll = ypr[2];
            
            float cos_roll = cos(roll);
            float sin_roll = sin(roll);
            float cos_pitch = cos(inverted_pitch);
            float sin_pitch = sin(inverted_pitch);
            
            float MAG_X = raw_update.mag_x * cos_pitch + raw_update.mag_z * sin_pitch;
            float MAG_Y = raw_update.mag_x * sin_roll * sin_pitch + raw_update.mag_y * cos_roll - raw_update.mag_z * sin_roll * cos_pitch;
            float tilt_compensated_heading_radians = atan2(MAG_Y,MAG_X);
            tilt_compensated_heading_degrees = tilt_compensated_heading_radians * (180.0 / 3.1415926);
            
            // Adjust compass for board orientation,
            // and modify range from -180-180 to
            // 0-360 degrees
          
            tilt_compensated_heading_degrees -= 90.0;
            if ( tilt_compensated_heading_degrees < 0 ) {
              tilt_compensated_heading_degrees += 360; 
            }
          }
          else {
            // If a YPR Update was received, send a stream command to switch to Raw Update Mode.
            // This case can happen if the nav6 IMU is reset after this application has completed
            // initialization.
            decode_length = IMUProtocol.decodeYPRUpdate(full_message, full_message.length, ypr_update);
            if ( decode_length > 0 ) {
              enableRawUpdateMode();
            }
            else {
              decode_length = IMUProtocol.decodeStreamResponse(full_message,full_message.length,stream_response);
              if ( decode_length > 0 ) {
                updateRateHz = stream_response.update_rate_hz;
                gyroFSRDPS = stream_response.gyro_fsr_dps;
                accelFSRG = stream_response.accel_fsr_g;
                yaw_offset_degrees = stream_response.yaw_offset_degrees;
                nav6_flags = stream_response.flags;
                if ( (nav6_flags & 0x03) < 2 ) {
                  initial_calibration_in_process = true;
                }              
                else {
                  initial_calibration_in_process = false;
                }
              }
            }
          }        
        }
      }
      else {
        byte[] buffer = port.readBytes();
        println(new String(buffer));
        port.clear();
      }
    }
  }
  catch(Exception ex ) {
    println("Exception during serialEvent()");
  }  
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        angle += angleIncrement;
    }
    endShape();
    
    // If it is not a cone, draw the circular top cap
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        
        // Center point
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
    // If it is not a cone, draw the circular bottom cap
    if (bottomRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
    
        // Center point
        vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
}

