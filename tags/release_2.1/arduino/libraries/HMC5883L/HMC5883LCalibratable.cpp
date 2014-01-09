/* =============================================================================
Nav6 source code is placed under the MIT license

Copyright (c) 2013 Kauai Labs

Portions of this work are based upon the FreeIMU Library by Fabio Varesano.
(www.freeimu.com) which is open-source licensed under the GPL v3
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

#include "HMC5883LCalibratable.h"

#define DEBUG_PRINT(s) Serial.println(s)

const int counts_per_milligauss[8]={  
	1370,
	1090,
	820,
	660,
	440,
	390,
	330,
	230
};

/** Default constructor
 */
HMC5883LCalibratable::HMC5883LCalibratable() : HMC5883L() {
  x_scale=1.0F;
  y_scale=1.0F;
  z_scale=1.0F;
}

// gain is a value from 0-7, corresponding to counts_per_milligaus

void HMC5883LCalibratable::calibrate(unsigned char gain) {
  x_scale=1; // get actual values
  y_scale=1;
  z_scale=1;
  I2Cdev::writeByte(devAddr,HMC5883L_RA_CONFIG_A, 0x010 + HMC5883L_BIAS_POSITIVE); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
  setGain(gain);
  float x, y, z, mx=0, my=0, mz=0, t=10;
  
  for (int i=0; i<(int)t; i++) { 
    setMode(1);
    getValues(&x,&y,&z);
    if (x>mx) mx=x;
    if (y>my) my=y;
    if (z>mz) mz=z;
  }
  
  float max=0;
  if (mx>max) max=mx;
  if (my>max) max=my;
  if (mz>max) max=mz;
  x_max=mx;
  y_max=my;
  z_max=mz;
  x_scale=max/mx; // calc scales
  y_scale=max/my;
  z_scale=max/mz;

  I2Cdev::writeByte(devAddr,HMC5883L_RA_CONFIG_A, 0x010); // set RegA/DOR back to default
}   // calibrate().

/*!
    \brief Calibrate using the self test operation.
  
    Average the values using bias mode to obtain the scale factors.

    \param gain [in] Gain setting for the sensor. See data sheet.
    \param n_samples [in] Number of samples to average together while applying the positive and negative bias.
    \return Returns false if any of the following occurs:
        # Invalid input parameters. (gain>7 or n_samples=0).
        # Id registers are wrong for the compiled device. Unfortunately, we can't distinguish between HMC5843 and HMC5883L.
        # Calibration saturates during the positive or negative bias on any of the readings.
        # Readings are outside of the expected range for bias current. 
*/
bool HMC5883LCalibratable::calibrate(unsigned char gain,unsigned int n_samples) 
{
    int xyz[3];                     // 16 bit integer values for each axis.
    long int xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
    bool bret=true;                 // Function return value.  Will return false if the wrong identifier is returned, saturation is detected or response is out of range to self test bias.
    char id[3];                     // Three identification registers should return 'H43'.
    long int low_limit, high_limit;                                    
    /*
        Make sure we are talking to the correct device.
        Hard to believe Honeywell didn't change the identifier.
    */
    if ((8>gain) && (0<n_samples)) // Notice this allows gain setting of 7 which the data sheet warns against.
    {
        id[0] = getIDA();
		id[1] = getIDB();
		id[2] = getIDC();
        if (('H' == id[0]) && ('4' == id[1]) && ('3' == id[2]))
        {   /*
                Use the positive bias current to impose a known field on each axis.
                This field depends on the device and the axis.
            */
            I2Cdev::writeByte(devAddr,HMC5883L_RA_CONFIG_A, 0x010 + HMC5883L_BIAS_POSITIVE); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
            /*
                Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
                The new gain setting is effective from the second measurement and on.
            */
            setGain(gain);                      
            setMode(HMC5883L_MODE_SINGLE);      // Change to single measurement mode.
            getHeading(&xyz[0],&xyz[1],&xyz[2]);    // Get the raw values and ignore since this reading may use previous gain.

            for (unsigned int i=0; i<n_samples; i++) 
            { 
                setMode(1);
                getHeading(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
                /*
                    Since the measurements are noisy, they should be averaged rather than taking the max.
                */
                xyz_total[0]+=xyz[0];
                xyz_total[1]+=xyz[1];
                xyz_total[2]+=xyz[2];
                /*
                    Detect saturation.
                */
                if (-(1<<12) >= min(xyz[0],min(xyz[1],xyz[2])))
                {
                    DEBUG_PRINT("HMC58x3 Self test saturated. Increase range.");
                    bret=false;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Apply the negative bias. (Same gain)
            */
            I2Cdev::writeByte(devAddr,HMC5883L_RA_CONFIG_A, 0x010 + HMC5883L_BIAS_NEGATIVE); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
            for (unsigned int i=0; i<n_samples; i++) 
            { 
                setMode(1);
                getHeading(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
                /*
                    Since the measurements are noisy, they should be averaged.
                */
                xyz_total[0]-=xyz[0];
                xyz_total[1]-=xyz[1];
                xyz_total[2]-=xyz[2];
                /*
                    Detect saturation.
                */
                if (-(1<<12) >= min(xyz[0],min(xyz[1],xyz[2])))
                {
                    DEBUG_PRINT("HMC58x3 Self test saturated. Increase range.");
                    bret=false;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Compare the values against the expected self test bias gauss.
                Notice, the same limits are applied to all axis.
            */
            low_limit =SELF_TEST_LOW_LIMIT *counts_per_milligauss[gain]*2*n_samples;
            high_limit=SELF_TEST_HIGH_LIMIT*counts_per_milligauss[gain]*2*n_samples;

            if ((true==bret) && 
                (low_limit <= xyz_total[0]) && (high_limit >= xyz_total[0]) &&
                (low_limit <= xyz_total[1]) && (high_limit >= xyz_total[1]) &&
                (low_limit <= xyz_total[2]) && (high_limit >= xyz_total[2]) )
            {   /*
                    Successful calibration.
                    Normalize the scale factors so all axis return the same range of values for the bias field.
                    Factor of 2 is from summation of total of n_samples from both positive and negative bias.
                */
                x_scale=(counts_per_milligauss[gain]*(HMC58X3_X_SELF_TEST_GAUSS*2))/(xyz_total[0]/n_samples);
                y_scale=(counts_per_milligauss[gain]*(HMC58X3_Y_SELF_TEST_GAUSS*2))/(xyz_total[1]/n_samples);
                z_scale=(counts_per_milligauss[gain]*(HMC58X3_Z_SELF_TEST_GAUSS*2))/(xyz_total[2]/n_samples);
            }else
            {
                DEBUG_PRINT("HMC58x3 Self test out of range.");
                bret=false;
            }
            I2Cdev::writeByte(devAddr,HMC5883L_RA_CONFIG_A, 0x010); // set RegA/DOR back to default.
        }else
        {
            DEBUG_PRINT("HMC5883L failed id check.");
            bret=false;
        }
    }else
    {   /*
            Bad input parameters.
        */
        DEBUG_PRINT("HMC5883 Bad parameters.");
        bret=false;
    }
    return(bret);
}   //  calibrate().

void HMC5883LCalibratable::getValues(int *x,int *y,int *z) {
  float fx,fy,fz;
  getValues(&fx,&fy,&fz);
  *x= (int) (fx + 0.5);
  *y= (int) (fy + 0.5);
  *z= (int) (fz + 0.5);
}

void HMC5883LCalibratable::getValues(float *x,float *y,float *z) {
  int xr,yr,zr;
  
	getHeading(&xr,&yr,&zr);
  *x= ((float) xr) / x_scale;
  *y = ((float) yr) / y_scale;
  *z = ((float) zr) / z_scale;
}

float HMC5883LCalibratable::compassHeadingRadians() {
	float xr, yr, zr;
	getValues(&xr,&yr,&zr);
	float heading_radians = atan2(yr, xr);
	// Correct for when signs are reversed.
	if(heading_radians < 0)
		heading_radians += 2*PI;
	return heading_radians;
}

float HMC5883LCalibratable::compassHeadingTiltCompensatedRadians(float pitch_radians, float roll_radians) {

  float tilt_compensated_heading;
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll_radians);
  sin_roll = sin(roll_radians);
  cos_pitch = cos(pitch_radians);
  sin_pitch = sin(pitch_radians);

	float xr, yr, zr;
	getValues(&xr,&yr,&zr);
		
  /*
 // Tilt compensated Magnetic field X:
  MAG_X = xr*cos_pitch+yr*sin_roll*sin_pitch+zr*cos_roll*sin_pitch;
  // Tilt compensated Magnetic field Y:
  MAG_Y = yr*cos_roll-zr*sin_roll;
  // Magnetic Heading
  tilt_compensated_heading = atan2(MAG_Y,MAG_X);  // TODO:  Review - why negative Y?
*/
	MAG_X = xr * cos_pitch + zr * sin_pitch;
	MAG_Y = xr * sin_roll * sin_pitch + yr * cos_roll - zr * sin_roll * cos_pitch;
	tilt_compensated_heading = atan2(MAG_Y,MAG_X);
	
  return tilt_compensated_heading;
}
