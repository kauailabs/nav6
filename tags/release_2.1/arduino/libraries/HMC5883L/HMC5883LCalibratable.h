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

#ifndef _HMC5883L_CALIBRATABLE_H_
#define _HMC5883L_CALIBRATABLE_H_

#include "HMC5883L.h"

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (HMC58X3_X_SELF_TEST_GAUSS)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.

#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.

class HMC5883LCalibratable : public HMC5883L{

public:

	HMC5883LCalibratable();
  
	void calibrate(unsigned char gain);
	bool calibrate(unsigned char gain, unsigned int n_samples); 

	void getValues(int *x,int *y,int *z);
	void getValues(float *x,float *y,float *z);

	float compassHeadingRadians();
	float compassHeadingTiltCompensatedRadians(float pitch_radians, float roll_radians);
	
private:

  float x_scale;
  float y_scale;
  float z_scale;
  float x_max;
  float y_max;
  float z_max;
	
};

#endif /* _HMC5883L_CALIBRATABLE_H_ */
