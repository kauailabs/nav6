
#define EMPL_TARGET_ATMEGA328
#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883LCalibratable.h"
extern "C" {
  #include "inv_mpu.h"
  #include "inv_mpu_dmp_motion_driver.h"
}
#include <avr/wdt.h>
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
};

static struct hal_s hal = {0};

boolean mag_data_ready = false;
void magDataAvailable() {
  mag_data_ready = true;
}

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)
/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

HMC5883LCalibratable mag;

#define ERROR_MPU6050_NOT_RESPONDING   1
#define ERROR_HMC5883L_NOT_RESPONDING   2
#define ERROR_MPU6050_INIT_FAILED       3
#define ERROR_MPU6050_GYROTEST_FAILED   4
#define ERROR_MPU6050_ACCELTEST_FAILED  5
#define ERROR_HMC5883L_COMMTEST_FAILED  6
#define ERROR_HMC5883L_CALIB_FAILED     7
#define ERROR_MPU6050_INT_FAILED        8
#define ERROR_HMC5883L_INT_FAILED       9

int error_code = 0;
unsigned long start_ms = 0;
bool test_complete = false;

void flashStatus( int num_flashes, int delay_ms )
{
  for ( int i = 0; i < num_flashes; i++ ) {
    digitalWrite(13,HIGH);
    delay(delay_ms);
    digitalWrite(13,LOW);
    delay(delay_ms);
  }
}
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#define SDA_PIN A4
#define SCL_PIN A5

void setup() {

    // reset i2c bus
    pinMode(SDA_PIN,INPUT);
    pinMode(SCL_PIN,OUTPUT);
    
    // Clock through up to 100 bits
    for ( int i = 0; i < 100; i++ ) 
    {
      digitalWrite(SCL_PIN,HIGH);
      digitalWrite(SCL_PIN,LOW);
      digitalWrite(SCL_PIN,HIGH);
    }
    // send a stop
    digitalWrite(SDA_PIN,HIGH);
    digitalWrite(SDA_PIN,LOW);
/*    
    digitalWrite(SDA_PIN,HIGH);
    byte curr_sda = digitalRead(SDA_PIN);
    bool stuck = (curr_sda == LOW);
    bool was_stuck = false;
    while ( stuck )
    {
      was_stuck = true;
      byte curr_sda = digitalRead(SDA_PIN);
      bool stuck = (curr_sda == LOW);
    }
    digitalWrite(SCL_PIN,HIGH);
    digitalWrite(SDA_PIN,LOW);    
*/    
    // initialize serial communication
    Serial.begin(57600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately  
    /*if ( was_stuck )
    {
      Serial.println("Was Stuck!");
    }*/

    Serial.println("Kauai Labs nav6 Factory Test");
    Serial.flush();
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    // Disable internal I2C pull-ups
    cbi(PORTC, 4);
    cbi(PORTC, 5);

    pinMode(13,OUTPUT);
    // Quick flashes to indicate test is starting
    flashStatus(3,75);
  
    Serial.println ("Scanning I2C Devices...");
    byte count = 0;

    // Scan I2C bus.  Two I2C devices are at 0x1E (HMC5883L) and 0x68 (MPU-6050).
  
    boolean found_mpu6050 = false;
    boolean found_hmc5883L = false;
    // Scan i2c bus for device addresses 8-119 (other addresses are reserved)
    for (byte i = 8; i < 120; i++)
    {
      Wire.beginTransmission (i);
      if (Wire.endTransmission () == 0)
      {
        if ( i == 0x1E ) {
          found_hmc5883L = true;
        } else if ( i == 0x68 ) {
          found_mpu6050 = true;
        }
        Serial.print ("Found address: ");
        Serial.print (i, DEC);
        Serial.print (" (0x");
        Serial.print (i, HEX);
        Serial.println (")");      count++;
        delay (1);  // maybe unneeded?
      } // end of good response
    } // end of for loop
  
    Serial.print ("Found ");
    Serial.print (count, DEC);
    Serial.println (" device(s).");
    if ( !found_mpu6050 ) {
      error_code = ERROR_MPU6050_NOT_RESPONDING;
      return;
    } else if ( !found_hmc5883L ) {
      error_code = ERROR_HMC5883L_NOT_RESPONDING;
      return;
    }
    delay(100);
  
    Serial.print("MPU-6050 Init:  ");
    
    if ( initialize_mpu() != 0 ) {
      Serial.println("Failed");
      Serial.flush();
      error_code = ERROR_MPU6050_INIT_FAILED;
      return;
    }
    else {
      Serial.println("Succeeded");
      Serial.flush();
      delay(50);
      boolean gyro_ok, accel_ok;
      run_mpu_self_test(gyro_ok,accel_ok);
  
      Serial.print("Gyro Self Test:  ");
      if ( gyro_ok ) {
        Serial.println("Passed");
      } else {
        Serial.println("Failed");
        error_code = ERROR_MPU6050_GYROTEST_FAILED;
        return;
      }
      Serial.print("Accel Self Test:  ");
      if ( accel_ok ) {
        Serial.println("Passed");
      } else {
        Serial.println("Failed");
        error_code = ERROR_MPU6050_ACCELTEST_FAILED;
        return;
      }
    }
    
    mag.initialize();

    // verify connection
    Serial.println("Testing magnetometer comm.");
    Serial.print( "HMC5883L connect ");
    Serial.flush();
    delay(10);
    if ( mag.testConnection() ) {
      Serial.println("Passed");
    } else {
      Serial.println("Failed");
      error_code = ERROR_HMC5883L_COMMTEST_FAILED;
      return;
    }

    uint8_t gain = mag.getGain();    
    Serial.print("Default Gain:  ");
    Serial.println(gain);
    Serial.println("Calibrating via Magnetometer Self-Test...   ");
    Serial.flush();
    delay(10);
    if ( mag.calibrate(gain,100) ) {
      Serial.println("Success");
    }
    else {
      error_code =  ERROR_HMC5883L_CALIB_FAILED;
      Serial.println("Failed");
      return;
    }
    
    attachInterrupt(1,magDataAvailable,RISING);
    // Initiate reading from magnetometer, this should trigger an interrupt.
    mag.getHeadingX();
    start_ms = millis();
}

boolean mpu6050_int_received = false;
boolean mag_int_received = false;
#define TEST_DURATION_MS  1000

void loop() {
  
  if ( !test_complete && ( ( millis() - start_ms ) > (unsigned long)TEST_DURATION_MS ) )
  {
    test_complete = true;
    if ( (error_code == 0) && !mpu6050_int_received ) {
      error_code = ERROR_MPU6050_INT_FAILED;
    }
    if ( (error_code == 0) && !mag_int_received ) {
      error_code = ERROR_HMC5883L_INT_FAILED;
    }
  }
  
  if (hal.new_gyro && !mpu6050_int_received) {
      Serial.println("Received MPU6050 Interrupt.");
      mpu6050_int_received = true;
  }
  if ( mag_data_ready && !mag_int_received ) {
      Serial.println("Received HMC5883L Interrupt.");
      mag_int_received = true;
  }
  delay(100);
  if ( test_complete )
  {
    while ( true ) {
      if ( error_code == 0 ) {
        // Steady ON
        digitalWrite(13,HIGH);
      }
      else {
        // Flash the error code
          flashStatus(error_code,300);
      }
      delay(2500);
    }
  }
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}

boolean initialize_mpu() {
    int result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    struct int_param_s int_param;

    /* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    int_param.cb = gyro_data_ready_cb;
    int_param.pin = 0;
    result = mpu_init(&int_param);

    if ( result != 0 ) {
      Serial.print("mpu_init failed!");
      Serial.flush();
      return false;
    }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    result = dmp_load_motion_driver_firmware();
    if ( result != 0 ) {
      Serial.print("Firmware Load ERROR ");
      Serial.println(result);
      Serial.flush();
      return false;
    }
    
    unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | 
        DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    return true;
}

void enable_mpu() {
    mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
    hal.dmp_on = 1;
}  

boolean run_mpu_self_test(boolean& gyro_ok, boolean& accel_ok) {
  
    int result;
    long gyro[3], accel[3];
    boolean success = false;

    gyro_ok = false;
    accel_ok = false;
    result = mpu_run_self_test(gyro, accel);
    if ( ( result & 0x1 ) != 0 ) {
      // Gyro passed self test
      gyro_ok = true;
      float sens;
      mpu_get_gyro_sens(&sens);
      gyro[0] = (long)(gyro[0] * sens);
      gyro[1] = (long)(gyro[1] * sens);
      gyro[2] = (long)(gyro[2] * sens);
      dmp_set_gyro_bias(gyro);
    }
    if ( ( result & 0x2 ) != 0 ) {
      // Accelerometer passed self test
      accel_ok = true;
      unsigned short accel_sens;
      mpu_get_accel_sens(&accel_sens);
      accel[0] *= accel_sens;
      accel[1] *= accel_sens;
      accel[2] *= accel_sens;
      dmp_set_accel_bias(accel);
    }

    success = gyro_ok && accel_ok;
  
    return success;
}

