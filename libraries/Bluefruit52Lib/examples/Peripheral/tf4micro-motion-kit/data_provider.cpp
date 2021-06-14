/* Copyright 2021 Google LLC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

/* 
 *  Namespaced methods for providing IMU data
 *  @author Rikard Lindstrom <rlindsrom@google.com>
*/
#include "data_provider.h"

#if defined(ARDUINO_NRF52840_CLUE) || defined(ARDUINO_NRF52840_FEATHER_SENSE)

#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>

Adafruit_LSM6DS33 IMU;     // Gyro and Accel
Adafruit_LIS3MDL  IMUMag;  // Magnetometer

#if defined ARDUINO_NRF52840_FEATHER_SENSE
  // normalize sensor orientation to match Arduino nano
  void normalize_orientation(float& x, float& y, float& z)
  {
    (void) x;
    (void) z;

    y = -y;
  }

#elif defined ARDUINO_NRF52840_CLUE
  // normalize sensor orientation to match Arduino nano
  void normalize_orientation(float& x, float& y, float& z)
  {
    float temp = x;

    x = -y;
    y = temp; // x
    z = -z;
  }
#endif

#else // For custom boards, please include your own sensor

#error "Sensor driver library for your is not included"

// normalize sensor orientation to match Arduino nano
void normalize_orientation(float& x, float& y, float& z)
{
  (void) x; (void) y; (void) z;
}

#endif

namespace data_provider
{
#define DATA_PROVIDER_CALIBRATION_THRESHOLD 10.0
#define DATA_PROVIDER_CALIBRATION_STEPS 40

  /************************************************************************
  * Calibration vars
  ************************************************************************/

  float lastMagneticFieldReading[3] = {0.0, 0.0, 0.0};
  float calibratedMagneticFieldHeading[3] = {0.0, 0.0, 0.0};
  int calibrationStep = 0;
  bool calibrating = false;

  /************************************************************************
  * "Public" functions
  ************************************************************************/

  // Calibrate the magnetometer
  void calibrate()
  {
    calibrating = true;
    calibrationStep = 0;
  }

  bool dataAvailable()
  {
    // Skip magnetometer since it's running a lot slower and always wanted
    return IMU.accelerationAvailable() && IMU.gyroscopeAvailable();
  }

  bool setup()
  {

    if ( !(IMU.begin_I2C() && IMUMag.begin_I2C()) )
    {
      Serial.println("Failed to initialized IMU!");
      return false;
    }

    // Experimental, enabling this will capture all readings
    // from the IMU sensors and should be more accurate. However,
    // it slows down the main loop by a lot when enabled.
    
    // IMU.setContinuousMode();

    Serial.println("IMU sample rates: ");
    Serial.print("Accelerometer sample rate = ");
    Serial.println(IMU.accelerationSampleRate());
    Serial.print("Gyroscope sample rate = ");
    Serial.println(IMU.gyroscopeSampleRate());

    Serial.print("Magnetometer sample rate = ");
    Serial.println(IMUMag.magneticFieldSampleRate());

    return true;
  }

  void update(float *buffer, bool useMagnetometer)
  {

    if (!dataAvailable())
    {
      return;
    }

    float ax, ay, az, gx, gy, gz;

    // read the acceleration and gyroscope data
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // normalize sensor orientation to match Arduino nano
    normalize_orientation(ax, ay, az);
    normalize_orientation(gx, gy, gz);

    // Accelorameter has a range of -4 – 4
    buffer[0] = ax / 4.0;
    buffer[1] = ay / 4.0;
    buffer[2] = az / 4.0;

    // Gyroscope has a range of -2000 – 2000
    buffer[3] = gx / 2000.0;
    buffer[4] = gy / 2000.0;
    buffer[5] = gz / 2000.0;

    if (useMagnetometer || calibrating)
    {
      float mx, my, mz;

      // The Magnetometer sample rate is only 20hz, so we'll use previous values
      // if no new ones are available
      if (IMUMag.magneticFieldAvailable())
      {
        IMUMag.readMagneticField(mx, my, mz);

        // normalize sensor orientation to match Arduino nano
        normalize_orientation(mx, my, mz);

        lastMagneticFieldReading[0] = mx;
        lastMagneticFieldReading[1] = my;
        lastMagneticFieldReading[2] = mz;

        if (calibrating)
        {
          // Running avarage
          calibratedMagneticFieldHeading[0] += mx;
          calibratedMagneticFieldHeading[1] += my;
          calibratedMagneticFieldHeading[2] += mz;
          calibratedMagneticFieldHeading[0] /= 2.0;
          calibratedMagneticFieldHeading[1] /= 2.0;
          calibratedMagneticFieldHeading[2] /= 2.0;
          calibrationStep++;
          if (calibrationStep > DATA_PROVIDER_CALIBRATION_STEPS &&
            abs(calibratedMagneticFieldHeading[0] - mx) < DATA_PROVIDER_CALIBRATION_THRESHOLD &&
            abs(calibratedMagneticFieldHeading[1] - my) < DATA_PROVIDER_CALIBRATION_THRESHOLD &&
            abs(calibratedMagneticFieldHeading[2] - mz) < DATA_PROVIDER_CALIBRATION_THRESHOLD)
          {
            calibrating = false;
            data_provider_calibrationComplete();
            return;
          }
        }
      }
      else
      {
        mx = lastMagneticFieldReading[0];
        my = lastMagneticFieldReading[1];
        mz = lastMagneticFieldReading[2];
      }

      if (calibrating)
      {
        return;
      }

      mx -= calibratedMagneticFieldHeading[0];
      my -= calibratedMagneticFieldHeading[1];
      mz -= calibratedMagneticFieldHeading[2];

      // Raw magnetometer data
      buffer[6] = mx / 50.0;
      buffer[7] = my / 50.0;
      buffer[8] = mz / 50.0;
    }
  }
}
