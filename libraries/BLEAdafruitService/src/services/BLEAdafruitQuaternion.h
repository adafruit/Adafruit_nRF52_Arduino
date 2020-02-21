/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org) for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef BLEADAFRUIT_QUATERNION_H_
#define BLEADAFRUIT_QUATERNION_H_

#include <Adafruit_Sensor.h>

class Adafruit_AHRS_FusionInterface;

class BLEAdafruitQuaternion : public BLEAdafruitSensor
{
  public:
    static const uint8_t UUID128_SERVICE[16];
    static const uint8_t UUID128_CHR_DATA[16];
    static const uint8_t FILTER_MEASURE_RATIO = 10; // number of filter update for each measure report to client

    BLEAdafruitQuaternion(void);
    err_t begin(Adafruit_AHRS_FusionInterface* filter, Adafruit_Sensor* accel, Adafruit_Sensor* gyro, Adafruit_Sensor* mag);

  protected:
    virtual void _update_timer(int32_t ms);
    virtual void _measure_handler(void);
    virtual void _notify_cb(uint16_t conn_hdl, uint16_t value);

    void _fitler_update(void);

  private:
    Adafruit_Sensor* _accel;
    Adafruit_Sensor* _gyro;
    Adafruit_Sensor* _mag;

    Adafruit_AHRS_FusionInterface* _filter;
    SoftwareTimer _filter_timer;

    static void quaternion_filter_timer_cb(TimerHandle_t xTimer);
};

#endif /* BLEADAFRUIT_QUATERNION_H_ */
