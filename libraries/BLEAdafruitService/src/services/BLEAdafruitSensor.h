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

#ifndef BLEADAFRUIT_SENSOR_H_
#define BLEADAFRUIT_SENSOR_H_

#include <Adafruit_Sensor.h>

class BLEAdafruitSensor : public BLEService
{
  public:
    static const int32_t DEFAULT_PERIOD = 1000;
    typedef uint16_t (*measure_callback_t )(uint8_t* buf, uint16_t bufsize);

    BLEAdafruitSensor(BLEUuid service_uuid, BLEUuid data_uuid);
    virtual err_t begin(int32_t ms);

    virtual err_t begin(measure_callback_t fp, int32_t ms = DEFAULT_PERIOD);
    virtual err_t begin(Adafruit_Sensor* sensor, int32_t ms = DEFAULT_PERIOD);

    void setMeasureCallback(measure_callback_t fp);
    void setPeriod(int32_t period_ms);

  protected:
    BLECharacteristic _period;
    BLECharacteristic _measurement;

    Adafruit_Sensor*  _sensor;

    measure_callback_t _measure_cb;
    SoftwareTimer _timer;

    void _update_timer(int32_t ms);
    void _timer_callback(void);

    static void sensor_timer_cb(TimerHandle_t xTimer);
    static void sensor_period_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
    static void sensor_data_cccd_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value);
};

#endif /* BLEADAFRUIT_SENSOR_H_ */
