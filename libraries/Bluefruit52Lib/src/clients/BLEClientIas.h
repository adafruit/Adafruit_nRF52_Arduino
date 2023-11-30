//
// Created by Charlie Bershatsky on 4/19/23.
//

#ifndef ADAFRUIT_NRF52_ARDUINO_BLECLIENTIAS_H
#define ADAFRUIT_NRF52_ARDUINO_BLECLIENTIAS_H

#include "bluefruit_common.h"
#include "BLEClientCharacteristic.h"
#include "BLEClientService.h"

class BLEClientIas : public BLEClientService {

public:
    BLEClientIas(void);

    virtual bool begin(void);
    virtual bool discover(uint16_t conn_handle);

    uint16_t getAlertLevel (void);

private:
    BLEClientCharacteristic _alert;

};


#endif //ADAFRUIT_NRF52_ARDUINO_BLECLIENTIAS_H
