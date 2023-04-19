//
// Created by Charlie Bershatsky on 4/19/23.
//

#ifndef ADAFRUIT_NRF52_ARDUINO_BLEIAS_H
#define ADAFRUIT_NRF52_ARDUINO_BLEIAS_H

#include "bluefruit_common.h"

#include "BLEService.h"
#include "BLECharacteristic.h"


class BLEIas : public BLEService {

protected:
    union {
        struct {
            const uint8_t _alert_level; // UUID 0x2A06
        };
    };

public:
    BLEIas(void);

    void setAlertLevel(uint8_t alert_level);

    virtual err_t begin(void);

};


#endif //ADAFRUIT_NRF52_ARDUINO_BLEIAS_H
