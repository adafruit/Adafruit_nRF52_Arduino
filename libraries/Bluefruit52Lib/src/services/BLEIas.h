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
    BLECharacteristic _alert;

public:
    BLEIas(void);

    void write(uint8_t level);

    virtual err_t begin(void);

};


#endif //ADAFRUIT_NRF52_ARDUINO_BLEIAS_H
