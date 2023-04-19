//
// Created by Charlie Bershatsky on 4/19/23.
//

#include "BLEIas.h"
#include "bluefruit.h"
#include "utility/utilities.h"
#include "BLEService.h"

BLEIas::BLEIas(void) : BLEService(UUID16_SVC_IMMEDIATE_ALERT) {

}

void BLEIas::setAlertLevel(uint8_t alert_level) {
    _alert_level = alert_level;
}

err_t BLEIas::begin(void) {

    // Invoke the superclass begin()
    VERIFY_STATUS(BLEService::begin());

    BLECharacteristic chars;

    chars.setUuid(UUID16_CHR_ALERT_LEVEL);
    chars.setTempMemory();
    chars.setProperties(CHR_PROPS_READ);
    chars.setFixedLen(sizeof(_alert_level));
    VERIFY_STATUS(chars.begin());
    chars.write8(_alert_level);

    return ERROR_NONE;

}