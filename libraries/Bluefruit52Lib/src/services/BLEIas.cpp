//
// Created by Charlie Bershatsky on 4/19/23.
//

#include "BLEIas.h"
#include "bluefruit.h"
#include "utility/utilities.h"
#include "BLEService.h"

BLEIas::BLEIas(void) :
BLEService(UUID16_SVC_IMMEDIATE_ALERT), _alert(UUID16_SVC_IMMEDIATE_ALERT) {

}

void BLEIas::write(uint8_t alert_level) {
    _alert.write8(alert_level);
}

err_t BLEIas::begin(void) {

    // Invoke the superclass begin()
    VERIFY_STATUS(BLEService::begin());

    _alert.setProperties(CHR_PROPS_READ);
    _alert.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    _alert.setFixedLen(1);

    VERIFY_STATUS( _alert.begin() );

    return ERROR_NONE;

}