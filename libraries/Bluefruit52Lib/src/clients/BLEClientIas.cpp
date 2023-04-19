//
// Created by Charlie Bershatsky on 4/19/23.
//

#include "BLEClientIas.h"
#include "bluefruit.h"

BLEClientIas::BLEClientIas() : BLEClientService(UUID16_SVC_IMMEDIATE_ALERT) {

}

bool BLEClientIas::begin(void) {
    // invoke superclass begin()
    BLEClientService::begin();
    return true;
}

bool BLEClientIas::discover(uint16_t conn_handle) {
    VERIFY(BLEClientService::discover(conn_handle));

    return true;
}

uint16_t BLEClientIas::getAlertLevel() {

    uint8_t level = 0;
    ble_gattc_handle_range_t bck_range = Bluefruit.Discovery.getHandleRange();

    BLEClientCharacteristic chr(uuid);
    chr.begin(this);

    if (Bluefruit.Discovery.discoverCharacteristic(_conn_hdl, chr)) {
        level = chr.read8();
    }

    Bluefruit.Discovery.setHandleRange(bck_range);

    return level;
}