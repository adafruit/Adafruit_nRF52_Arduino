/**************************************************************************/
/*!
    @file     bluefruit_common.h
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef BLUEFRUIT_COMMON_H_
#define BLUEFRUIT_COMMON_H_

#include <Arduino.h>
#include "ble.h"
#include "nrf_sdm.h"

#include "utility/AdaMsg.h"

#define CFG_MAX_DEVNAME_LEN                     32

#define BLE_GENERIC_TIMEOUT                     100

#define BLE_GAP_CONN_SUPERVISION_TIMEOUT_MS     2000
#define BLE_GAP_CONN_SLAVE_LATENCY              0

#define BLE_GAP_CONN_MIN_INTERVAL_DFLT          MS100TO125(20)
#define BLE_GAP_CONN_MAX_INTERVAL_DFLT          MS100TO125(30)

// Converts an integer of 1.25ms units to msecs
#define MS100TO125(ms100) (((ms100)*4)/5)

// Converts an integer of 1.25ms units to msecs
#define MS125TO100(ms125) (((ms125)*5)/4)

// Converts msec to 0.625 unit
#define MS1000TO625(ms1000) (((ms1000)*8)/5)

// Converts an integer of 625ms units to msecs
#define MS625TO1000(u625) ( ((u625)*5) / 8 )


typedef void (*ble_connect_callback_t    ) (uint16_t conn_hdl);
typedef void (*ble_disconnect_callback_t ) (uint16_t conn_hdl, uint8_t reason);

enum SecureMode_t
{
  SECMODE_NO_ACCESS          = 0x00,
  SECMODE_OPEN               = 0x11,
  SECMODE_ENC_NO_MITM        = 0x21,
  SECMODE_ENC_WITH_MITM      = 0x31,
  SECMODE_ENC_WITH_LESC_MITM = 0x41, // LESC MITM with 128-bit key
  SECMODE_SIGNED_NO_MITM     = 0x12,
  SECMODE_SIGNED_WITH_MITM   = 0x22
};

#define BLE_SECMODE_NO_ACCESS        ((ble_gap_conn_sec_mode_t) { .sm = 0, .lv = 0 })
#define BLE_SECMODE_OPEN             ((ble_gap_conn_sec_mode_t) { .sm = 1, .lv = 1 })
#define BLE_SECMODE_ENC_NO_MITM      ((ble_gap_conn_sec_mode_t) { .sm = 1, .lv = 2 })
#define BLE_SECMODE_ENC_WITH_MITM    ((ble_gap_conn_sec_mode_t) { .sm = 1, .lv = 3 })
#define BLE_SECMODE_SIGNED_NO_MITM   ((ble_gap_conn_sec_mode_t) { .sm = 2, .lv = 1 })
#define BLE_SECMODE_SIGNED_WITH_MITM ((ble_gap_conn_sec_mode_t) { .sm = 2, .lv = 2 })


#endif /* BLUEFRUIT_COMMON_H_ */
