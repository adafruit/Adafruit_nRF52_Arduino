/**************************************************************************/
/*!
    @file     BLEUuid.h
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2016, Adafruit Industries (adafruit.com)
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
#ifndef BLEUUID_H_
#define BLEUUID_H_

#include "utility/common_inc.h"

class BLEUuid
{
  public:
    ble_uuid_t _uuid;
    uint8_t const* _uuid128;
    
    // Constructors
    BLEUuid(void                      ) { _uuid.type = BLE_UUID_TYPE_UNKNOWN; _uuid.uuid = 0; }
    BLEUuid(uint16_t uuid16           ) { set(uuid16); }
    BLEUuid(uint8_t const uuid128[16] ) { set(uuid128); }

    void set(uint16_t uuid16)
    {
      _uuid.type = BLE_UUID_TYPE_BLE;
      _uuid.uuid = uuid16;
    }

    void set(uint8_t const uuid128[16])
    {
      _uuid.type = BLE_UUID_TYPE_UNKNOWN; _uuid.uuid = 0;
      _uuid128 = uuid128;
    }

    // Add UUID128 if any, in case of UUID16, no ations is required
    // Application should call it anyway to be safe and consistent
    void add(void)
    {
      /* Add base uuid and decode to get uuid16
       * This should cover the already added base uuid128 previously
       */
      if (_uuid.type == BLE_UUID_TYPE_UNKNOWN && _uuid128 != NULL )
      {
        (void) sd_ble_uuid_vs_add( (ble_uuid128_t const*) _uuid128, &_uuid.type );
        (void) sd_ble_uuid_decode(16, _uuid128, &_uuid);
      }
    }

    bool operator==(const BLEUuid&   uuid) const
    {
      return (this->_uuid.type == uuid._uuid.type) && (this->_uuid.uuid == uuid._uuid.uuid);
    }

    bool operator!=(const BLEUuid&   uuid) const
    {
      return !(*this == uuid);
    }

    bool operator==(const ble_uuid_t uuid) const
    {
      return (this->_uuid.type == uuid.type) && (this->_uuid.uuid == uuid.uuid);
    }

    bool operator!=(const ble_uuid_t uuid) const
    {
      return !(*this == uuid);
    }
};

// Service UUID
#define UUID16_SVC_ALERT_NOTIFICATION                         0x1811
#define UUID16_SVC_BATTERY                                    0x180F
#define UUID16_SVC_BLOOD_PRESSURE                             0x1810
#define UUID16_SVC_CURRENT_TIME                               0x1805
#define UUID16_SVC_CYCLING_SPEED_AND_CADENCE                  0x1816
#define UUID16_SVC_LOCATION_AND_NAVIGATION                    0x1819
#define UUID16_SVC_DEVICE_INFORMATION                         0x180A
#define UUID16_SVC_GLUCOSE                                    0x1808
#define UUID16_SVC_HEALTH_THERMOMETER                         0x1809
#define UUID16_SVC_HEART_RATE                                 0x180D
#define UUID16_SVC_HUMAN_INTERFACE_DEVICE                     0x1812
#define UUID16_SVC_IMMEDIATE_ALERT                            0x1802
#define UUID16_SVC_LINK_LOSS                                  0x1803
#define UUID16_SVC_NEXT_DST_CHANGE                            0x1807
#define UUID16_SVC_PHONE_ALERT_STATUS                         0x180E
#define UUID16_SVC_REFERENCE_TIME_UPDATE                      0x1806
#define UUID16_SVC_RUNNING_SPEED_AND_CADENCE                  0x1814
#define UUID16_SVC_SCAN_PARAMETERS                            0x1813
#define UUID16_SVC_TX_POWER                                   0x1804
#define UUID16_SVC_IPSP                                       0x1820
#define UUID16_SVC_BMS                                        0x181E
#define UUID16_SVC_CGM                                        0x181F
#define UUID16_SVC_PLX                                        0x1822

// Characteristic UUID
#define UUID16_CHR_REMOVABLE                                  0x2A3A
#define UUID16_CHR_SERVICE_REQUIRED                           0x2A3B
#define UUID16_CHR_ALERT_CATEGORY_ID                          0x2A43
#define UUID16_CHR_ALERT_CATEGORY_ID_BIT_MASK                 0x2A42
#define UUID16_CHR_ALERT_LEVEL                                0x2A06
#define UUID16_CHR_ALERT_NOTIFICATION_CONTROL_POINT           0x2A44
#define UUID16_CHR_ALERT_STATUS                               0x2A3F
#define UUID16_CHR_BATTERY_LEVEL                              0x2A19
#define UUID16_CHR_BLOOD_PRESSURE_FEATURE                     0x2A49
#define UUID16_CHR_BLOOD_PRESSURE_MEASUREMENT                 0x2A35
#define UUID16_CHR_BODY_SENSOR_LOCATION                       0x2A38
#define UUID16_CHR_BOOT_KEYBOARD_INPUT_REPORT                 0x2A22
#define UUID16_CHR_BOOT_KEYBOARD_OUTPUT_REPORT                0x2A32
#define UUID16_CHR_BOOT_MOUSE_INPUT_REPORT                    0x2A33
#define UUID16_CHR_CURRENT_TIME                               0x2A2B
#define UUID16_CHR_DATE_TIME                                  0x2A08
#define UUID16_CHR_DAY_DATE_TIME                              0x2A0A
#define UUID16_CHR_DAY_OF_WEEK                                0x2A09
#define UUID16_CHR_DST_OFFSET                                 0x2A0D
#define UUID16_CHR_EXACT_TIME_256                             0x2A0C
#define UUID16_CHR_FIRMWARE_REVISION_STRING                   0x2A26
#define UUID16_CHR_GLUCOSE_FEATURE                            0x2A51
#define UUID16_CHR_GLUCOSE_MEASUREMENT                        0x2A18
#define UUID16_CHR_GLUCOSE_MEASUREMENT_CONTEXT                0x2A34
#define UUID16_CHR_HARDWARE_REVISION_STRING                   0x2A27
#define UUID16_CHR_HEART_RATE_CONTROL_POINT                   0x2A39
#define UUID16_CHR_HEART_RATE_MEASUREMENT                     0x2A37
#define UUID16_CHR_HID_CONTROL_POINT                          0x2A4C
#define UUID16_CHR_HID_INFORMATION                            0x2A4A
#define UUID16_CHR_IEEE_REGULATORY_CERTIFICATION_DATA_LIST    0x2A2A
#define UUID16_CHR_INTERMEDIATE_CUFF_PRESSURE                 0x2A36
#define UUID16_CHR_INTERMEDIATE_TEMPERATURE                   0x2A1E
#define UUID16_CHR_LOCAL_TIME_INFORMATION                     0x2A0F
#define UUID16_CHR_MANUFACTURER_NAME_STRING                   0x2A29
#define UUID16_CHR_MEASUREMENT_INTERVAL                       0x2A21
#define UUID16_CHR_MODEL_NUMBER_STRING                        0x2A24
#define UUID16_CHR_UNREAD_ALERT                               0x2A45
#define UUID16_CHR_NEW_ALERT                                  0x2A46
#define UUID16_CHR_PNP_ID                                     0x2A50
#define UUID16_CHR_PROTOCOL_MODE                              0x2A4E
#define UUID16_CHR_RECORD_ACCESS_CONTROL_POINT                0x2A52
#define UUID16_CHR_REFERENCE_TIME_INFORMATION                 0x2A14
#define UUID16_CHR_REPORT                                     0x2A4D
#define UUID16_CHR_REPORT_MAP                                 0x2A4B
#define UUID16_CHR_RINGER_CONTROL_POINT                       0x2A40
#define UUID16_CHR_RINGER_SETTING                             0x2A41
#define UUID16_CHR_SCAN_INTERVAL_WINDOW                       0x2A4F
#define UUID16_CHR_SCAN_REFRESH                               0x2A31
#define UUID16_CHR_SERIAL_NUMBER_STRING                       0x2A25
#define UUID16_CHR_SOFTWARE_REVISION_STRING                   0x2A28
#define UUID16_CHR_SUPPORTED_NEW_ALERT_CATEGORY               0x2A47
#define UUID16_CHR_SUPPORTED_UNREAD_ALERT_CATEGORY            0x2A48
#define UUID16_CHR_SYSTEM_ID                                  0x2A23
#define UUID16_CHR_TEMPERATURE_MEASUREMENT                    0x2A1C
#define UUID16_CHR_TEMPERATURE_TYPE                           0x2A1D
#define UUID16_CHR_TIME_ACCURACY                              0x2A12
#define UUID16_CHR_TIME_SOURCE                                0x2A13
#define UUID16_CHR_TIME_UPDATE_CONTROL_POINT                  0x2A16
#define UUID16_CHR_TIME_UPDATE_STATE                          0x2A17
#define UUID16_CHR_TIME_WITH_DST                              0x2A11
#define UUID16_CHR_TIME_ZONE                                  0x2A0E
#define UUID16_CHR_TX_POWER_LEVEL                             0x2A07
#define UUID16_CHR_CSC_FEATURE                                0x2A5C
#define UUID16_CHR_CSC_MEASUREMENT                            0x2A5B
#define UUID16_CHR_RSC_FEATURE                                0x2A54
#define UUID16_CHR_SC_CTRLPT                                  0x2A55
#define UUID16_CHR_RSC_MEASUREMENT                            0x2A53
#define UUID16_CHR_SENSOR_LOCATION                            0x2A5D
#define UUID16_EXTERNAL_REPORT_REF_DESCR                      0x2907
#define UUID16_REPORT_REF_DESCR                               0x2908
#define UUID16_CHR_LN_FEATURE                                 0x2A6A
#define UUID16_CHR_LN_POSITION_QUALITY                        0x2A69
#define UUID16_CHR_LN_LOCATION_AND_SPEED                      0x2A67
#define UUID16_CHR_LN_NAVIGATION                              0x2A68
#define UUID16_CHR_LN_CONTROL_POINT                           0x2A6B
#define UUID16_BMS_CTRLPT                                     0x2AA4
#define UUID16_BMS_FEATURE                                    0x2AA5
#define UUID16_CGM_MEASUREMENT                                0x2AA7
#define UUID16_CGM_FEATURE                                    0x2AA8
#define UUID16_CGM_STATUS                                     0x2AA9
#define UUID16_CGM_SESSION_START_TIME                         0x2AAA
#define UUID16_CGM_SESSION_RUN_TIME                           0x2AAB
#define UUID16_CGM_SPECIFIC_OPS_CTRLPT                        0x2AAC
#define UUID16_PLX_SPOT_CHECK_MEAS                            0x2A5E
#define UUID16_PLX_CONTINUOUS_MEAS                            0x2A5F
#define UUID16_PLX_FEATURES                                   0x2A60


#endif /* BLEUUID_H_ */
