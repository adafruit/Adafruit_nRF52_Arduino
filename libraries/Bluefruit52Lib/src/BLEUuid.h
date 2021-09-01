/**************************************************************************/
/*!
    @file     BLEUuid.h
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
#ifndef BLEUUID_H_
#define BLEUUID_H_

#include "bluefruit_common.h"

class BLEUuid
{
  public:
    ble_uuid_t _uuid;
    uint8_t const* _uuid128;
    const char*    _str;
    
    // Constructors
    BLEUuid(void                      );
    BLEUuid(uint16_t uuid16           );
    BLEUuid(uint8_t const uuid128[16] );
    BLEUuid(const char* str           );
    BLEUuid(ble_uuid_t uuid           );

    virtual ~BLEUuid();

    void set(uint16_t uuid16);
    void set(uint8_t const uuid128[16]);
    void set(const char* str);

    bool get(uint16_t* uuid16) const;
    bool get(uint8_t uuid128[16]);

    size_t size (void) const;

    // Add UUID128 if needed, in case of UUID16, no actions is required
    bool begin(void);

    String toString(void) const;

    bool operator==(const BLEUuid&   uuid) const;
    bool operator!=(const BLEUuid&   uuid) const;
    bool operator==(const ble_uuid_t uuid) const;
    bool operator!=(const ble_uuid_t uuid) const;

    // Overload copy operator to allow initialization from other type
    BLEUuid& operator=(const uint16_t uuid);
    BLEUuid& operator=(uint8_t const uuid128[16]);
    BLEUuid& operator=(ble_uuid_t uuid);
    BLEUuid& operator=(const char* str);
};

//--------------------------------------------------------------------+
// Custom UUID Service
//--------------------------------------------------------------------+
extern const uint8_t UUID128_CHR_ADAFRUIT_MEASUREMENT_PERIOD[16];
extern const uint8_t UUID128_CHR_ADAFRUIT_VERSION[16];

/*------------------------------------------------------------------*/
/* Service UUID
 * https://www.bluetooth.com/specifications/gatt/services
 *------------------------------------------------------------------*/
#define UUID16_SVC_GENERIC_ACCESS                             0x1800
#define UUID16_SVC_GENERIC_ATTRIBUTE                          0x1801
#define UUID16_SVC_IMMEDIATE_ALERT                            0x1802
#define UUID16_SVC_LINK_LOSS                                  0x1803
#define UUID16_SVC_TX_POWER                                   0x1804
#define UUID16_SVC_CURRENT_TIME                               0x1805
#define UUID16_SVC_REFERENCE_TIME_UPDATE                      0x1806
#define UUID16_SVC_NEXT_DST_CHANGE                            0x1807
#define UUID16_SVC_GLUCOSE                                    0x1808
#define UUID16_SVC_HEALTH_THERMOMETER                         0x1809
#define UUID16_SVC_DEVICE_INFORMATION                         0x180A
#define UUID16_SVC_HEART_RATE                                 0x180D
#define UUID16_SVC_PHONE_ALERT_STATUS                         0x180E
#define UUID16_SVC_BATTERY                                    0x180F
#define UUID16_SVC_BLOOD_PRESSURE                             0x1810
#define UUID16_SVC_ALERT_NOTIFICATION                         0x1811
#define UUID16_SVC_HUMAN_INTERFACE_DEVICE                     0x1812
#define UUID16_SVC_SCAN_PARAMETERS                            0x1813
#define UUID16_SVC_RUNNING_SPEED_AND_CADENCE                  0x1814
#define UUID16_SVC_AUTOMATION_IO                              0x1815
#define UUID16_SVC_CYCLING_SPEED_AND_CADENCE                  0x1816
#define UUID16_SVC_CYCLING_POWER                              0x1818
#define UUID16_SVC_LOCATION_AND_NAVIGATION                    0x1819
#define UUID16_SVC_ENVIRONMENTAL_SENSING                      0x181A
#define UUID16_SVC_BODY_COMPOSITION                           0x181B
#define UUID16_SVC_USER_DATA                                  0x181C
#define UUID16_SVC_WEIGHT_SCALE                               0x181D
#define UUID16_SVC_BMS                                        0x181E
#define UUID16_SVC_CGM                                        0x181F
#define UUID16_SVC_IPSP                                       0x1820
#define UUID16_SVC_INDOOR_POSITIONING                         0x1821
#define UUID16_SVC_PLX                                        0x1822
#define UUID16_SVC_HTTP_PROXY                                 0x1823
#define UUID16_SVC_TRANSPORT_DISCOVERY                        0x1824
#define UUID16_SVC_OTS                                        0x1825
#define UUID16_SVC_FITNESS_MACHINE                            0x1826
#define UUID16_SVC_MESH_PROVISIONING                          0x1827
#define UUID16_SVC_MESH_PROXY                                 0x1828
#define UUID16_SVC_RECONNECTION_CONFIGURATION                 0x1829
#define UUID16_SVC_INSULIN_DELIVERY                           0x183A
#define UUID16_SVC_BINARY_SENSOR                              0x183B
#define UUID16_SVC_EMERGENCY_CONFIGURATION                    0x183C
#define UUID16_SVC_PHYSICAL_ACTIVITY_MONITOR                  0x183E
#define UUID16_SVC_AUDIO_INPUT_CONTROL                        0x1843
#define UUID16_SVC_VOLUME_CONTROL                             0x1844
#define UUID16_SVC_VOLUME_OFFSET_CONTROL                      0x1845
#define UUID16_SVC_COORDINATED_SET_IDENTIFICATION_SERVICE     0x1846
#define UUID16_SVC_DEVICE_TIME                                0x1847
#define UUID16_SVC_MEDIA_CONTROL_SERVICE                      0x1848
#define UUID16_SVC_GENERIC_MEDIA_CONTROL_SERVICE              0x1849
#define UUID16_SVC_CONSTANT_TONE_EXTENSION                    0x184A
#define UUID16_SVC_TELEPHONE_BEARER_SERVICE                   0x184B
#define UUID16_SVC_GENERIC_TELEPHONE_BEARER_SERVICE           0x184C
#define UUID16_SVC_MICROPHONE_CONTROL                         0x184D

#define UUID16_SVC_EDDYSTONE                                  0xFEAA

//
#define UUID16_EXTERNAL_REPORT_REF_DESCR                      0x2907
#define UUID16_REPORT_REF_DESCR                               0x2908

/*------------------------------------------------------------------*/
/* Characteristic UUID
 * https://www.bluetooth.com/specifications/gatt/characteristics
 *------------------------------------------------------------------*/
#define UUID16_CHR_DEVICE_NAME                                0x2A00
#define UUID16_CHR_APPEARANCE                                 0x2A01
#define UUID16_CHR_PERIPHERAL_PRIVACY_FLAG                    0x2A02
#define UUID16_CHR_RECONNECTION_ADDRESS                       0x2A03
#define UUID16_CHR_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS 0x2A04
#define UUID16_CHR_SERVICE_CHANGED                            0x2A05
#define UUID16_CHR_ALERT_LEVEL                                0x2A06
#define UUID16_CHR_TX_POWER_LEVEL                             0x2A07
#define UUID16_CHR_DATE_TIME                                  0x2A08
#define UUID16_CHR_DAY_OF_WEEK                                0x2A09
#define UUID16_CHR_DAY_DATE_TIME                              0x2A0A
#define UUID16_CHR_EXACT_TIME_256                             0x2A0C
#define UUID16_CHR_DST_OFFSET                                 0x2A0D
#define UUID16_CHR_TIME_ZONE                                  0x2A0E
#define UUID16_CHR_LOCAL_TIME_INFORMATION                     0x2A0F
#define UUID16_CHR_TIME_WITH_DST                              0x2A11
#define UUID16_CHR_TIME_ACCURACY                              0x2A12
#define UUID16_CHR_TIME_SOURCE                                0x2A13
#define UUID16_CHR_REFERENCE_TIME_INFORMATION                 0x2A14
#define UUID16_CHR_TIME_UPDATE_CONTROL_POINT                  0x2A16
#define UUID16_CHR_TIME_UPDATE_STATE                          0x2A17
#define UUID16_CHR_GLUCOSE_MEASUREMENT                        0x2A18
#define UUID16_CHR_BATTERY_LEVEL                              0x2A19
#define UUID16_CHR_TEMPERATURE_MEASUREMENT                    0x2A1C
#define UUID16_CHR_TEMPERATURE_TYPE                           0x2A1D
#define UUID16_CHR_INTERMEDIATE_TEMPERATURE                   0x2A1E
#define UUID16_CHR_TEMPERATURE_CELSIUS                        0x2A1F
#define UUID16_CHR_TEMPERATURE_FAHRENHEIT                     0x2A20
#define UUID16_CHR_MEASUREMENT_INTERVAL                       0x2A21
#define UUID16_CHR_BOOT_KEYBOARD_INPUT_REPORT                 0x2A22
#define UUID16_CHR_SYSTEM_ID                                  0x2A23
#define UUID16_CHR_MODEL_NUMBER_STRING                        0x2A24
#define UUID16_CHR_SERIAL_NUMBER_STRING                       0x2A25
#define UUID16_CHR_FIRMWARE_REVISION_STRING                   0x2A26
#define UUID16_CHR_HARDWARE_REVISION_STRING                   0x2A27
#define UUID16_CHR_SOFTWARE_REVISION_STRING                   0x2A28
#define UUID16_CHR_MANUFACTURER_NAME_STRING                   0x2A29
#define UUID16_CHR_IEEE_REGULATORY_CERTIFICATION_DATA_LIST    0x2A2A
#define UUID16_CHR_CURRENT_TIME                               0x2A2B
#define UUID16_CHR_SCAN_REFRESH                               0x2A31
#define UUID16_CHR_BOOT_KEYBOARD_OUTPUT_REPORT                0x2A32
#define UUID16_CHR_BOOT_MOUSE_INPUT_REPORT                    0x2A33
#define UUID16_CHR_GLUCOSE_MEASUREMENT_CONTEXT                0x2A34
#define UUID16_CHR_BLOOD_PRESSURE_MEASUREMENT                 0x2A35
#define UUID16_CHR_INTERMEDIATE_CUFF_PRESSURE                 0x2A36
#define UUID16_CHR_HEART_RATE_MEASUREMENT                     0x2A37
#define UUID16_CHR_BODY_SENSOR_LOCATION                       0x2A38
#define UUID16_CHR_HEART_RATE_CONTROL_POINT                   0x2A39
#define UUID16_CHR_REMOVABLE                                  0x2A3A
#define UUID16_CHR_SERVICE_REQUIRED                           0x2A3B
#define UUID16_CHR_ALERT_STATUS                               0x2A3F
#define UUID16_CHR_RINGER_CONTROL_POINT                       0x2A40
#define UUID16_CHR_RINGER_SETTING                             0x2A41
#define UUID16_CHR_ALERT_CATEGORY_ID_BIT_MASK                 0x2A42
#define UUID16_CHR_ALERT_CATEGORY_ID                          0x2A43
#define UUID16_CHR_ALERT_NOTIFICATION_CONTROL_POINT           0x2A44
#define UUID16_CHR_UNREAD_ALERT                               0x2A45
#define UUID16_CHR_NEW_ALERT                                  0x2A46
#define UUID16_CHR_SUPPORTED_NEW_ALERT_CATEGORY               0x2A47
#define UUID16_CHR_SUPPORTED_UNREAD_ALERT_CATEGORY            0x2A48
#define UUID16_CHR_BLOOD_PRESSURE_FEATURE                     0x2A49
#define UUID16_CHR_HID_INFORMATION                            0x2A4A
#define UUID16_CHR_REPORT_MAP                                 0x2A4B
#define UUID16_CHR_HID_CONTROL_POINT                          0x2A4C
#define UUID16_CHR_REPORT                                     0x2A4D
#define UUID16_CHR_PROTOCOL_MODE                              0x2A4E
#define UUID16_CHR_SCAN_INTERVAL_WINDOW                       0x2A4F
#define UUID16_CHR_PNP_ID                                     0x2A50
#define UUID16_CHR_GLUCOSE_FEATURE                            0x2A51
#define UUID16_CHR_RECORD_ACCESS_CONTROL_POINT                0x2A52
#define UUID16_CHR_RSC_MEASUREMENT                            0x2A53
#define UUID16_CHR_RSC_FEATURE                                0x2A54
#define UUID16_CHR_SC_CTRLPT                                  0x2A55
#define UUID16_CHR_AGGREGATE                                  0x2A5A
#define UUID16_CHR_CSC_MEASUREMENT                            0x2A5B
#define UUID16_CHR_CSC_FEATURE                                0x2A5C
#define UUID16_CHR_SENSOR_LOCATION                            0x2A5D
#define UUID16_CHR_PLX_SPOT_CHECK_MEAS                        0x2A5E
#define UUID16_CHR_PLX_CONTINUOUS_MEAS                        0x2A5F
#define UUID16_CHR_PLX_FEATURES                               0x2A60
#define UUID16_CHR_CYCLING_POWER_MEASUREMENT                  0x2A63
#define UUID16_CHR_CYCLING_POWER_VECTOR                       0x2A64
#define UUID16_CHR_CYCLING_POWER_FEATURE                      0x2A65
#define UUID16_CHR_CYCLING_POWER_CONTROL_POINT                0x2A66
#define UUID16_CHR_LN_LOCATION_AND_SPEED                      0x2A67
#define UUID16_CHR_LN_NAVIGATION                              0x2A68
#define UUID16_CHR_LN_POSITION_QUALITY                        0x2A69
#define UUID16_CHR_LN_FEATURE                                 0x2A6A
#define UUID16_CHR_LN_CONTROL_POINT                           0x2A6B
#define UUID16_CHR_ELEVATION                                  0x2A6C
#define UUID16_CHR_PRESSURE                                   0x2A6D
#define UUID16_CHR_TEMPERATURE                                0x2A6E
#define UUID16_CHR_HUMIDITY                                   0x2A6F
#define UUID16_CHR_TRUE_WIND_SPEED                            0x2A70
#define UUID16_CHR_TRUE_WIND_DIRECTION                        0x2A71
#define UUID16_CHR_APPARENT_WIND_SPEED                        0x2A72
#define UUID16_CHR_APPARENT_WIND_DIRECTION                    0x2A73
#define UUID16_CHR_GUST_FACTOR                                0x2A74
#define UUID16_CHR_POLLEN_CONCENTRATION                       0x2A75
#define UUID16_CHR_UV_INDEX                                   0x2A76
#define UUID16_CHR_IRRADIANCE                                 0x2A77
#define UUID16_CHR_RAINFALL                                   0x2A78
#define UUID16_CHR_WIND_CHILL                                 0x2A79
#define UUID16_CHR_HEAT_INDEX                                 0x2A7A
#define UUID16_CHR_DEW_POINT                                  0x2A7B
#define UUID16_CHR__DESCRIPTOR_VALUE_CHANGED                  0x2A7D
#define UUID16_CHR_AEROBIC_HEART_RATE_LOWER_LIMIT             0x2A7E
#define UUID16_CHR_AEROBIC_THRESHOLD                          0x2A7F
#define UUID16_CHR_AGE                                        0x2A80
#define UUID16_CHR_ANAEROBIC_HEART_RATE_LOWER_LIMIT           0x2A81
#define UUID16_CHR_ANAEROBIC_HEART_RATE_UPPER_LIMIT           0x2A82
#define UUID16_CHR_ANAEROBIC_THRESHOLD                        0x2A83
#define UUID16_CHR_AEROBIC_HEART_RATE_UPPER_LIMIT             0x2A84
#define UUID16_CHR_DATE_OF_BIRTH                              0x2A85
#define UUID16_CHR_DATE_OF_THRESHOLD_ASSESSMENT               0x2A86
#define UUID16_CHR_EMAIL_ADDRESS                              0x2A87
#define UUID16_CHR_FAT_BURN_HEART_RATE_LOWER_LIMIT            0x2A88
#define UUID16_CHR_FAT_BURN_HEART_RATE_UPPER_LIMIT            0x2A89
#define UUID16_CHR_FIRST_NAME                                 0x2A8A
#define UUID16_CHR_FIVE_ZONE_HEART_RATE_LIMITS                0x2A8B
#define UUID16_CHR_GENDER                                     0x2A8C
#define UUID16_CHR_HEART_RATE_MAX                             0x2A8D
#define UUID16_CHR_HEIGHT                                     0x2A8E
#define UUID16_CHR_HIP_CIRCUMFERENCE                          0x2A8F
#define UUID16_CHR_LAST_NAME                                  0x2A90
#define UUID16_CHR_MAXIMUM_RECOMMENDED_HEART_RATE             0x2A91
#define UUID16_CHR_RESTING_HEART_RATE                         0x2A92
#define UUID16_CHR_SPORT_TYPE                                 0x2A93
#define UUID16_CHR_THREE_ZONE_HEART_RATE_LIMITS               0x2A94
#define UUID16_CHR_TWO_ZONE_HEART_RATE_LIMITS                 0x2A95
#define UUID16_CHR_VO2_MAX                                    0x2A96
#define UUID16_CHR_WAIST_CIRCUMFERENCE                        0x2A97
#define UUID16_CHR_WEIGHT                                     0x2A98
#define UUID16_CHR_DATABASE_CHANGE_INCREMENT                  0x2A99
#define UUID16_CHR_USER_INDEX                                 0x2A9A
#define UUID16_CHR_BODY_COMPOSITION_FEATURE                   0x2A9B
#define UUID16_CHR_BODY_COMPOSITION_MEASUREMENT               0x2A9C
#define UUID16_CHR_WEIGHT_MEASUREMENT                         0x2A9D
#define UUID16_CHR_WEIGHT_SCALE_FEATURE                       0x2A9E
#define UUID16_CHR_USER_CONTROL_POINT                         0x2A9F
#define UUID16_CHR_MAGNETIC_FLUX_DENSITY_2D                   0x2AA0
#define UUID16_CHR_MAGNETIC_FLUX_DENSITY_3D                   0x2AA1
#define UUID16_CHR_LANGUAGE                                   0x2AA2
#define UUID16_CHR_BAROMETRIC_PRESSURE_TREND                  0x2AA3
#define UUID16_CHR_BMS_CTRLPT                                 0x2AA4
#define UUID16_CHR_BMS_FEATURE                                0x2AA5
#define UUID16_CHR_CENTRAL_ADDRESS_RESOLUTION                 0x2AA6
#define UUID16_CHR_CGM_MEASUREMENT                            0x2AA7
#define UUID16_CHR_CGM_FEATURE                                0x2AA8
#define UUID16_CHR_CGM_STATUS                                 0x2AA9
#define UUID16_CHR_CGM_SESSION_START_TIME                     0x2AAA
#define UUID16_CHR_CGM_SESSION_RUN_TIME                       0x2AAB
#define UUID16_CHR_CGM_SPECIFIC_OPS_CTRLPT                    0x2AAC
#define UUID16_CHR_INDOOR_POSITIONING_CONFIGURATION           0x2AAD
#define UUID16_CHR_LATITUDE                                   0x2AAE
#define UUID16_CHR_LONGITUDE                                  0x2AAF
#define UUID16_CHR_LOCAL_NORTH_COORDINATE                     0x2AB0
#define UUID16_CHR_LOCAL_EAST_COORDINATE                      0x2AB1
#define UUID16_CHR_FLOOR_NUMBER                               0x2AB2
#define UUID16_CHR_ALTITUDE                                   0x2AB3
#define UUID16_CHR_UNCERTAINTY                                0x2AB4
#define UUID16_CHR_LOCATION_NAME                              0x2AB5
#define UUID16_CHR_URI                                        0x2AB6
#define UUID16_CHR_HTTP_HEADERS                               0x2AB7
#define UUID16_CHR_HTTP_STATUS_CODE                           0x2AB8
#define UUID16_CHR_HTTP_ENTITY_BODY                           0x2AB9
#define UUID16_CHR_HTTP_CONTROL_POINT                         0x2ABA
#define UUID16_CHR_HTTPS_SECURITY                             0x2ABB
#define UUID16_CHR_TDS_CONTROL_POINT                          0x2ABC
#define UUID16_CHR_OTS_FEATURES                               0x2ABD
#define UUID16_CHR_OTS_OBJECT_NAME                            0x2ABE
#define UUID16_CHR_OTS_OBJECT_TYPE                            0x2ABF
#define UUID16_CHR_OTS_OBJECT_SIZE                            0x2AC0
#define UUID16_CHR_OTS_OBJECT_FIRST_CREATED                   0x2AC1
#define UUID16_CHR_OTS_OBJECT_LAST_MODIFIED                   0x2AC2
#define UUID16_CHR_OTS_OBJECT_ID                              0x2AC3
#define UUID16_CHR_OTS_OBJECT_PROPERTIES                      0x2AC4
#define UUID16_CHR_OTS_OACP                                   0x2AC5
#define UUID16_CHR_OTS_OLCP                                   0x2AC6
#define UUID16_CHR_OTS_LF                                     0x2AC7
#define UUID16_CHR_OTS_OBJECT_CHANGED                         0x2AC8
#define UUID16_CHR_RESOLVABLE_PRIVATE_ADDRESS_ONLY            0x2AC9
#define UUID16_CHR_UNSPECIFIED                                0x2ACA
#define UUID16_CHR_DIRECTORY_LISTING                          0x2ACB
#define UUID16_CHR_FITNESS_MACHINE_FEATURE                    0x2ACC
#define UUID16_CHR_TREADMILL_DATA                             0x2ACD
#define UUID16_CHR_CROSS_TRAINER_DATA                         0x2ACE
#define UUID16_CHR_STEP_CLIMBER_DATA                          0x2ACF
#define UUID16_CHR_STAIR_CLIMBER_DATA                         0x2AD0
#define UUID16_CHR_ROWER_DATA                                 0x2AD1
#define UUID16_CHR_INDOOR_BIKE_DATA                           0x2AD2
#define UUID16_CHR_TRAINING_STATUS                            0x2AD3
#define UUID16_CHR_SUPPORTED_SPEED_RANGE                      0x2AD4
#define UUID16_CHR_SUPPORTED_INCLINATION_RANGE                0x2AD5
#define UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE           0x2AD6
#define UUID16_CHR_SUPPORTED_HEART_RATE_RANGE                 0x2AD7
#define UUID16_CHR_SUPPORTED_POWER_RANGE                      0x2AD8
#define UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT              0x2AD9
#define UUID16_CHR_FITNESS_MACHINE_STATUS                     0x2ADA
#define UUID16_CHR_MESH_PROVISIONING_DATA_IN                  0x2ADB
#define UUID16_CHR_MESH_PROVISIONING_DATA_OUT                 0X2ADC
#define UUID16_CHR_MESH_PROXY_DATA_IN                         0X2ADD
#define UUID16_CHR_MESH_PROXY_DATA_OUT                        0X2ADE
#define UUID16_CHR_AVERAGE_CURRENT                            0X2AE0
#define UUID16_CHR_AVERAGE_VOLTAGE                            0X2AE1
#define UUID16_CHR_BOOLEAN                                    0X2AE2
#define UUID16_CHR_CHROMATIC_DISTANCE_FROM_PLANCKIAN          0X2AE3
#define UUID16_CHR_CHROMATICITY_COORDINATES                   0X2AE4
#define UUID16_CHR_CHROMATICITY_IN_CCT_AND_DUV_VALUES         0X2AE5
#define UUID16_CHR_CHROMATICITY_TOLERANCE                     0X2AE6
#define UUID16_CHR_CIE_COLOR_RENDERING_INDEX                  0X2AE7
#define UUID16_CHR_COEFFICIENT                                0X2AE8
#define UUID16_CHR_CORRELATED_COLOR_TEMPERATURE               0X2AE9
#define UUID16_CHR_COUNT_16                                   0X2AEA
#define UUID16_CHR_COUNT_24                                   0X2AEB
#define UUID16_CHR_COUNTRY_CODE                               0X2AEC
#define UUID16_CHR_DATE_UTC                                   0X2AED
#define UUID16_CHR_ELECTRIC_CURRENT                           0X2AEE
#define UUID16_CHR_ELECTRIC_CURRENT_RANGE                     0X2AEF
#define UUID16_CHR_ELECTRIC_CURRENT_SPECIFICATION             0X2AF0
#define UUID16_CHR_ELECTRIC_CURRENT_STATISTICS                0X2AF1
#define UUID16_CHR_ENERGY                                     0X2AF2
#define UUID16_CHR_ENERGY_IN_A_PERIOD_OF_DAY                  0X2AF3
#define UUID16_CHR_EVENT_STATISTICS                           0X2AF4
#define UUID16_CHR_FIXED_STRING_16                            0X2AF5
#define UUID16_CHR_FIXED_STRING_24                            0X2AF6
#define UUID16_CHR_FIXED_STRING_36                            0X2AF7
#define UUID16_CHR_FIXED_STRING_8                             0X2AF8
#define UUID16_CHR_GENERIC_LEVEL                              0X2AF9
#define UUID16_CHR_GLOBAL_TRADE_ITEM_NUMBER                   0X2AFA
#define UUID16_CHR_ILLUMINANCE                                0X2AFB
#define UUID16_CHR_LUMINOUS_EFFICACY                          0X2AFC
#define UUID16_CHR_LUMINOUS_ENERGY                            0X2AFD
#define UUID16_CHR_LUMINOUS_EXPOSURE                          0X2AFE
#define UUID16_CHR_LUMINOUS_FLUX                              0X2AFF
#define UUID16_CHR_LUMINOUS_FLUX_RANGE                        0x2B00
#define UUID16_CHR_LUMINOUS_INTENSITY                         0x2B01
#define UUID16_CHR_MASS_FLOW                                  0x2B02
#define UUID16_CHR_PERCEIVED_LIGHTNESS                        0x2B03
#define UUID16_CHR_PERCENTAGE_8                               0x2B04
#define UUID16_CHR_POWER                                      0x2B05
#define UUID16_CHR_POWER_SPECIFICATION                        0x2B06
#define UUID16_CHR_RELATIVE_RUNTIME_IN_A_CURRENT_RANGE        0x2B07
#define UUID16_CHR_RELATIVE_RUNTIME_IN_A_GENERIC_LEVEL_RANGE  0x2B08
#define UUID16_CHR_RELATIVE_VALUE_IN_A_VOLTAGE_RANGE          0x2B09
#define UUID16_CHR_RELATIVE_VALUE_IN_AN_ILLUMINANCE_RANGE     0x2B0A
#define UUID16_CHR_RELATIVE_VALUE_IN_A_PERIOD_OF_DAY          0x2B0B
#define UUID16_CHR_RELATIVE_VALUE_IN_A_TEMPERATURE_RANGE      0x2B0C
#define UUID16_CHR_TEMPERATURE_8                              0x2B0D
#define UUID16_CHR_TEMPERATURE_8_IN_A_PERIOD_OF_DAY           0x2B0E
#define UUID16_CHR_TEMPERATURE_8_STATISTICS                   0x2B0F
#define UUID16_CHR_TEMPERATURE_RANGE                          0x2B10
#define UUID16_CHR_TEMPERATURE_STATISTICS                     0x2B11
#define UUID16_CHR_TIME_DECIHOUR_8                            0x2B12
#define UUID16_CHR_TIME_EXPONENTIAL_8                         0x2B13
#define UUID16_CHR_TIME_HOUR_24                               0x2B14
#define UUID16_CHR_TIME_MILLISECOND_24                        0x2B15
#define UUID16_CHR_TIME_SECOND_16                             0x2B16
#define UUID16_CHR_TIME_SECOND_8                              0x2B17
#define UUID16_CHR_VOLTAGE                                    0x2B18
#define UUID16_CHR_VOLTAGE_SPECIFICATION                      0x2B19
#define UUID16_CHR_VOLTAGE_STATISTICS                         0x2B1A
#define UUID16_CHR_VOLUME_FLOW                                0x2B1B
#define UUID16_CHR_CHROMATICITY_COORDINATE                    0x2B1C
#define UUID16_CHR_RC_FEATURE                                 0x2B1D
#define UUID16_CHR_RC_SETTINGS                                0x2B1E
#define UUID16_CHR_RECONNECTION_CONFIGURATION_CONTROL_POINT   0x2B1F
#define UUID16_CHR_IDD_STATUS_CHANGED                         0x2B20
#define UUID16_CHR_IDD_STATUS                                 0x2B21
#define UUID16_CHR_IDD_ANNUNCIATION_STATUS                    0x2B22
#define UUID16_CHR_IDD_FEATURES                               0x2B23
#define UUID16_CHR_IDD_STATUS_READER_CONTROL_POINT            0x2B24
#define UUID16_CHR_IDD_COMMAND_CONTROL_POINT                  0x2B25
#define UUID16_CHR_IDD_COMMAND_DATA                           0x2B26
#define UUID16_CHR_IDD_RECORD_ACCESS_CONTROL_POINT            0x2B27
#define UUID16_CHR_IDD_HISTORY_DATA                           0x2B28
#define UUID16_CHR_CLIENT_SUPPORTED_FEATURES                  0x2B29
#define UUID16_CHR_DATABASE_HASH                              0x2B2A
#define UUID16_CHR_BSS_CONTROL_POINT                          0x2B2B
#define UUID16_CHR_BSS_RESPONSE                               0x2B2C
#define UUID16_CHR_EMERGENCY_ID                               0x2B2D
#define UUID16_CHR_EMERGENCY_TEXT                             0x2B2E
#define UUID16_CHR_ENHANCED_BLOOD_PRESSURE_MEASUREMENT        0x2B34
#define UUID16_CHR_ENHANCED_INTERMEDIATE_CUFF_PRESSURE        0x2B35
#define UUID16_CHR_BLOOD_PRESSURE_RECORD                      0x2B36
#define UUID16_CHR_BR_EDR_HANDOVER_DATA                       0x2B38
#define UUID16_CHR_BLUETOOTH_SIG_DATA                         0x2B39
#define UUID16_CHR_SERVER_SUPPORTED_FEATURES                  0x2B3A
#define UUID16_CHR_PHYSICAL_ACTIVITY_MONITOR_FEATURES         0x2B3B
#define UUID16_CHR_GENERAL_ACTIVITY_INST_DATA                 0x2B3C
#define UUID16_CHR_GENERAL_ACTIVITY_SUMMARY_DATA              0x2B3D
#define UUID16_CHR_CARDIORESPIRATORY_ACTIVITY_INST_DATA       0x2B3E
#define UUID16_CHR_CARDIORESPIRATORY_ACTIVITY_SUMMARY_DATA    0x2B3F
#define UUID16_CHR_STEP_COUNTER_ACTIVITY_SUMMARY_DATA         0x2B40
#define UUID16_CHR_SLEEP_ACTIVITY_INST_DATA                   0x2B41
#define UUID16_CHR_SLEEP_ACTIVITY_SUMMARY_DATA                0x2B42
#define UUID16_CHR_PHYSICAL_ACTIVITY_MONITOR_CONTROL_POINT    0x2B43
#define UUID16_CHR_CURRENT_SESSION                            0x2B44
#define UUID16_CHR_SESSION                                    0x2B45
#define UUID16_CHR_PREFERRED_UNITS                            0x2B46
#define UUID16_CHR_HIGH_RESOLUTION_HEIGHT                     0x2B47
#define UUID16_CHR_MIDDLE_NAME                                0x2B48
#define UUID16_CHR_STRIDE_LENGTH                              0x2B49
#define UUID16_CHR_HANDEDNESS                                 0x2B4A
#define UUID16_CHR_DEVICE_WEARING_POSITION                    0x2B4B
#define UUID16_CHR_FOUR_ZONE_HEART_RATE_LIMITS                0x2B4C
#define UUID16_CHR_HIGH_INTENSITY_EXERCISE_THRESHOLD          0x2B4D
#define UUID16_CHR_ACTIVITY_GOAL                              0x2B4E
#define UUID16_CHR_SEDENTARY_INTERVAL_NOTIFICATION            0x2B4F
#define UUID16_CHR_CALORIC_INTAKE                             0x2B50
#define UUID16_CHR_AUDIO_INPUT_STATE                          0x2B77
#define UUID16_CHR_GAIN_SETTINGS_ATTRIBUTE                    0x2B78
#define UUID16_CHR_AUDIO_INPUT_TYPE                           0x2B79
#define UUID16_CHR_AUDIO_INPUT_STATUS                         0x2B7A
#define UUID16_CHR_AUDIO_INPUT_CONTROL_POINT                  0x2B7B
#define UUID16_CHR_AUDIO_INPUT_DESCRIPTION                    0x2B7C
#define UUID16_CHR_VOLUME_STATE                               0x2B7D
#define UUID16_CHR_VOLUME_CONTROL_POINT                       0x2B7E
#define UUID16_CHR_VOLUME_FLAGS                               0x2B7F
#define UUID16_CHR_OFFSET_STATE                               0x2B80
#define UUID16_CHR_AUDIO_LOCATION                             0x2B81
#define UUID16_CHR_VOLUME_OFFSET_CONTROL_POINT                0x2B82
#define UUID16_CHR_AUDIO_OUTPUT_DESCRIPTION                   0x2B83
#define UUID16_CHR_SET_IDENTITY_RESOLVING_KEY_CHARACTERISTIC  0x2B84
#define UUID16_CHR_SIZE_CHARACTERISTIC                        0x2B85
#define UUID16_CHR_LOCK_CHARACTERISTIC                        0x2B86
#define UUID16_CHR_RANK_CHARACTERISTIC                        0x2B87
#define UUID16_CHR_DEVICE_TIME_FEATURE                        0x2B8E
#define UUID16_CHR_DEVICE_TIME_PARAMETERS                     0x2B8F
#define UUID16_CHR_DEVICE_TIME                                0x2B90
#define UUID16_CHR_DEVICE_TIME_CONTROL_POINT                  0x2B91
#define UUID16_CHR_TIME_CHANGE_LOG_DATA                       0x2B92
#define UUID16_CHR_MEDIA_PLAYER_NAME                          0x2B93
#define UUID16_CHR_MEDIA_PLAYER_ICON_OBJECT_ID                0x2B94
#define UUID16_CHR_MEDIA_PLAYER_ICON_URL                      0x2B95
#define UUID16_CHR_TRACK_CHANGED                              0x2B96
#define UUID16_CHR_TRACK_TITLE                                0x2B97
#define UUID16_CHR_TRACK_DURATION                             0x2B98
#define UUID16_CHR_TRACK_POSITION                             0x2B99
#define UUID16_CHR_PLAYBACK_SPEED                             0x2B9A
#define UUID16_CHR_SEEKING_SPEED                              0x2B9B
#define UUID16_CHR_CURRENT_TRACK_SEGMENTS_OBJECT_ID           0x2B9C
#define UUID16_CHR_CURRENT_TRACK_OBJECT_ID                    0x2B9D
#define UUID16_CHR_NEXT_TRACK_OBJECT_ID                       0x2B9E
#define UUID16_CHR_PARENT_GROUP_OBJECT_ID                     0x2B9F
#define UUID16_CHR_CURRENT_GROUP_OBJECT_ID                    0x2BA0
#define UUID16_CHR_PLAYING_ORDER                              0x2BA1
#define UUID16_CHR_PLAYING_ORDERS_SUPPORTED                   0x2BA2
#define UUID16_CHR_MEDIA_STATE                                0x2BA3
#define UUID16_CHR_MEDIA_CONTROL_POINT                        0x2BA4
#define UUID16_CHR_MEDIA_CONTROL_POINT_OPCODES_SUPPORTED      0x2BA5
#define UUID16_CHR_SEARCH_RESULTS_OBJECT_ID                   0x2BA6
#define UUID16_CHR_SEARCH_CONTROL_POINT                       0x2BA7
#define UUID16_CHR_MEDIA_PLAYER_ICON_OBJECT_TYPE              0x2BA9
#define UUID16_CHR_TRACK_SEGMENTS_OBJECT_TYPE                 0x2BAA
#define UUID16_CHR_TRACK_OBJECT_TYPE                          0x2BAB
#define UUID16_CHR_GROUP_OBJECT_TYPE                          0x2BAC
#define UUID16_CHR_CTE_ENABLE                                 0x2BAD
#define UUID16_CHR_ADVERTISING_CTE_MINIMUM_LENGTH             0x2BAE
#define UUID16_CHR_ADVERTISING_CTE_MINIMUM_TRANSMIT_COUNT     0x2BAF
#define UUID16_CHR_ADVERTISING_CTE_TRANSMIT_DURATION          0x2BB0
#define UUID16_CHR_ADVERTISING_CTE_INTERVAL                   0x2BB1
#define UUID16_CHR_ADVERTISING_CTE_PHY                        0x2BB2
#define UUID16_CHR_BEARER_PROVIDER_NAME                       0x2BB3
#define UUID16_CHR_BEARER_UCI                                 0x2BB4
#define UUID16_CHR_BEARER_TECHNOLOGY                          0x2BB5
#define UUID16_CHR_BEARER_URI_SCHEMES_SUPPORTED_LIST          0x2BB6
#define UUID16_CHR_BEARER_SIGNAL_STRENGTH                     0x2BB7
#define UUID16_CHR_BEARER_SIGNAL_STRENGTH_REPORTING_INTERVAL  0x2BB8
#define UUID16_CHR_BEARER_LIST_CURRENT_CALLS                  0x2BB9
#define UUID16_CHR_CONTENT_CONTROL_ID                         0x2BBA
#define UUID16_CHR_STATUS_FLAGS                               0x2BBB
#define UUID16_CHR_INCOMING_CALL_TARGET_BEARER_URI            0x2BBC
#define UUID16_CHR_CALL_STATE                                 0x2BBD
#define UUID16_CHR_CALL_CONTROL_POINT                         0x2BBE
#define UUID16_CHR_CALL_CONTROL_POINT_OPTIONAL_OPCODES        0x2BBF
#define UUID16_CHR_TERMINATION_REASON                         0x2BC0
#define UUID16_CHR_INCOMING_CALL                              0x2BC1
#define UUID16_CHR_CALL_FRIENDLY_NAME                         0x2BC2
#define UUID16_CHR_MUTE                                       0x2BC3

/*------------------------------------------------------------------*/
/* Company UUID
 * https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers
 *------------------------------------------------------------------*/
#define UUID16_COMPANY_ID_APPLE         0x004C
#define UUID16_COMPANY_ID_ADAFRUIT      0x0822

/*------------------------------------------------------------------*/
/* Unit values ( used in Characteristic Presentation Format )
 * https://developer.bluetooth.org/gatt/units/Pages/default.aspx
 *------------------------------------------------------------------*/
#define UUID16_UNIT_UNITLESS                                                0x2700
#define UUID16_UNIT_LENGTH_METRE                                            0x2701
#define UUID16_UNIT_MASS_KILOGRAM                                           0x2702
#define UUID16_UNIT_TIME_SECOND                                             0x2703
#define UUID16_UNIT_ELECTRIC_CURRENT_AMPERE                                 0x2704
#define UUID16_UNIT_THERMODYNAMIC_TEMPERATURE_KELVIN                        0x2705
#define UUID16_UNIT_AMOUNT_OF_SUBSTANCE_MOLE                                0x2706
#define UUID16_UNIT_LUMINOUS_INTENSITY_CANDELA                              0x2707
#define UUID16_UNIT_AREA_SQUARE_METRES                                      0x2710
#define UUID16_UNIT_VOLUME_CUBIC_METRES                                     0x2711
#define UUID16_UNIT_VELOCITY_METRES_PER_SECOND                              0x2712
#define UUID16_UNIT_ACCELERATION_METRES_PER_SECOND_SQUARED                  0x2713
#define UUID16_UNIT_WAVENUMBER_RECIPROCAL_METRE                             0x2714
#define UUID16_UNIT_DENSITY_KILOGRAM_PER_CUBIC_METRE                        0x2715
#define UUID16_UNIT_SURFACE_DENSITY_KILOGRAM_PER_SQUARE_METRE               0x2716
#define UUID16_UNIT_SPECIFIC_VOLUME_CUBIC_METRE_PER_KILOGRAM                0x2717
#define UUID16_UNIT_CURRENT_DENSITY_AMPERE_PER_SQUARE_METRE                 0x2718
#define UUID16_UNIT_MAGNETIC_FIELD_STRENGTH_AMPERE_PER_METRE                0x2719
#define UUID16_UNIT_AMOUNT_CONCENTRATION_MOLE_PER_CUBIC_METRE               0x271A
#define UUID16_UNIT_MASS_CONCENTRATION_KILOGRAM_PER_CUBIC_METRE             0x271B
#define UUID16_UNIT_LUMINANCE_CANDELA_PER_SQUARE_METRE                      0x271C
#define UUID16_UNIT_REFRACTIVE_INDEX                                        0x271D
#define UUID16_UNIT_RELATIVE_PERMEABILITY                                   0x271E
#define UUID16_UNIT_PLANE_ANGLE_RADIAN                                      0x2720
#define UUID16_UNIT_SOLID_ANGLE_STERADIAN                                   0x2721
#define UUID16_UNIT_FREQUENCY_HERTZ                                         0x2722
#define UUID16_UNIT_FORCE_NEWTON                                            0x2723
#define UUID16_UNIT_PRESSURE_PASCAL                                         0x2724
#define UUID16_UNIT_ENERGY_JOULE                                            0x2725
#define UUID16_UNIT_POWER_WATT                                              0x2726
#define UUID16_UNIT_ELECTRIC_CHARGE_COULOMB                                 0x2727
#define UUID16_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE_VOLT                      0x2728
#define UUID16_UNIT_CAPACITANCE_FARAD                                       0x2729
#define UUID16_UNIT_ELECTRIC_RESISTANCE_OHM                                 0x272A
#define UUID16_UNIT_ELECTRIC_CONDUCTANCE_SIEMENS                            0x272B
#define UUID16_UNIT_MAGNETIC_FLEX_WEBER                                     0x272C
#define UUID16_UNIT_MAGNETIC_FLEX_DENSITY_TESLA                             0x272D
#define UUID16_UNIT_INDUCTANCE_HENRY                                        0x272E
#define UUID16_UNIT_THERMODYNAMIC_TEMPERATURE_DEGREE_CELSIUS                0x272F
#define UUID16_UNIT_LUMINOUS_FLUX_LUMEN                                     0x2730
#define UUID16_UNIT_ILLUMINANCE_LUX                                         0x2731
#define UUID16_UNIT_ACTIVITY_REFERRED_TO_A_RADIONUCLIDE_BECQUEREL           0x2732
#define UUID16_UNIT_ABSORBED_DOSE_GRAY                                      0x2733
#define UUID16_UNIT_DOSE_EQUIVALENT_SIEVERT                                 0x2734
#define UUID16_UNIT_CATALYTIC_ACTIVITY_KATAL                                0x2735
#define UUID16_UNIT_DYNAMIC_VISCOSITY_PASCAL_SECOND                         0x2740
#define UUID16_UNIT_MOMENT_OF_FORCE_NEWTON_METRE                            0x2741
#define UUID16_UNIT_SURFACE_TENSION_NEWTON_PER_METRE                        0x2742
#define UUID16_UNIT_ANGULAR_VELOCITY_RADIAN_PER_SECOND                      0x2743
#define UUID16_UNIT_ANGULAR_ACCELERATION_RADIAN_PER_SECOND_SQUARED          0x2744
#define UUID16_UNIT_HEAT_FLUX_DENSITY_WATT_PER_SQUARE_METRE                 0x2745
#define UUID16_UNIT_HEAT_CAPACITY_JOULE_PER_KELVIN                          0x2746
#define UUID16_UNIT_SPECIFIC_HEAT_CAPACITY_JOULE_PER_KILOGRAM_KELVIN        0x2747
#define UUID16_UNIT_SPECIFIC_ENERGY_JOULE_PER_KILOGRAM                      0x2748
#define UUID16_UNIT_THERMAL_CONDUCTIVITY_WATT_PER_METRE_KELVIN              0x2749
#define UUID16_UNIT_ENERGY_DENSITY_JOULE_PER_CUBIC_METRE                    0x274A
#define UUID16_UNIT_ELECTRIC_FIELD_STRENGTH_VOLT_PER_METRE                  0x274B
#define UUID16_UNIT_ELECTRIC_CHARGE_DENSITY_COULOMB_PER_CUBIC_METRE         0x274C
#define UUID16_UNIT_SURFACE_CHARGE_DENSITY_COULOMB_PER_SQUARE_METRE         0x274D
#define UUID16_UNIT_ELECTRIC_FLUX_DENSITY_COULOMB_PER_SQUARE_METRE          0x274E
#define UUID16_UNIT_PERMITTIVITY_FARAD_PER_METRE                            0x274F
#define UUID16_UNIT_PERMEABILITY_HENRY_PER_METRE                            0x2750
#define UUID16_UNIT_MOLAR_ENERGY_JOULE_PER_MOLE                             0x2751
#define UUID16_UNIT_MOLAR_ENTROPY_JOULE_PER_MOLE_KELVIN                     0x2752
#define UUID16_UNIT_EXPOSURE_COULOMB_PER_KILOGRAM                           0x2753
#define UUID16_UNIT_ABSORBED_DOSE_RATE_GRAY_PER_SECOND                      0x2754
#define UUID16_UNIT_RADIANT_INTENSITY_WATT_PER_STERADIAN                    0x2755
#define UUID16_UNIT_RADIANCE_WATT_PER_SQUARE_METRE_STERADIAN                0x2756
#define UUID16_UNIT_CATALYTIC_ACTIVITY_CONCENTRATION_KATAL_PER_CUBIC_METRE  0x2757
#define UUID16_UNIT_TIME_MINUTE                                             0x2760
#define UUID16_UNIT_TIME_HOUR                                               0x2761
#define UUID16_UNIT_TIME_DAY                                                0x2762
#define UUID16_UNIT_PLANE_ANGLE_DEGREE                                      0x2763
#define UUID16_UNIT_PLANE_ANGLE_MINUTE                                      0x2764
#define UUID16_UNIT_PLANE_ANGLE_SECOND                                      0x2765
#define UUID16_UNIT_AREA_HECTARE                                            0x2766
#define UUID16_UNIT_VOLUME_LITRE                                            0x2767
#define UUID16_UNIT_MASS_TONNE                                              0x2768
#define UUID16_UNIT_PRESSURE_BAR                                            0x2780
#define UUID16_UNIT_PRESSURE_MILLIMETRE_OF_MERCURY                          0x2781
#define UUID16_UNIT_LENGTH_ANGSTROM                                         0x2782
#define UUID16_UNIT_LENGTH_NAUTICAL_MILE                                    0x2783
#define UUID16_UNIT_AREA_BARN                                               0x2784
#define UUID16_UNIT_VELOCITY_KNOT                                           0x2785
#define UUID16_UNIT_LOGARITHMIC_RADIO_QUANTITY_NEPER                        0x2786
#define UUID16_UNIT_LOGARITHMIC_RADIO_QUANTITY_BEL                          0x2787
#define UUID16_UNIT_LENGTH_YARD                                             0x27A0
#define UUID16_UNIT_LENGTH_PARSEC                                           0x27A1
#define UUID16_UNIT_LENGTH_INCH                                             0x27A2
#define UUID16_UNIT_LENGTH_FOOT                                             0x27A3
#define UUID16_UNIT_LENGTH_MILE                                             0x27A4
#define UUID16_UNIT_PRESSURE_POUND_FORCE_PER_SQUARE_INCH                    0x27A5
#define UUID16_UNIT_VELOCITY_KILOMETRE_PER_HOUR                             0x27A6
#define UUID16_UNIT_VELOCITY_MILE_PER_HOUR                                  0x27A7
#define UUID16_UNIT_ANGULAR_VELOCITY_REVOLUTION_PER_MINUTE                  0x27A8
#define UUID16_UNIT_ENERGY_GRAM_CALORIE                                     0x27A9
#define UUID16_UNIT_ENERGY_KILOGRAM_CALORIE                                 0x27AA
#define UUID16_UNIT_ENERGY_KILOWATT_HOUR                                    0x27AB
#define UUID16_UNIT_THERMODYNAMIC_TEMPERATURE_DEGREE_FAHRENHEIT             0x27AC
#define UUID16_UNIT_PERCENTAGE                                              0x27AD
#define UUID16_UNIT_PER_MILLE                                               0x27AE
#define UUID16_UNIT_PERIOD_BEATS_PER_MINUTE                                 0x27AF
#define UUID16_UNIT_ELECTRIC_CHARGE_AMPERE_HOURS                            0x27B0
#define UUID16_UNIT_MASS_DENSITY_MILLIGRAM_PER_DECILITRE                    0x27B1
#define UUID16_UNIT_MASS_DENSITY_MILLIMOLE_PER_LITRE                        0x27B2
#define UUID16_UNIT_TIME_YEAR                                               0x27B3
#define UUID16_UNIT_TIME_MONTH                                              0x27B4
#define UUID16_UNIT_CONCENTRATION_COUNT_PER_CUBIC_METRE                     0x27B5
#define UUID16_UNIT_IRRADIANCE_WATT_PER_SQUARE_METRE                        0x27B6
#define UUID16_UNIT_MILLILITER_PER_KILOGRAM_PER_MINUTE                      0x27B7
#define UUID16_UNIT_MASS_POUND                                              0x27B8
#define UUID16_UNIT_METABOLIC_EQUIVALENT                                    0x27B9
#define UUID16_UNIT_STEP_PER_MINUTE                                         0x27BA
#define UUID16_UNIT_STROKE_PER_MINUTE                                       0x27BC
#define UUID16_UNIT_PACE_KILOMETRE_PER_MINUTE                               0x27BD
#define UUID16_UNIT_LUMINOUS_EFFICACY_LUMEN_PER_WATT                        0x27BE
#define UUID16_UNIT_LUMINOUS_ENERGY_LUMEN_HOUR                              0x27BF
#define UUID16_UNIT_LUMINOUS_EXPOSURE_LUX_HOUR                              0x27C0
#define UUID16_UNIT_MASS_FLOW_GRAM_PER_SECOND                               0x27C1
#define UUID16_UNIT_VOLUME_FLOW_LITRE_PER_SECOND                            0x27C2
#define UUID16_UNIT_SOUND_PRESSURE_DECIBLE                                  0x27C3
#define UUID16_UNIT_PARTS_PER_MILLION                                       0x27C4
#define UUID16_UNIT_PARTS_PER_BILLION                                       0x27C5

#endif /* BLEUUID_H_ */
