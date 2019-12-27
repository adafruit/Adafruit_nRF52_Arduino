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
    
    // Constructors
    BLEUuid(void                      ) { _uuid.type = BLE_UUID_TYPE_UNKNOWN; _uuid.uuid = 0; _uuid128 = NULL; }
    BLEUuid(uint16_t uuid16           ) { set(uuid16 );                  }
    BLEUuid(uint8_t const uuid128[16] ) { set(uuid128);                  }
    BLEUuid(ble_uuid_t uuid           ) { _uuid = uuid; _uuid128 = NULL; }

    void set(uint16_t uuid16);
    void set(uint8_t const uuid128[16]);

    bool get(uint16_t* uuid16) const;
    bool get(uint8_t uuid128[16]);

    size_t size (void) const;

    // Add UUID128 if needed, in case of UUID16, no actions is required
    bool begin(void);

    bool operator==(const BLEUuid&   uuid) const;
    bool operator!=(const BLEUuid&   uuid) const;
    bool operator==(const ble_uuid_t uuid) const;
    bool operator!=(const ble_uuid_t uuid) const;

    // Overload copy operator to allow initialization from other type
    BLEUuid& operator=(const uint16_t uuid);
    BLEUuid& operator=(uint8_t const uuid128[16]);
    BLEUuid& operator=(ble_uuid_t uuid);
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
#define UUID16_SVC_ALERT_NOTIFICATION                         0x1811
#define UUID16_SVC_BATTERY                                    0x180F
#define UUID16_SVC_BLOOD_PRESSURE                             0x1810
#define UUID16_SVC_CURRENT_TIME                               0x1805
#define UUID16_SVC_CYCLING_SPEED_AND_CADENCE                  0x1816
#define UUID16_SVC_CYCLING_POWER                              0x1818
#define UUID16_SVC_LOCATION_AND_NAVIGATION                    0x1819
#define UUID16_SVC_DEVICE_INFORMATION                         0x180A
#define UUID16_SVC_ENVIRONMENTAL_SENSING                      0x181A
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
#define UUID16_SVC_OTS                                        0x1825

#define UUID16_SVC_EDDYSTONE                                  0xFEAA

//
#define UUID16_EXTERNAL_REPORT_REF_DESCR                      0x2907
#define UUID16_REPORT_REF_DESCR                               0x2908

/*------------------------------------------------------------------*/
/* Characteristic UUID
 * https://www.bluetooth.com/specifications/gatt/characteristics
 *------------------------------------------------------------------*/
#define UUID16_CHR_TEMPERATURE_CELSIUS                        0x2A1F
#define UUID16_CHR_TEMPERATURE_FAHRENHEIT                     0x2A20
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
#define UUID16_CHR_CYCLING_POWER_MEASUREMENT                  0x2A63
#define UUID16_CHR_CYCLING_POWER_FEATURE                      0x2A65
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
#define UUID16_CHR_LN_FEATURE                                 0x2A6A
#define UUID16_CHR_LN_POSITION_QUALITY                        0x2A69
#define UUID16_CHR_LN_LOCATION_AND_SPEED                      0x2A67
#define UUID16_CHR_LN_NAVIGATION                              0x2A68
#define UUID16_CHR_LN_CONTROL_POINT                           0x2A6B
#define UUID16_CHR_BMS_CTRLPT                                 0x2AA4
#define UUID16_CHR_BMS_FEATURE                                0x2AA5
#define UUID16_CHR_CGM_MEASUREMENT                            0x2AA7
#define UUID16_CHR_CGM_FEATURE                                0x2AA8
#define UUID16_CHR_CGM_STATUS                                 0x2AA9
#define UUID16_CHR_CGM_SESSION_START_TIME                     0x2AAA
#define UUID16_CHR_CGM_SESSION_RUN_TIME                       0x2AAB
#define UUID16_CHR_CGM_SPECIFIC_OPS_CTRLPT                    0x2AAC
#define UUID16_CHR_PLX_SPOT_CHECK_MEAS                        0x2A5E
#define UUID16_CHR_PLX_CONTINUOUS_MEAS                        0x2A5F
#define UUID16_CHR_PLX_FEATURES                               0x2A60
#define UUID16_CHR_TEMPERATURE                                0x2A6E
#define UUID16_CHR_UV_INDEX                                   0x2A76
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

#endif /* BLEUUID_H_ */
