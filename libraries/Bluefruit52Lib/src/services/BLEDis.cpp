/**************************************************************************/
/*!
    @file     BLEDis.cpp
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

#include "bluefruit.h"
#include "utility/utilities.h"

BLEDis::BLEDis(void)
  : BLEService(UUID16_SVC_DEVICE_INFORMATION)
{
#ifdef NRF52840_XXAA
  _model        = "Bluefruit Feather nRF52840";
#else
  _model        = "Bluefruit Feather nRF52832";
#endif

  _serial       = NULL;
  _firmware_rev = NULL;
  _hardware_rev = NULL;
  _software_rev = ARDUINO_BSP_VERSION;
  _manufacturer = "Adafruit Industries";
  _system_id    = NULL;
  _reg_cert_list = NULL;
  _pnp_id       = NULL;
}

void BLEDis::setFirmwareRev(const char* firmware_rev)
{
  _firmware_rev = firmware_rev;
  _firmware_rev_length = strlen(firmware_rev);
}
void BLEDis::setFirmwareRev(const char* firmware_rev, uint8_t length)
{
  _firmware_rev = firmware_rev;
  _firmware_rev_length = length;
}

void BLEDis::setSerialNum(const char* serial_num)
{
  _serial = serial_num;
  _serial_length = strlen(serial_num);
}
void BLEDis::setSerialNum(const char* serial_num, uint8_t length)
{
  _serial = serial_num;
  _serial_length = length;
}

void BLEDis::setSystemID(const char* system_id)
{
  _system_id = system_id;
  _system_id_length = strlen(system_id);
}
void BLEDis::setSystemID(const char* system_id, uint8_t length)
{
  _system_id = system_id;
  _system_id_length = length;
}

void BLEDis::setRegCertList(const char* reg_cert_list)
{
  _reg_cert_list = reg_cert_list;
  _reg_cert_list_length = strlen(reg_cert_list);
}
void BLEDis::setRegCertList(const char* reg_cert_list, uint8_t length)
{
  _reg_cert_list = reg_cert_list;
  _reg_cert_list_length = length;
}

void BLEDis::setPNPID(const char* pnp_id)
{
  _pnp_id = pnp_id;
  _pnp_id_length = strlen(pnp_id);
}
void BLEDis::setPNPID(const char* pnp_id, uint8_t length)
{
  _pnp_id = pnp_id;
  _pnp_id_length = length;
}
void BLEDis::setModel(const char* model,uint8_t length)
{
  _model = model;
  _model_length = length;
}

void BLEDis::setHardwareRev(const char* hw_rev,uint8_t length)
{
  _hardware_rev = hw_rev;
  _hardware_rev_length = length;
}

void BLEDis::setSoftwareRev(const char* sw_rev, uint8_t length)
{
  _software_rev = sw_rev;
  _software_rev_length = length;
}

void BLEDis::setManufacturer(const char* manufacturer, uint8_t length)
{
  _manufacturer = manufacturer;
  _manufacturer_length = length;
}

void BLEDis::setModel(const char* model)
{
  _model = model;
  _model_length = strlen(model);
}

void BLEDis::setHardwareRev(const char* hw_rev)
{
  _hardware_rev = hw_rev;
  _hardware_rev_length = strlen(hw_rev);
}

void BLEDis::setSoftwareRev(const char* sw_rev)
{
  _software_rev = sw_rev;
  _software_rev_length = strlen(sw_rev);
}

void BLEDis::setManufacturer(const char* manufacturer)
{
  _manufacturer = manufacturer;
  _manufacturer_length = strlen(manufacturer);
}

err_t BLEDis::begin(void)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  if(!_serial) {
    _serial       = getMcuUniqueID();
  }
  if (!_firmware_rev) {
    _firmware_rev = getBootloaderVersion();
  }

  for(uint8_t i=0; i<arrcount(_strarr)-1; i++)    // without PNP_ID
  {
    if ( _strarr[i] != NULL )
    {
      BLECharacteristic chars(UUID16_CHR_SYSTEM_ID+i);
      chars.setTempMemory();
      chars.setProperties(CHR_PROPS_READ);
      if (_strarr_length[i]) {
        chars.setFixedLen(_strarr_length[i]);
      } else {
        chars.setFixedLen(strlen(_strarr[i]));
      }
      VERIFY_STATUS( chars.begin() );
      if (_strarr_length[i]) {
        chars.write(_strarr[i], _strarr_length[i]);
      } else {
        chars.write(_strarr[i]);
      }
      
    }
  }

  if ( _strarr[arrcount(_strarr)-1] != NULL )
    {
      BLECharacteristic chars(UUID16_CHR_PNP_ID);
      chars.setTempMemory();
      chars.setProperties(CHR_PROPS_READ);
      if (_pnp_id_length == 0) {
        chars.setFixedLen(strlen(_strarr[arrcount(_strarr)-1]));
      } else {
        chars.setFixedLen(_pnp_id_length);
      }
      VERIFY_STATUS( chars.begin() );
      if (_pnp_id_length == 0) {
        chars.write(_strarr[arrcount(_strarr)-1]);
      } else {
        chars.write(_strarr[arrcount(_strarr)-1], _pnp_id_length);
      }
    }

  return ERROR_NONE;
}
