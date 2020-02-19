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
  memclr(_strarr, sizeof(_strarr));
  memclr(_strarr_length, sizeof(_strarr_length));

#ifdef USB_PRODUCT
  this->setModel(USB_PRODUCT);
#else
  this->setModel("Feather nRF52832");
#endif

  this->setSoftwareRev(ARDUINO_BSP_VERSION);
  this->setManufacturer("Adafruit Industries");
}

void BLEDis::setSystemID(const char* system_id, uint8_t length)
{
  _system_id = system_id;
  _system_id_length = length;
}

void BLEDis::setModel(const char* model,uint8_t length)
{
  _model = model;
  _model_length = length;
}

void BLEDis::setSerialNum(const char* serial_num, uint8_t length)
{
  _serial = serial_num;
  _serial_length = length;
}

void BLEDis::setFirmwareRev(const char* firmware_rev, uint8_t length)
{
  _firmware_rev = firmware_rev;
  _firmware_rev_length = length;
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

void BLEDis::setRegCertList(const char* reg_cert_list, uint8_t length)
{
  _reg_cert_list = reg_cert_list;
  _reg_cert_list_length = length;
}

void BLEDis::setPNPID(const char* pnp_id, uint8_t length)
{
  _pnp_id = pnp_id;
  _pnp_id_length = length;
}


err_t BLEDis::begin(void)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  if (!_serial) setSerialNum(getMcuUniqueID());
  if (!_firmware_rev) setFirmwareRev(getBootloaderVersion());

  for(uint8_t i=0; i<arrcount(_strarr); i++)
  {
    if ( _strarr[i] != NULL )
    {
      BLECharacteristic chars;

      // PNP_ID is not consecutive with the rest
      if ( _strarr[i] == _pnp_id )
      {
        chars.setUuid(UUID16_CHR_PNP_ID);
      }else
      {
        chars.setUuid(UUID16_CHR_SYSTEM_ID+i);
      }

      chars.setTempMemory();
      chars.setProperties(CHR_PROPS_READ);
      chars.setFixedLen(_strarr_length[i]);
      VERIFY_STATUS( chars.begin() );
      chars.write(_strarr[i], _strarr_length[i]);
    }
  }

  return ERROR_NONE;
}
