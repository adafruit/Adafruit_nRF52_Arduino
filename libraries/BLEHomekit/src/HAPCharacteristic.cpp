/**************************************************************************/
/*!
    @file     HAPCharacteristic.cpp
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
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

#include <bluefruit.h>
#include "BLEHomekit.h"

BLEUuid HAPCharacteristic::_g_uuid_cid(HAP_UUID_DSC_CHARACTERISTIC_ID);


/**
 *
 * @param bleuuid   Uuid of the characteristic
 * @param format    BLE_GATT_CPF_FORMAT_x value, see ble_gatt.h
 * @param unit      UUID16_UNIT_x, see BLEUuid.h
 */
HAPCharacteristic::HAPCharacteristic(BLEUuid bleuuid, uint8_t format, uint16_t unit)
 : BLECharacteristic(bleuuid)
{
  _cid = 0;
  _hap_props = 0;

  setPresentationFormatDescriptor(format, 0, unit, 1, 0);
}

void HAPCharacteristic::setHapProperties(uint16_t prop)
{
  _hap_props = prop;

  uint8_t chrprop = 0;

  if ((prop & HAP_CHR_PROPS_READ  ) || (prop & HAP_CHR_PROPS_SECURE_READ        )) chrprop |= CHR_PROPS_READ;
  if ((prop & HAP_CHR_PROPS_WRITE ) || (prop & HAP_CHR_PROPS_SECURE_WRITE       )) chrprop |= CHR_PROPS_WRITE;
  if ((prop & HAP_CHR_PROPS_NOTIFY) || (prop & HAP_CHR_PROPS_NOTIFY_DISCONNECTED)) chrprop |= CHR_PROPS_INDICATE;

  setProperties(chrprop);
}

/**
 * GATT Descriptor includes: Instance ID
 * HAP  Descriptor includes: HAP Properties, User String, Presentation Format,
 *                           Valid Range (GATT), Step Value, Valid Values, Valid Values Range
 * @return
 */
err_t HAPCharacteristic::begin(void)
{
  // Needed to response to HAP Procedure
  _attr_meta.rd_auth = _attr_meta.wr_auth = 1;

  // Descriptors are not included in Gatt Table, only returned in
  // HAP Characteristic Signature Read procedure.
  // Including: User String, Presentation Format
  uint8_t temp_format = _format_desc.format;
  _format_desc.format = 0; // invalid to prevent adding presentation format

  const char* temp_usr = _usr_descriptor;
  _usr_descriptor = NULL;

  VERIFY_STATUS( BLECharacteristic::begin() );
  VERIFY_STATUS( _addChrIdDescriptor() );

  // Restore configured setting
  _format_desc.format = temp_format;
  _usr_descriptor = temp_usr;

  return ERROR_NONE;
}

/**
 * Add Characteristic Instance ID descriptor
 * @return status code
 */
err_t HAPCharacteristic::_addChrIdDescriptor(void)
{
  // Save Characteristic Instance ID
  _cid = BLEHomekit::_gInstanceID++;

  return addDescriptor(_g_uuid_cid, &_cid, sizeof(_cid), SECMODE_OPEN, SECMODE_NO_ACCESS);
}

err_t HAPCharacteristic::_addHapDescriptor(void)
{
//  HAPCharacteristic

  return ERROR_NONE;
}

void HAPCharacteristic::_eventHandler(ble_evt_t* event)
{
  switch(event->header.evt_id)
  {
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
      ble_gatts_evt_rw_authorize_request_t * request = &event->evt.gatts_evt.params.authorize_request;

      // Handle HAP Request
      if (request->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
      {
        HAPRequest_t* hap_req = (HAPRequest_t*) request->request.write.data;
        VERIFY(hap_req->control.type == HAP_PDU_REQUEST, );

//        ble_gatts_rw_authorize_reply_params_t reply =
//        {
//            .type = request->type,
//            .params.write =
//            {
//                .gatt_status = BLE_GATT_STATUS_SUCCESS,
//                .
//            };
//        };

//      sd_ble_gatts_rw_authorize_reply(Bluefruit.connHandle(), &reply);

        switch(hap_req->opcode)
        {
          /* Return <Chr Type, Svc Type, Svc ID, Meta Descriptors>
           * Where descriptors are:
           * - Gatt Usr String, Gatt Format Desc, Gatt Valid Range
           * - Hap Properties, Hap step value, Hap valid values, Hap valid Range
           */
          case HAP_OPCODE_CHR_SIGNATURE_READ:
          {

          }
          break;

          default: break;
        }
      }

      // Handle HAP Response
      if (request->type == BLE_GATTS_AUTHORIZE_TYPE_READ)
      {
        _rd_authorize_cb(*this, &request->request.read);
      }
    }
    break;

    default: break;
  }
}

