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

  _hap_resp = NULL;

  setPresentationFormatDescriptor(format, 0, unit, 1, 0);
}

void HAPCharacteristic::setHapProperties(uint16_t prop)
{
  _hap_props = prop;

  uint8_t chrprop = CHR_PROPS_READ | CHR_PROPS_WRITE;
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
  _attr_meta.rd_auth = 1;
  _attr_meta.wr_auth = 1;

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

void HAPCharacteristic::_eventHandler(ble_evt_t* event)
{
  switch(event->header.evt_id)
  {
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
      /*------------- Handle HAP Request -------------*/
      if (event->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
      {
        ble_gatts_evt_write_t * gatt_req = &event->evt.gatts_evt.params.authorize_request.request.write;

        LOG_LV2(GATTS, "attr's value, uuid = 0x%04X", gatt_req->uuid.uuid);
        PRINT2_BUFFER(gatt_req->data, gatt_req->len);

        HAPRequest_t* hap_req = (HAPRequest_t*) gatt_req->data;
        VERIFY(hap_req->header.control.type == HAP_PDU_REQUEST, );

//        PRINT_INT(hap_req->header.opcode);

        ble_gatts_rw_authorize_reply_params_t reply =
        {
            .type = BLE_GATTS_AUTHORIZE_TYPE_WRITE,
            .params = {
                .write = {
                    .gatt_status = BLE_GATT_STATUS_SUCCESS,
                    .update      = 1,
                    .offset      = 0,
                    .len         = 0,
                    .p_data      = NULL
                }
            }
        };

        switch(hap_req->header.opcode)
        {
          /* Return <Chr Type, Svc Type, Svc ID, Meta Descriptors>
           * Where descriptors are:
           * - Gatt Usr String, Gatt Format Desc, Gatt Valid Range
           * - Hap Properties, Hap step value, Hap valid values, Hap valid Range
           */
          case HAP_OPCODE_CHR_SIGNATURE_READ:
          {
            uint16_t svc_id = ((HAPService*)_service)->getSvcId();

            // ble_gatts_char_pf_t is not packed struct
            struct ATTR_PACKED
            {
              uint8_t          format;
              int8_t           exponent;
              uint16_t         unit;
              uint8_t          name_space;
              uint16_t         desc;
            }fmt_desc =
            {
                .format     = _format_desc.format,
                .exponent   = _format_desc.exponent,
                .unit       = _format_desc.unit,
                .name_space = _format_desc.name_space,
                .desc       = _format_desc.desc
            };

            TLV8_t tlv_para[] =
            {
                { .type  = HAP_PARAM_CHR_TYPE, .len = 16, .value = uuid._uuid128           },
                { .type  = HAP_PARAM_CHR_ID  , .len = 2 , .value = &_cid                   },
                { .type  = HAP_PARAM_SVC_TYPE, .len = 16, .value = _service->uuid._uuid128 },
                { .type  = HAP_PARAM_SVC_ID  , .len = 2 , .value = &svc_id                 },
                // Descriptors
                { .type  = HAP_PARAM_HAP_CHR_PROPERTIES_DESC, .len = 2 , .value = &_hap_props  },
                { .type  = HAP_PARAM_GATT_FORMAT_DESC       , .len = sizeof(fmt_desc) , .value = &fmt_desc  },
            };

            // Determine body len
            uint8_t body_len = 2;
            for(uint8_t i=0; i < arrcount(tlv_para); i++)
            {
              body_len += tlv_para[i].len + 2;
            }

            // Free if in used (a previous Response Read is skipped)
            if ( _hap_resp ) rtos_free(_hap_resp);

            _hap_resp = (HAPResponse_t*) rtos_malloc(sizeof(HAPResponseHeader_t) + body_len);
            VERIFY( _hap_resp != NULL, );

            /*------------- Header -------------*/
            varclr(&_hap_resp->header.control);
            _hap_resp->header.control.fragment = 0;
            _hap_resp->header.control.type     = HAP_PDU_RESPONSE;

            _hap_resp->header.tid    = hap_req->header.tid;
            _hap_resp->header.status = HAP_STATUS_SUCCESS;
            _hap_resp->body_len      = body_len;

            /*------------- Serialize Data -------------*/
            uint8_t* pdata = _hap_resp->body_data;
            for(uint8_t i=0; i < arrcount(tlv_para); i++)
            {
              memcpy(pdata, &tlv_para[i], 2); // type + len
              pdata += 2;

              memcpy(pdata, tlv_para[i].value, tlv_para[i].len);
              pdata += tlv_para[i].len;
            }
          }
          break;

          default:
//            reply.params.write.gatt_status = BLE_GATT_STATUS_UNKNOWN;
          break;
        }

        VERIFY_STATUS( sd_ble_gatts_rw_authorize_reply(Bluefruit.connHandle(), &reply), );
      }

      /*------------- Handle HAP Response -------------*/
      if (event->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_READ)
      {
        ble_gatts_evt_read_t * gatt_req = &event->evt.gatts_evt.params.authorize_request.request.read;

        LOG_LV2(GATTS, "attr's value, uuid = 0x%04X", gatt_req->uuid.uuid);

        ble_gatts_rw_authorize_reply_params_t reply =
        {
            .type = BLE_GATTS_AUTHORIZE_TYPE_READ,
            .params = {
                .read = {
                    .gatt_status = BLE_GATT_STATUS_ATTERR_READ_NOT_PERMITTED,
                    .update      = 0,
                    .offset      = 0,
                    .len         = 0,
                    .p_data      = NULL
                }
            }
        };

        if ( _hap_resp )
        {
          reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
          reply.params.read.len         = _hap_resp->body_len + sizeof(HAPResponseHeader_t) ;
          reply.params.read.p_data      = (uint8_t*) _hap_resp;
        }

        PRINT2_BUFFER(reply.params.read.p_data, reply.params.read.len);

        VERIFY_STATUS( sd_ble_gatts_rw_authorize_reply(Bluefruit.connHandle(), &reply), );

        if ( _hap_resp )
        {
          rtos_free(_hap_resp);
          _hap_resp = NULL;
        }
      }
    }
    break;

    case BLE_GATTS_EVT_WRITE:
    {
      ble_gatts_evt_write_t* request = &event->evt.gatts_evt.params.write;

      // Value write
      if (request->handle == _handles.value_handle)
      {
        LOG_LV2(GATTS, "attr's value, uuid = 0x%04X", request->uuid.uuid);
        PRINT2_BUFFER(request->data, request->len);

        // TODO Ada callback
        if (_wr_cb) _wr_cb(*this, request->data, request->len, request->offset);
      }

      // CCCD write
      if ( request->handle == _handles.cccd_handle )
      {
        LOG_LV2(GATTS, "attr's cccd");
        PRINT2_BUFFER(request->data, request->len);

        // Invoke callback if set
        if (_cccd_wr_cb)
        {
          uint16_t value;
          memcpy(&value, request->data, 2);
          _cccd_wr_cb(*this, value);
        }

        // TODO could move later
        // Save CCCD if bonded to file whenever changed
        extern void _adafruit_save_bond_cccd_dfr(uint32_t conn_handle);
        if ( Bluefruit.connPaired() )
        {
          ada_callback(NULL, _adafruit_save_bond_cccd_dfr, event->evt.common_evt.conn_handle);
        }
      }
    }

    default: break;
  }
}
