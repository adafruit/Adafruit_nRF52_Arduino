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

#if CFG_DEBUG >= 2
static const char* hap_opcode_str[] =
{
    "", "Signature Read", "Write", "Read", "Timed Write", "Execute Write", "Service Signature Read"
};
#endif

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
  _cid       = 0;
  _hap_props = 0;

  _value     = NULL;
  _vallen    = 0;

  _resp_len  = 0;

  _hap_wr_cb = NULL;

  // Need at least decent length for HAP Procedure
  _max_len = 100;

  setPresentationFormatDescriptor(format, 0, unit, 1, 0);
}

void HAPCharacteristic::setHapProperties(uint16_t prop)
{
  _hap_props = prop;

  uint8_t chrprop = CHR_PROPS_READ | CHR_PROPS_WRITE;
  if ((prop & HAP_CHR_PROPS_NOTIFY) || (prop & HAP_CHR_PROPS_NOTIFY_DISCONNECTED)) chrprop |= CHR_PROPS_INDICATE;
  setProperties(chrprop);

  setPermission(SECMODE_OPEN, SECMODE_OPEN);
}

void HAPCharacteristic::setHapWriteCallback(hap_write_cb_t fp)
{
  _hap_wr_cb = fp;
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

  // FIXME temporary
  _max_len = minof(_max_len, 100);

  VERIFY_STATUS( BLECharacteristic::begin() );
  VERIFY_STATUS( _addChrIdDescriptor() );

  // Restore configured setting
  _format_desc.format = temp_format;
  _usr_descriptor = temp_usr;

  LOG_LV2("HAP", "Added Char UUID=0x%04X, CID=0x%04X", uuid._uuid.uuid, _cid);

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

uint16_t HAPCharacteristic::writeHapValue(const void* data, uint16_t len)
{
  free(_value);

  _vallen = len;
  _value  = rtos_malloc(len);
  VERIFY(_value, 0);

  memcpy(_value, data, len);

  return len;
}

uint16_t HAPCharacteristic::writeHapValue(const char* str)
{
  return writeHapValue((const void*) str, strlen(str));
}

uint16_t HAPCharacteristic::writeHapValue(uint32_t num)
{
  uint8_t len;

  // determine len by format type
  switch ( _format_desc.format )
  {
    case BLE_GATT_CPF_FORMAT_UINT8:
    case BLE_GATT_CPF_FORMAT_BOOLEAN:
      len = 1;
    break;

    case BLE_GATT_CPF_FORMAT_UINT16:
      len = 2;
    break;

    case BLE_GATT_CPF_FORMAT_UINT32:
      len = 4;
    break;

    default: len = 0; break;
  }

  VERIFY(len, 0);

  return writeHapValue(&num, len);
}

HAPResponse_t* HAPCharacteristic::createHapResponse(uint8_t tid, uint8_t status, TLV8_t tlv_para[], uint8_t count)
{
  // Determine body len, does not to include 2 byte length itself
  uint8_t body_len = 0;
  for(uint8_t i=0; i <count ; i++)
  {
    body_len += tlv_para[i].len + 2;
  }

  HAPResponse_t* hap_resp = (HAPResponse_t*) rtos_malloc(sizeof(HAPResponseHeader_t) + 2 + body_len);
  VERIFY( hap_resp != NULL, NULL );

  /*------------- Header -------------*/
  varclr(&hap_resp->header.control);
  hap_resp->header.control.fragment = 0;
  hap_resp->header.control.type     = HAP_PDU_RESPONSE;
  hap_resp->header.tid              = tid;
  hap_resp->header.status           = status;
  hap_resp->body_len                = body_len;

  /*------------- Serialize Data -------------*/
  uint8_t* pdata = hap_resp->body_data;
  for(uint8_t i=0; i < count; i++)
  {
    memcpy(pdata, &tlv_para[i], 2); // type + len
    pdata += 2;

    memcpy(pdata, tlv_para[i].value, tlv_para[i].len);
    pdata += tlv_para[i].len;
  }

  return hap_resp;
}

HAPResponse_t* HAPCharacteristic::processChrSignatureRead(HAPRequest_t* hap_req)
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

  /* Return <Chr Type, Chr ID, Svc Type, Svc ID, Meta Descriptors>
   * Where descriptors are:
   * - Gatt Usr String, Gatt Format Desc, Gatt Valid Range
   * - Hap Properties, Hap step value, Hap valid values, Hap valid Range
   */
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

  return createHapResponse(hap_req->header.tid, HAP_STATUS_SUCCESS, tlv_para, arrcount(tlv_para));
}

HAPResponse_t* HAPCharacteristic::processChrRead(HAPRequest_t* hap_req)
{
  TLV8_t tlv_para = { .type = HAP_PARAM_VALUE, .len = _vallen, .value = _value };

  return createHapResponse(hap_req->header.tid, HAP_STATUS_SUCCESS, &tlv_para, 1);
}

void HAPCharacteristic::_eventHandler(ble_evt_t* event)
{
  const uint16_t conn_hdl = event->evt.common_evt.conn_handle;

  switch(event->header.evt_id)
  {
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
      /*------------- Handle HAP Request -------------*/
      if (event->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
      {
        ble_gatts_evt_write_t * gatt_req = &event->evt.gatts_evt.params.authorize_request.request.write;

        LOG_LV2("GATTS", "Write Op = %d, uuid = 0x%04X", gatt_req->op, gatt_req->uuid.uuid);
        LOG_LV2_BUFFER(NULL, gatt_req->data, gatt_req->len);

        HAPRequest_t*  hap_req = (HAPRequest_t*) gatt_req->data;
        HAPResponse_t* hap_resp = NULL;

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

        if (hap_req->header.control.type != HAP_PDU_REQUEST)
        {
          hap_resp = createHapResponse(hap_req->header.tid, HAP_STATUS_UNSUPPORTED_PDU);
        }
        else if (hap_req->header.instance_id != _cid)
        {
          hap_resp = createHapResponse(hap_req->header.tid, HAP_STATUS_INVALID_INSTANCE_ID);
        }else
        {
          LOG_LV2("HAP", "Recv %s request", hap_opcode_str[hap_req->header.opcode]);
          switch(hap_req->header.opcode)
          {
            case HAP_OPCODE_CHR_SIGNATURE_READ:
              hap_resp = processChrSignatureRead(hap_req);
            break;

            case HAP_OPCODE_CHR_WRITE:
              if (_hap_wr_cb)
              {
                hap_resp = _hap_wr_cb(this, gatt_req, hap_req);
              }
            break;

            case HAP_OPCODE_CHR_READ:
              hap_resp = processChrRead(hap_req);
            break;

            case HAP_OPCODE_CHR_TIMED_WRITE:
            break;

            case HAP_OPCODE_CHR_EXECUTE_WRITE:
            break;

            // need seperated chr for service
//            case HAP_OPCODE_SVC_SIGNATURE_READ: break;

            default:
              hap_resp = createHapResponse(hap_req->header.tid, HAP_STATUS_UNSUPPORTED_PDU);
            break;
          }
        }

        if ( hap_resp )
        {
          reply.params.write.len    = _resp_len = sizeof(HAPResponseHeader_t) + 2 + hap_resp->body_len;
          reply.params.write.p_data = (uint8_t*) hap_resp;
        }else
        {
          _resp_len = 0;
          reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_INSUF_RESOURCES;
        }

        LOG_LV2("HAP", "Response Data");
        LOG_LV2_BUFFER(NULL, hap_resp, reply.params.write.len);
        err_t err = sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);

        rtos_free(hap_resp);
        VERIFY_STATUS( err, );
      }

      /*------------- Handle HAP Response -------------*/
      if (event->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_READ)
      {
        ble_gatts_evt_read_t * gatt_req = &event->evt.gatts_evt.params.authorize_request.request.read;

        LOG_LV2("GATTS", "Read uuid = 0x%04X, offset = %d", gatt_req->uuid.uuid, gatt_req->offset);

        ble_gatts_rw_authorize_reply_params_t reply =
        {
            .type = BLE_GATTS_AUTHORIZE_TYPE_READ,
            .params = {
                .read = {
                    .gatt_status = BLE_GATT_STATUS_SUCCESS,
                    .update      = 0,
                    .offset      = 0,
                    .len         = 0,
                    .p_data      = NULL
                }
            }
        };

        if (_resp_len)
        {
          _resp_len -= min16(_resp_len, Bluefruit.Gap.getMTU(conn_hdl)-2);
          VERIFY_STATUS( sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply), );
        }else
        {
          // reject read attempt
          reply.params.read.gatt_status = BLE_GATT_STATUS_ATTERR_READ_NOT_PERMITTED;
          VERIFY_STATUS( sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply), );
        }
      }
    }
    break;

    case BLE_GATTS_EVT_WRITE:
    {
      ble_gatts_evt_write_t* request = &event->evt.gatts_evt.params.write;

      // CCCD write
      if ( request->handle == _handles.cccd_handle )
      {
        LOG_LV2("GATTS", "attr's cccd");
        LOG_LV2_BUFFER(NULL, request->data, request->len);

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
          ada_callback(NULL, _adafruit_save_bond_cccd_dfr, conn_hdl);
        }
      }
    }

    default: break;
  }
}
