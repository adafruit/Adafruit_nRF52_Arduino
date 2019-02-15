/**************************************************************************/
/*!
    @file     HAPCharacteristic.cpp
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
  _cid         = 0;
  _hap_props   = 0;

  _value       = NULL;
  _vallen      = 0;

  _hap_req     = NULL;
  _hap_reqlen  = 0;
  _tid         = 0;

  _hap_resp    = NULL;
  _hap_resplen = 0;
  _hap_resplen_sent = 0;

  _hap_wr_cb   = NULL;
  _hap_rd_cb   = NULL;

  // Need at least decent length for HAP Procedure
  _max_len     = 64;

  setPresentationFormatDescriptor(format, 0, unit, 1, 0);
}

void HAPCharacteristic::setHapProperties(uint16_t prop)
{
  _hap_props = prop;

  // HAP always require read and write
  uint8_t chrprop = CHR_PROPS_READ | CHR_PROPS_WRITE;

  if ((prop & HAP_CHR_PROPS_NOTIFY) || (prop & HAP_CHR_PROPS_NOTIFY_DISCONNECTED)) chrprop |= CHR_PROPS_INDICATE;
  setProperties(chrprop);

  setPermission(SECMODE_OPEN, SECMODE_OPEN);
}

void HAPCharacteristic::setHapWriteCallback(hap_write_cb_t fp)
{
  _hap_wr_cb = fp;
}

void HAPCharacteristic::setHapReadCallback(hap_read_cb_t fp)
{
  _hap_rd_cb = fp;
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

void HAPCharacteristic::createHapResponse(uint16_t conn_hdl, uint8_t status, TLV8_t tlv_para[], uint8_t count)
{
  BLEConnection* conn = Bluefruit.Connection( conn_hdl );

  // Determine body len (not including 2 byte length itself)
  uint16_t body_len = tlv8_encode_calculate_len(tlv_para, count);

  // Determine the response length including fragmentation scheme
  uint16_t const gatt_mtu = conn->getMtu() - 3;
  uint16_t resp_len = sizeof(HAPResponseHeader_t) + 2 + body_len;

  uint16_t nfrag = 0; // number of fragments (excluding the first)
  uint16_t const fraglen = gatt_mtu-2;

  // fragmentation required
  if (resp_len > gatt_mtu)
  {
    // calculate number of fragment. From 2nd frag, there is 2 extra byte (control + tid)
    nfrag = (resp_len-gatt_mtu) / fraglen;
    if ( (resp_len-gatt_mtu) % fraglen )   nfrag++;
  }

  HAPResponse_t* hap_resp = (HAPResponse_t*) rtos_malloc(resp_len + nfrag*2);
  VERIFY( hap_resp != NULL, );

  /*------------- Header -------------*/
  varclr(&hap_resp->header.control);
  hap_resp->header.control.fragment = 0;
  hap_resp->header.control.type     = HAP_PDU_RESPONSE;
  hap_resp->header.tid              = _tid;
  hap_resp->header.status           = status;
  hap_resp->body_len                = body_len;

  /*------------- Serialize Data -------------*/
  tlv8_encode_n(hap_resp->body_data, body_len, tlv_para, count);

  /*------------- Fragmentation -------------*/
  if (nfrag)
  {
    HAPControl_t ctrl = hap_resp->header.control;
    ctrl.fragment = 1;

    for(int i=nfrag-1; i >= 0; i--)
    {
      uint16_t const idx = gatt_mtu + i*fraglen;
      uint8_t* src = ((uint8_t*)hap_resp) + idx;
      uint8_t* dst = src + 2*i;

      // right shift 2 byte for fragment scheme
      uint16_t cc = (i == nfrag-1) ? (resp_len - idx) : fraglen;

      memmove(dst+2, src, cc);

      memcpy(&dst[0], &ctrl, 1);
      dst[1] = _tid;
    }

    resp_len += nfrag*2;
  }

  _hap_resp         = hap_resp;
  _hap_resplen      = resp_len;
  _hap_resplen_sent = 0;
}

void HAPCharacteristic::processChrSignatureRead(uint16_t conn_hdl, HAPRequest_t* hap_req)
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

  createHapResponse(conn_hdl, HAP_STATUS_SUCCESS, tlv_para, arrcount(tlv_para));
}

void HAPCharacteristic::processChrRead(uint16_t conn_hdl, HAPRequest_t* hap_req)
{
  TLV8_t tlv_para = { .type = HAP_PARAM_VALUE, .len = _vallen, .value = _value };

  createHapResponse(conn_hdl, HAP_STATUS_SUCCESS, &tlv_para, 1);
}

void HAPCharacteristic::processHapRequest(uint16_t conn_hdl, HAPRequest_t* hap_req)
{
  if (hap_req->header.control.type != HAP_PDU_REQUEST)
  {
    createHapResponse(conn_hdl, HAP_STATUS_UNSUPPORTED_PDU);
  }
  else if (hap_req->header.instance_id != _cid)
  {
    createHapResponse(conn_hdl, HAP_STATUS_INVALID_INSTANCE_ID);
  }else
  {
    LOG_LV2("HAP", "Recv %s request, TID = 0x%02X, CS_ID = 0x%04X", hap_opcode_str[hap_req->header.opcode], hap_req->header.tid, hap_req->header.instance_id);
    switch(hap_req->header.opcode)
    {
      case HAP_OPCODE_CHR_SIGNATURE_READ:
        processChrSignatureRead(conn_hdl, hap_req);
      break;

      case HAP_OPCODE_CHR_READ:
        processChrRead(conn_hdl, hap_req);
      break;

      case HAP_OPCODE_CHR_WRITE:
        if ( _hap_wr_cb )
        {
          _hap_wr_cb(conn_hdl, this, hap_req);
        }else
        {
          createHapResponse(conn_hdl, HAP_STATUS_UNSUPPORTED_PDU);
        }
      break;

      case HAP_OPCODE_CHR_TIMED_WRITE:
      break;

      case HAP_OPCODE_CHR_EXECUTE_WRITE:
      break;

      // need seperated chr for service
      //            case HAP_OPCODE_SVC_SIGNATURE_READ: break;

      default:
        createHapResponse(conn_hdl, HAP_STATUS_UNSUPPORTED_PDU);
      break;
    }
  }
}

void HAPCharacteristic::processGattWrite(uint16_t conn_hdl, ble_gatts_evt_write_t* gatt_req)
{
  LOG_LV2("GATTS", "Write Op = %d, len = %d, offset = %d, uuid = 0x%04X", gatt_req->op, gatt_req->len, gatt_req->offset, gatt_req->uuid.uuid);
  LOG_LV2_BUFFER(NULL, gatt_req->data, gatt_req->len);

  BLEConnection* conn = Bluefruit.Connection( conn_hdl );
  uint16_t const gatt_mtu = conn->getMtu() - 3;

  ble_gatts_rw_authorize_reply_params_t reply =
  {
      .type = BLE_GATTS_AUTHORIZE_TYPE_WRITE,
      .params = {
          .write = {
              .gatt_status = BLE_GATT_STATUS_SUCCESS,
              .update      = 1,
          }
      }
  };

  // new request
  if ( _hap_req == NULL )
  {
    uint16_t reqlen = sizeof(HAPRequestHeader_t);

    // if body is present
    if ( gatt_req->len > reqlen )
    {
      reqlen += 2 + ((HAPRequest_t*) gatt_req->data)->body_len;
    }

    // request len is larger than MTU --> buffer assembly required
    if ( reqlen > gatt_mtu )
    {
      _hap_req = (HAPRequest_t*) rtos_malloc( reqlen );
    }
  }

  // Assembly fragmentation request if needed
  if ( !_hap_req )
  {
    // no fragment
    _hap_req    = (HAPRequest_t*) gatt_req->data;
    _hap_reqlen = gatt_req->len;
  } else
  {
    // default to 1st fragment
    uint8_t* fragdata = gatt_req->data;
    uint16_t fraglen  = gatt_req->len;

    // 2nd and later Fragment
    if (gatt_req->data[0] & 0x80)
    {
      // match TID
      if ( _hap_req->header.tid == gatt_req->data[1] )
      {
        // copy data sip control & tid
        fragdata = gatt_req->data+2;
        fraglen  = gatt_req->len-2;
      }else
      {
        // Data corruption, unlikely to happen
        LOG_LV2("HAP", "Fragment TID not match");

        // clean up
        rtos_free(_hap_req);
        _hap_req    = NULL;
        _hap_reqlen = 0;

        // reject request
        reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_UNLIKELY_ERROR;
        VERIFY_STATUS( sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply), );
        return;
      }
    }

    // Assembly data
    memcpy(((uint8_t*)_hap_req) +_hap_reqlen, fragdata, fraglen);
    _hap_reqlen += fraglen;
  }

  // process when full request is received
  if ( (_hap_reqlen == sizeof(HAPRequestHeader_t)) ||
       (_hap_reqlen == _hap_req->body_len + sizeof(HAPRequestHeader_t) + 2) )
  {
    LOG_LV2_BUFFER("HAP Request", _hap_req, _hap_reqlen);

    // save tid
    _tid    = _hap_req->header.tid;

    processHapRequest(conn_hdl, _hap_req);

    if (_hap_resp )
    {
      LOG_LV2("HAP", "Response: Control = 0x%02X, TID = 0x%02X, Status = %d", _hap_resp->header.control, _hap_resp->header.tid, _hap_resp->header.status);
      LOG_LV2_BUFFER(NULL, _hap_resp, _hap_resplen);
    }

    // Free if needed
    if ( _hap_reqlen > gatt_mtu ) rtos_free(_hap_req);
    _hap_req    = NULL;
    _hap_reqlen = 0;
  }

  // Reply before processing request since cryptography take time and could cause timeout
  VERIFY_STATUS( sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply), );
}

void HAPCharacteristic::processGattRead(uint16_t conn_hdl, ble_gatts_evt_read_t* gatt_req)
{
  BLEConnection* conn = Bluefruit.Connection( conn_hdl );
  uint16_t const gatt_mtu = conn->getMtu() - 3;

  LOG_LV2("GATTS", "Read uuid = 0x%04X, offset = %d", gatt_req->uuid.uuid, gatt_req->offset);

  ble_gatts_rw_authorize_reply_params_t reply =
  {
      .type = BLE_GATTS_AUTHORIZE_TYPE_READ,
      .params = {
          .read = {
              .gatt_status = BLE_GATT_STATUS_SUCCESS,
              .update      = 1,
              .offset      = 0,
              .len         = 0,
              .p_data      = NULL
          }
      }
  };

  // If Response has not been generated, call the read callback
  if ( _hap_resp == NULL && _hap_resplen == 0 )
  {
    if ( _hap_rd_cb )
    {
      _hap_rd_cb(conn_hdl, this);
    }
  }

  if (_hap_resplen_sent < _hap_resplen)
  {
    uint16_t len = min16(_hap_resplen - _hap_resplen_sent, gatt_mtu);

    reply.params.read.len    = len;
    reply.params.read.p_data = ((uint8_t*)_hap_resp) + _hap_resplen_sent;

    LOG_LV2_BUFFER(NULL, reply.params.read.p_data, len);

    err_t err = sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);

    _hap_resplen_sent += len;

    if ( _hap_resplen == _hap_resplen_sent )
    {
      rtos_free(_hap_resp);
      _hap_resp = NULL;
      _hap_resplen = _hap_resplen_sent = 0;
    }

    VERIFY_STATUS(err, );
  }else
  {
    // reject read attempt
    reply.params.read.gatt_status = BLE_GATT_STATUS_ATTERR_READ_NOT_PERMITTED;
    VERIFY_STATUS( sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply), );
  }
}

void HAPCharacteristic::_eventHandler(ble_evt_t* event)
{
  const uint16_t conn_hdl = event->evt.common_evt.conn_handle;

  switch(event->header.evt_id)
  {
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
      // HAP Request
      if (event->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
      {
        processGattWrite(conn_hdl, &event->evt.gatts_evt.params.authorize_request.request.write);
      }

      // HAP Response
      if (event->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_READ)
      {
        processGattRead(conn_hdl,&event->evt.gatts_evt.params.authorize_request.request.read);
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
          _cccd_wr_cb(conn_hdl, this, value);
        }
      }
    }

    default: break;
  }
}
