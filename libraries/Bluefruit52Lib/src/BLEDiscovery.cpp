/**************************************************************************/
/*!
    @file     BLEDiscovery.cpp
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

BLEDiscovery::BLEDiscovery(void)
  : _adamsg()
{
  _hdl_range.start_handle = 1;
  _hdl_range.end_handle   = 0xffff;

  _begun = false;
}

void BLEDiscovery::begin(void)
{
  if ( !_begun )
  {
    _adamsg.begin(false);
    _begun = true;
  }
}

bool BLEDiscovery::begun(void)
{
  return _begun;
}

void BLEDiscovery::setHandleRange(ble_gattc_handle_range_t handle_range)
{
  _hdl_range = handle_range;
}

ble_gattc_handle_range_t BLEDiscovery::getHandleRange(void)
{
  return _hdl_range;
}

bool BLEDiscovery::_discoverService(uint16_t conn_handle, BLEClientService& svc, uint16_t start_handle)
{
  ble_gattc_evt_prim_srvc_disc_rsp_t disc_svc;

  LOG_LV2("DISC", "[SVC] Handle start = %d", start_handle);

  _adamsg.prepare(&disc_svc, sizeof(disc_svc));
  VERIFY_STATUS( sd_ble_gattc_primary_services_discover(conn_handle, start_handle, &svc.uuid._uuid), false );

  // wait for discovery event
  int32_t bytecount = _adamsg.waitUntilComplete(BLE_DISCOVERY_TIMEOUT);

  // timeout or has no data (due to GATT Error)
  if ( bytecount <= 0 )
  {
    LOG_LV1("DISC", "[SVC] timeout or error %ud", start_handle);
    return false;
  }

  // Check the discovered UUID with input one
  if ( (disc_svc.count) && (svc.uuid == disc_svc.services[0].uuid) )
  {
    _hdl_range = disc_svc.services[0].handle_range;
    svc.setHandleRange(_hdl_range);

    LOG_LV2("DISC", "[SVC] Found 0x%04X, Handle start = %d, end = %d\n-----------------", disc_svc.services[0].uuid.uuid, _hdl_range.start_handle, _hdl_range.end_handle);

    // increase for next discovery
    _hdl_range.start_handle++;
    return true;
  }

  return false;
}

uint8_t BLEDiscovery::discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic* chr[], uint8_t count)
{
  // We could found more characteristic than we looking for. Buffer must be large enough
  enum { MAX_DISC_CHARS = 8 };

  // -1 because the first ble_gattc_char_t is built in to ble_gattc_evt_char_disc_rsp_t
  uint16_t bufsize = sizeof(ble_gattc_evt_char_disc_rsp_t) + (MAX_DISC_CHARS-1)*sizeof(ble_gattc_char_t); 
  ble_gattc_evt_char_disc_rsp_t* disc_chr = (ble_gattc_evt_char_disc_rsp_t*) rtos_malloc( bufsize );

  uint8_t found = 0;

  while( found < count )
  {
    LOG_LV2("DISC", "[CHR] Handle start = %d, end = %d", _hdl_range.start_handle, _hdl_range.end_handle);

    memclr(disc_chr, bufsize);
    _adamsg.prepare(disc_chr, bufsize);

    if( ERROR_NONE != sd_ble_gattc_characteristics_discover(conn_handle, &_hdl_range) ) break;

    // wait for discovery event
    int32_t bytecount = _adamsg.waitUntilComplete(BLE_DISCOVERY_TIMEOUT);

    // timeout or has no data (due to GATT Error)
    if ( bytecount <= 0 ) break;

    // if we truncated the response, adjust the count to match
    if ( disc_chr->count > MAX_DISC_CHARS ) disc_chr->count = MAX_DISC_CHARS;

    // Look for matched uuid in the discovered list
    for(uint8_t d=0 ; d<disc_chr->count; d++)
    {
      for (uint8_t i=0; i<count; i++)
      {
        // Skip if output chr is already discovered, happens with multiple instances of same UUIDs
        if ( chr[i]->discovered() ) continue;

        if ( chr[i]->uuid == disc_chr->chars[d].uuid )
        {
          LOG_LV2("DISC", "[CHR] Found 0x%04X, handle = %d\n-----------------", disc_chr->chars[d].uuid.uuid,  disc_chr->chars[d].handle_value);

          // characteristic assign overload
          chr[i]->_assign(&disc_chr->chars[d]);

          // only discover CCCD descriptor
          if (disc_chr->chars[d].char_props.notify || disc_chr->chars[d].char_props.indicate )
          {
            ble_gattc_handle_range_t range = {
              .start_handle = (uint16_t) (disc_chr->chars[d].handle_value + 1),
              .end_handle   = _hdl_range.end_handle
            };

            if ( range.start_handle <= range.end_handle  )
            {
              // skip if reaching end of range (last char has no descriptor)p
              chr[i]->_discoverDescriptor(conn_handle, range);
            }
          }

          found++;

          break;
        }
      }
    }

    // increase handle range for next discovery
    // should be last descriptor +1, but that will cause missing on the next Characteristic !!!!!
    // Reason is descriptor also include BLE_UUID_CHARACTERISTIC 0x2803 (Char declaration) in the result
    //
    // To be safe we use last chars + 1
    _hdl_range.start_handle = disc_chr->chars[ disc_chr->count-1  ].handle_value + 1;
  }

  rtos_free(disc_chr);

  return found;
}

uint16_t BLEDiscovery::_discoverDescriptor(uint16_t conn_handle, ble_gattc_evt_desc_disc_rsp_t* disc_desc, uint16_t bufsize, ble_gattc_handle_range_t hdl_range)
{
  LOG_LV2("DISC", "[DESC] Handle start = %d, end = %d", hdl_range.start_handle, hdl_range.end_handle);

  _adamsg.prepare(disc_desc, bufsize);

  VERIFY_STATUS( sd_ble_gattc_descriptors_discover(conn_handle, &hdl_range), 0 );

  // wait for discovery event
  int32_t bytecount = _adamsg.waitUntilComplete(BLE_DISCOVERY_TIMEOUT);

  // timeout or has no data (due to GATT Error)
  if ( bytecount <= 0 ) return 0;

  for(uint16_t i=0; i<disc_desc->count; i++)
  {
    LOG_LV2("DISC", "[DESC] Descriptor %d: uuid = 0x%04X, handle = %d", i, disc_desc->descs[i].uuid.uuid, disc_desc->descs[i].handle);
  }

  return disc_desc->count;
}


void BLEDiscovery::_eventHandler(ble_evt_t* evt)
{
  ble_gattc_evt_t* gattc = &evt->evt.gattc_evt;

  switch ( evt->header.evt_id  )
  {
    case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
    {
      ble_gattc_evt_prim_srvc_disc_rsp_t* svc_rsp = &gattc->params.prim_srvc_disc_rsp;

      LOG_LV2("DISC", "[SVC] Service Count: %d", svc_rsp->count);

      if (gattc->gatt_status == BLE_GATT_STATUS_SUCCESS)
      {
        // Only 1 service at a time
        if (svc_rsp->count)
        {
          _adamsg.feed(svc_rsp, sizeof(ble_gattc_evt_prim_srvc_disc_rsp_t));
        }
      }else
      {
        LOG_LV1("DISC", "[SVC] Gatt Status = 0x%04X", gattc->gatt_status);
      }

      _adamsg.complete();
    }
    break;

    case BLE_GATTC_EVT_CHAR_DISC_RSP:
    {
      ble_gattc_evt_char_disc_rsp_t* chr_rsp = &gattc->params.char_disc_rsp;

      LOG_LV2("DISC", "[CHR] Characteristic Count: %d", chr_rsp->count);

      if (gattc->gatt_status == BLE_GATT_STATUS_SUCCESS)
      {
        if ( chr_rsp->count )
        {
          uint16_t len = sizeof(ble_gattc_evt_char_disc_rsp_t) + (chr_rsp->count-1)*sizeof(ble_gattc_char_t);
          _adamsg.feed(chr_rsp, len);
        }
      }else
      {
        LOG_LV1("DISC", "[CHR] Gatt Status = 0x%04X", gattc->gatt_status);
      }

      _adamsg.complete();
    }
    break;

    case BLE_GATTC_EVT_DESC_DISC_RSP:
    {
      ble_gattc_evt_desc_disc_rsp_t* desc_rsp = &gattc->params.desc_disc_rsp;

      LOG_LV2("DISC", "[DESC] Descriptor Count: %d", desc_rsp->count);

      if (gattc->gatt_status == BLE_GATT_STATUS_SUCCESS)
      {
        if ( desc_rsp->count )
        {
          uint16_t len = sizeof(ble_gattc_evt_desc_disc_rsp_t) + (desc_rsp->count-1)*sizeof(ble_gattc_desc_t);
          _adamsg.feed(desc_rsp, len);
        }
      }else
      {
        LOG_LV1("DISC", "[DESC] Gatt Status = 0x%04X", gattc->gatt_status);
      }

      _adamsg.complete();
    }
    break;

    default: break;
  }
}


