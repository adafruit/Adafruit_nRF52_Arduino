/**************************************************************************/
/*!
    @file     BLEDiscovery.cpp
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

#include "bluefruit.h"

#define BLE_CENTRAL_TIMEOUT     3000

BLEDiscovery::BLEDiscovery(void)
{
  _disc_hdl_range.start_handle = 1;
  _disc_hdl_range.end_handle   = 0xffff;

  _evt_sem     = NULL;
  _evt_buf     = NULL;
  _evt_bufsize = 0;
}

void BLEDiscovery::begin(void)
{
  if (_evt_sem == NULL)
  {
    _evt_sem = xSemaphoreCreateBinary();
  }
}

bool BLEDiscovery::begun(void)
{
  return (_evt_sem != NULL);
}

bool BLEDiscovery::discoverService(uint16_t conn_handle, BLECentralService& svc, uint16_t start_handle)
{
  ble_gattc_evt_prim_srvc_disc_rsp_t disc_svc;

  _evt_buf     = &disc_svc;
  _evt_bufsize = sizeof(disc_svc);

  VERIFY_STATUS( sd_ble_gattc_primary_services_discover(conn_handle, start_handle, &svc.uuid._uuid), false );

  // wait for discovery event: timeout or has no data
  if ( !xSemaphoreTake(_evt_sem, BLE_CENTRAL_TIMEOUT) || (_evt_bufsize == 0) ) return false;

  // Check the discovered UUID with input one
  if ( (disc_svc.count == 1) && (svc.uuid == disc_svc.services[0].uuid) )
  {
    _disc_hdl_range = disc_svc.services[0].handle_range;
    LOG_LV1(Discover, "[SVC] Found 0x%04X, Handle start = %d, end = %d", disc_svc.services[0].uuid.uuid, _disc_hdl_range.start_handle, _disc_hdl_range.end_handle);

    _disc_hdl_range.start_handle++; // increase for characteristic discovery
    return true;
  }

  return false;
}

uint8_t BLEDiscovery::discoverCharacteristic(uint16_t conn_handle, BLECentralCharacteristic* chr[], uint8_t count)
{
  uint8_t found = 0;

  while( found < count )
  {
    ble_gattc_evt_char_disc_rsp_t disc_chr;

    _evt_buf     = &disc_chr;
    _evt_bufsize = sizeof(disc_chr);

    LOG_LV1(Discover, "[CHR] Handle start = %d, end = %d", _disc_hdl_range.start_handle, _disc_hdl_range.end_handle);

    VERIFY_STATUS( sd_ble_gattc_characteristics_discover(conn_handle, &_disc_hdl_range), found );

    // wait for discovery event: timeout or has no data
    // Assume only 1 characteristic discovered each
    if ( !xSemaphoreTake(_evt_sem, BLE_CENTRAL_TIMEOUT) || (_evt_bufsize == 0) || (disc_chr.count == 0) ) break;

    // increase handle range for next discovery
    _disc_hdl_range.start_handle = disc_chr.chars[0].handle_value + 1;

    // Look for matched uuid
    for (uint8_t i=0; i<count; i++)
    {
      if ( chr[i]->uuid == disc_chr.chars[0].uuid )
      {
        LOG_LV1(Discover, "[CHR] Found 0x%04X, handle = %d", disc_chr.chars[0].uuid.uuid,  disc_chr.chars[0].handle_value);

        // characteristic assign overload
        chr[i]->assign(&disc_chr.chars[0]);

        // Discovery All descriptors as well
        chr[i]->discoverDescriptor(conn_handle);

        found++;

        break;
      }
    }
  }

  return found;
}

uint16_t BLEDiscovery::_discoverDescriptor(uint16_t conn_handle, ble_gattc_evt_desc_disc_rsp_t* disc_desc, uint16_t max_count)
{
  _evt_buf     = disc_desc;
  _evt_bufsize = sizeof(ble_gattc_evt_desc_disc_rsp_t) + (max_count-1)*sizeof(ble_gattc_desc_t);

  LOG_LV1(Discover, "[DESC] Handle start = %d, end = %d", _disc_hdl_range.start_handle, _disc_hdl_range.end_handle);

  uint16_t result = 0;
  VERIFY_STATUS( sd_ble_gattc_descriptors_discover(conn_handle, &_disc_hdl_range), 0 );

  // wait for discovery event: timeout or has no data
  if ( !xSemaphoreTake(_evt_sem, BLE_CENTRAL_TIMEOUT) || (_evt_bufsize == 0) ) return 0;

  result = min16(disc_desc->count, max_count);
  if (result)
  {
    for(uint16_t i=0; i<result; i++)
    {
      LOG_LV1(Discover, "[DESC] Descriptor %d: uuid = 0x%04X, handle = %d", i, disc_desc->descs[i].uuid.uuid, disc_desc->descs[i].handle);
    }

    // increase handle range for next discovery
    // should be +1 more, but that will cause missing on the next Characteristic !!!!!
    // Reason is descriptor also include BLE_UUID_CHARACTERISTIC 0x2803 (Char declaration) in the result
    _disc_hdl_range.start_handle = disc_desc->descs[result-1].handle;
  }

  return result;
}


void BLEDiscovery::_event_handler(ble_evt_t* evt)
{
  switch ( evt->header.evt_id  )
  {
    case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
    {
      ble_gattc_evt_t* gattc = &evt->evt.gattc_evt;
      ble_gattc_evt_prim_srvc_disc_rsp_t* svc_rsp = &gattc->params.prim_srvc_disc_rsp;

      LOG_LV1(Discover, "[SVC] Service Count: %d", svc_rsp->count);

      if (gattc->gatt_status == BLE_GATT_STATUS_SUCCESS && svc_rsp->count && _evt_buf)
      {
        // Only support 1 service

        _evt_bufsize = min16(_evt_bufsize, sizeof(ble_gattc_evt_prim_srvc_disc_rsp_t));
        memcpy(_evt_buf, svc_rsp, _evt_bufsize);
      }else
      {
        _evt_bufsize = 0; // no data
      }

      xSemaphoreGive(_evt_sem);
    }
    break;

    case BLE_GATTC_EVT_CHAR_DISC_RSP:
    {
      ble_gattc_evt_t* gattc = &evt->evt.gattc_evt;
      ble_gattc_evt_char_disc_rsp_t* chr_rsp = &gattc->params.char_disc_rsp;

      LOG_LV1(Discover, "[CHR] Characteristic Count: %d", chr_rsp->count);
      if ( (gattc->gatt_status == BLE_GATT_STATUS_SUCCESS) && chr_rsp->count && _evt_buf )
      {
        // TODO support only 1 discovered char now
        _evt_bufsize = min16(_evt_bufsize, sizeof(ble_gattc_evt_char_disc_rsp_t));

        memcpy(_evt_buf, chr_rsp, _evt_bufsize);
      }else
      {
        _evt_bufsize = 0; // no data
      }

      xSemaphoreGive(_evt_sem);
    }
    break;

    case BLE_GATTC_EVT_DESC_DISC_RSP:
    {
      ble_gattc_evt_t* gattc = &evt->evt.gattc_evt;
      ble_gattc_evt_desc_disc_rsp_t* desc_rsp = &gattc->params.desc_disc_rsp;

      LOG_LV1(Discover, "[DESC] Descriptor Count: %d", desc_rsp->count);

      if ( (gattc->gatt_status == BLE_GATT_STATUS_SUCCESS) && desc_rsp->count && _evt_buf )
      {
        // Copy up to bufsize
        uint16_t len = sizeof(ble_gattc_evt_desc_disc_rsp_t) + (desc_rsp->count-1)*sizeof(ble_gattc_desc_t);
        _evt_bufsize = min16(_evt_bufsize, len);

        memcpy(_evt_buf, desc_rsp, _evt_bufsize);
      }else
      {
        _evt_bufsize = 0; // no data
      }

      xSemaphoreGive(_evt_sem);
    }
    break;

    default: break;
  }
}


