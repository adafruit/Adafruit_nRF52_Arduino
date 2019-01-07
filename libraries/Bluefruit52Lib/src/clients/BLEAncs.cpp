/**************************************************************************/
/*!
    @file     BLEAncs.cpp
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

#define BLE_ANCS_TIMEOUT   (5*BLE_GENERIC_TIMEOUT)

#define DEBUG_ANCS    0

void bleancs_notification_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
void bleancs_data_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);

/* ANCS Service        : 7905F431-B5CE-4E99-A40F-4B1E122D00D0
 * Control Point       : 69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9
 * Notification Source : 9FBF120D-6301-42D9-8C58-25E699A21DBD
 * Data Source         : 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB
 */

const uint8_t BLEANCS_UUID_SERVICE[] =
{
    0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4,
    0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79
};

const uint8_t BLEANCS_UUID_CHR_CONTROL[] =
{
    0xD9, 0xD9, 0xAA, 0xFD, 0xBD, 0x9B, 0x21, 0x98,
    0xA8, 0x49, 0xE1, 0x45, 0xF3, 0xD8, 0xD1, 0x69
};

const uint8_t BLEANCS_UUID_CHR_NOTIFICATION[]
{
    0xBD, 0x1D, 0xA2, 0x99, 0xE6, 0x25, 0x58, 0x8C,
    0xD9, 0x42, 0x01, 0x63, 0x0D, 0x12, 0xBF, 0x9F
};

const uint8_t BLEANCS_UUID_CHR_DATA[] =
{
    0xFB, 0x7B, 0x7C, 0xCE, 0x6A, 0xB3, 0x44, 0xBE,
    0xB5, 0x4B, 0xD6, 0x24, 0xE9, 0xC6, 0xEA, 0x22
};

BLEAncs::BLEAncs(void)
  : BLEClientService(BLEANCS_UUID_SERVICE), _control(BLEANCS_UUID_CHR_CONTROL),
    _notification(BLEANCS_UUID_CHR_NOTIFICATION), _data(BLEANCS_UUID_CHR_DATA),
    _adamsg()
{
  _notif_cb = NULL;
}

bool BLEAncs::begin(void)
{
  // Invoke base class begin()
  BLEClientService::begin();

  _adamsg.begin(false);

  _control.begin();
  _notification.begin();
  _data.begin();

  _notification.setNotifyCallback(bleancs_notification_cb);

  // Data Attribute is most likely requested in notification callback
  // let's call data's callback in the ble task
  _data.setNotifyCallback(bleancs_data_cb, false);

  return true;
}

bool BLEAncs::discover(uint16_t conn_handle)
{
  // Call BLECentralService discover
  VERIFY( BLEClientService::discover(conn_handle) );
  _conn_hdl = BLE_CONN_HANDLE_INVALID; // make as invalid until we found all chars

  // Discover characteristics
  BLEClientCharacteristic* chr_arr[] = { &_control, &_notification, &_data };

  VERIFY( 3 == Bluefruit.Discovery.discoverCharacteristic(conn_handle, chr_arr, 3) );

  _conn_hdl = conn_handle;
  return true;
}

void BLEAncs::setNotificationCallback(notification_callback_t fp)
{
  _notif_cb = fp;
}

bool BLEAncs::enableNotification(void)
{
  // enable both Notification & Data Source
  VERIFY ( _data.enableNotify() );
  VERIFY ( _notification.enableNotify() );

  return true;
}

bool BLEAncs::disableNotification(void)
{
  _notification.disableNotify();
  _data.disableNotify();

  return true;
}

/*------------------------------------------------------------------*/
/* NOTIFICATION
 *------------------------------------------------------------------*/
typedef struct ATTR_PACKED
{
  // Cortex M4 can access unaligned memory
  uint8_t  cmd;
  uint32_t uid;
  uint8_t  attr;
  uint16_t len; // optional when sending command
}get_notif_attr_t;

VERIFY_STATIC( sizeof(get_notif_attr_t) == 8);

typedef struct ATTR_PACKED
{
  // Cortex M4 can access unaligned memory
  uint8_t  cmd;
  uint32_t uid;
  uint8_t  actionid;
}perform_action_t;

VERIFY_STATIC( sizeof(perform_action_t) == 6);

/*------------------------------------------------------------------*/
/*
 *------------------------------------------------------------------*/
uint16_t BLEAncs::getAttribute(uint32_t uid, uint8_t attr, void* buffer, uint16_t bufsize)
{
  VERIFY ( attr < ANCS_ATTR_INVALID, 0);

  // command ID | uid | attr (+ len)
  get_notif_attr_t command =
  {
      .cmd  = ANCS_CMD_GET_NOTIFICATION_ATTR,
      .uid  = uid,
      .attr = attr,
      .len  = bufsize
  };
  uint8_t cmdlen = 6;

  // Title, Subtitle, Message must require 2-byte length
  if (attr == ANCS_ATTR_TITLE || attr == ANCS_ATTR_SUBTITLE || attr == ANCS_ATTR_MESSAGE)
  {
    cmdlen = 8;
  }

  // Write command using write response
#if DEBUG_ANCS
   PRINT_BUFFER(&command, cmdlen);
#endif

  _adamsg.prepare(buffer, bufsize);
  VERIFY( cmdlen == _control.write_resp(&command, cmdlen), 0);
  VERIFY( _adamsg.waitUntilComplete(BLE_ANCS_TIMEOUT) >= 0, 0);

  // At least 1 packet arrived, enough to know attribute length
  uint16_t attr_len = ((get_notif_attr_t*) buffer)->len;

  // wait until all data received Or we run out of memory
  while ( ( (attr_len + sizeof(get_notif_attr_t))  > _adamsg.xferlen ) &&
          ( _adamsg.remaining > 0 ) )
  {
    VERIFY( _adamsg.waitUntilComplete(BLE_ANCS_TIMEOUT) >= 0, 0);
  }

  // Received length could be less if we run out of buffer
  attr_len = _adamsg.xferlen - sizeof(get_notif_attr_t);

  // Shift out the Command data, left only Attribute data
  memmove(buffer, ((uint8_t*)buffer) +sizeof(get_notif_attr_t), attr_len);

  // Include null-terminator for some string application
  if ( attr_len < bufsize )
  {
    ((char*) buffer)[attr_len] = 0;
  }

  return attr_len;

}

uint16_t BLEAncs::getAppAttribute(const char* appid, uint8_t attr, void* buffer, uint16_t bufsize)
{
  VERIFY ( attr < ANCS_APP_ATTR_INVALID, 0);

  // command ID | App ID (including Null terminator) | Attr
  uint8_t cmdlen = 1 + strlen(appid)+1 + 1;
  uint8_t* command = (uint8_t*) rtos_malloc( cmdlen );

  command[0] = ANCS_CMD_GET_APP_ATTR;
  strcpy( (char*) command+1, appid);
  command[cmdlen-1] = attr;

#if DEBUG_ANCS
  PRINT_BUFFER(command, cmdlen);
#endif
  _adamsg.prepare(buffer, bufsize);

  // Write command using write response
  if ( cmdlen != _control.write_resp(command, cmdlen) )
  {
    rtos_free(command);
    return 0;
  }
  rtos_free(command);

  // Phase 1: Get data until Attribute Length is known
  while ( (cmdlen+2) > _adamsg.xferlen &&
          (_adamsg.remaining > 0) )
  {
    VERIFY( _adamsg.waitUntilComplete(BLE_ANCS_TIMEOUT) >= 0, 0);
  }

  uint16_t attr_len;
  memcpy(&attr_len, ((uint8_t*)buffer)+cmdlen, 2);

  // Phase 2: Get data until all attribute data received
  // Or we run out of memory
  while ( (attr_len + cmdlen+2) > _adamsg.xferlen &&
          (_adamsg.remaining > 0) )
  {
    VERIFY( _adamsg.waitUntilComplete(BLE_ANCS_TIMEOUT) >= 0, 0);
  }

  // Received length could be less if we run out of buffer
  attr_len = _adamsg.xferlen - (cmdlen+2);

  // Shift out the Command data, left only Attribute data
  memmove(buffer, ((uint8_t*)buffer) +cmdlen+2, attr_len);

  // including null-terminator for some string application
  if ( attr_len < bufsize )
  {
    ((char*) buffer)[attr_len] = 0;
  }

  return attr_len;
}

bool BLEAncs::performAction(uint32_t uid, uint8_t actionid)
{
  perform_action_t action =
  {
      .cmd      = ANCS_CMD_PERFORM_NOTIFICATION_ACTION,
      .uid      = uid,
      .actionid = actionid
  };

  return sizeof(perform_action_t) == _control.write_resp(&action, sizeof(perform_action_t));
}

void BLEAncs::_handleNotification(uint8_t* data, uint16_t len)
{
  if ( len != 8  ) return;
  if ( _notif_cb ) _notif_cb((AncsNotification_t*) data);
}

void BLEAncs::_handleData(uint8_t* data, uint16_t len)
{
#if DEBUG_ANCS
  PRINT_BUFFER(data, len);
#endif

  _adamsg.feed(data, len);
  _adamsg.complete(); // mark as complete each time we received data
}

/*------------------------------------------------------------------*/
/* High Level API
 *------------------------------------------------------------------*/
uint16_t BLEAncs::getAppID(uint32_t uid, void* buffer, uint16_t bufsize)
{
  return getAttribute(uid, ANCS_ATTR_APP_IDENTIFIER, buffer, bufsize);
}

uint16_t BLEAncs::getTitle(uint32_t uid, void* buffer, uint16_t bufsize)
{
  return getAttribute(uid, ANCS_ATTR_TITLE, buffer, bufsize);
}

uint16_t BLEAncs::getSubtitle(uint32_t uid, void* buffer, uint16_t bufsize)
{
  return getAttribute(uid, ANCS_ATTR_SUBTITLE, buffer, bufsize);
}

uint16_t BLEAncs::getMessage(uint32_t uid, void* buffer, uint16_t bufsize)
{
  return getAttribute(uid, ANCS_ATTR_MESSAGE, buffer, bufsize);
}

uint16_t BLEAncs::getMessageSize(uint32_t uid)
{
  char buf[20] = { 0 };

  VERIFY( getAttribute(uid, ANCS_ATTR_MESSAGE_SIZE, buf, sizeof(buf)), 0);
  uint16_t result = strtoul(buf, NULL, 10);

  return result;
}

uint16_t BLEAncs::getDate(uint32_t uid, void* buffer, uint16_t bufsize)
{
  return getAttribute(uid, ANCS_ATTR_DATE, buffer, bufsize);
}

uint16_t BLEAncs::getPosActionLabel(uint32_t uid, void* buffer, uint16_t bufsize)
{
  return getAttribute(uid, ANCS_ATTR_POSITIVE_ACTION_LABEL, buffer, bufsize);
}

uint16_t BLEAncs::getNegActionLabel(uint32_t uid, void* buffer, uint16_t bufsize)
{
  return getAttribute(uid, ANCS_ATTR_NEGATIVE_ACTION_LABEL, buffer, bufsize);
}

uint16_t BLEAncs::getAppName(uint32_t uid, void* buffer, uint16_t bufsize)
{
  // First get AppID
  char appID[64] = { 0 };
  VERIFY( getAppID(uid, appID, sizeof(appID)), 0 );

  // Then get App Display Name
  return getAppAttribute(appID, ANCS_APP_ATTR_DISPLAY_NAME, buffer, bufsize);
}

bool BLEAncs::actPositive(uint32_t uid)
{
  return performAction(uid, ANCS_ACTION_POSITIVE);
}

bool BLEAncs::actNegative(uint32_t uid)
{
  return performAction(uid, ANCS_ACTION_NEGATIVE);
}


/*------------------------------------------------------------------*/
/* Callback
 *------------------------------------------------------------------*/
void bleancs_notification_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEAncs& svc = (BLEAncs&) chr->parentService();
  svc._handleNotification(data, len);
}

void bleancs_data_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEAncs& svc = (BLEAncs&) chr->parentService();
  svc._handleData(data, len);
}
