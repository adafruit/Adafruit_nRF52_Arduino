/**************************************************************************/
/*!
    @file     BLEAncs.h
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
#ifndef BLEANCS_H_
#define BLEANCS_H_

#include "../BLEClientCharacteristic.h"
#include "bluefruit_common.h"

#include "BLEClientService.h"

extern const uint8_t BLEANCS_UUID_SERVICE[];
extern const uint8_t BLEANCS_UUID_CHR_CONTROL[];
extern const uint8_t BLEANCS_UUID_CHR_NOTIFICATION[];
extern const uint8_t BLEANCS_UUID_CHR_DATA[];

// Category ID
enum
{
  ANCS_CAT_OTHER                 ,
  ANCS_CAT_INCOMING_CALL         ,
  ANCS_CAT_MISSED_CALL           ,
  ANCS_CAT_VOICE_MAIL            ,
  ANCS_CAT_SOCIAL                ,
  ANCS_CAT_SCHEDULE              ,
  ANCS_CAT_EMAIL                 ,
  ANCS_CAT_NEWS                  ,
  ANCS_CAT_HEALTH_AND_FITNESS    ,
  ANCS_CAT_BUSSINESS_AND_FINANCE ,
  ANCS_CAT_LOCATION              ,
  ANCS_CAT_ENTERTAINMENT
};

// Event ID
enum
{
  ANCS_EVT_NOTIFICATION_ADDED    ,
  ANCS_EVT_NOTIFICATION_MODIFIED ,
  ANCS_EVT_NOTIFICATION_REMOVED
};

// Command ID
enum
{
  ANCS_CMD_GET_NOTIFICATION_ATTR ,
  ANCS_CMD_GET_APP_ATTR          ,
  ANCS_CMD_PERFORM_NOTIFICATION_ACTION
};

// Notification Attribute ID
enum
{
  ANCS_ATTR_APP_IDENTIFIER        ,
  ANCS_ATTR_TITLE                 , // followed bye 2-byte length
  ANCS_ATTR_SUBTITLE              , // followed bye 2-byte length
  ANCS_ATTR_MESSAGE               , // followed bye 2-byte length
  ANCS_ATTR_MESSAGE_SIZE          ,
  ANCS_ATTR_DATE                  , // UTC#35 yyyyMMdd'T'HHmmSS
  ANCS_ATTR_POSITIVE_ACTION_LABEL ,
  ANCS_ATTR_NEGATIVE_ACTION_LABEL ,

  ANCS_ATTR_INVALID
};

// Action ID
enum
{
  ANCS_ACTION_POSITIVE,
  ANCS_ACTION_NEGATIVE
};

// Application Attribute ID
enum
{
  ANCS_APP_ATTR_DISPLAY_NAME,

  ANCS_APP_ATTR_INVALID
};

typedef struct
{
  uint8_t  eventID;

  struct ATTR_PACKED
  {
    uint8_t silent         : 1;
    uint8_t important      : 1;
    uint8_t preExisting    : 1;
    uint8_t positiveAction : 1;
    uint8_t NegativeAction : 1;
  }eventFlags;

  uint8_t  categoryID;
  uint8_t  categoryCount;
  uint32_t uid;
} AncsNotification_t;

VERIFY_STATIC( sizeof(AncsNotification_t) == 8);

class BLEAncs : public BLEClientService
{
  public:
    typedef void (*notification_callback_t) (AncsNotification_t* notif);

    BLEAncs(void);

    virtual bool  begin(void);
    virtual bool  discover(uint16_t conn_handle);

    void setNotificationCallback(notification_callback_t fp);
    bool enableNotification(void);
    bool disableNotification(void);

    // Main commands
    uint16_t getAttribute    (uint32_t uid, uint8_t attr, void* buffer, uint16_t bufsize);
    uint16_t getAppAttribute (const char* appid, uint8_t attr, void* buffer, uint16_t bufsize);
    bool     performAction   (uint32_t uid, uint8_t actionid);

    // High Level helper
    uint16_t getAppName        (uint32_t uid, void* buffer, uint16_t bufsize);
    uint16_t getAppID          (uint32_t uid, void* buffer, uint16_t bufsize);
    uint16_t getTitle          (uint32_t uid, void* buffer, uint16_t bufsize);
    uint16_t getSubtitle       (uint32_t uid, void* buffer, uint16_t bufsize);
    uint16_t getMessage        (uint32_t uid, void* buffer, uint16_t bufsize);
    uint16_t getMessageSize    (uint32_t uid);
    uint16_t getDate           (uint32_t uid, void* buffer, uint16_t bufsize);
    uint16_t getPosActionLabel (uint32_t uid, void* buffer, uint16_t bufsize);
    uint16_t getNegActionLabel (uint32_t uid, void* buffer, uint16_t bufsize);

    bool     actPositive       (uint32_t uid);
    bool     actNegative       (uint32_t uid);

  private:
    BLEClientCharacteristic _control;
    BLEClientCharacteristic _notification;
    BLEClientCharacteristic _data;

    notification_callback_t _notif_cb;

    AdaMsg                  _adamsg;

    void _handleNotification(uint8_t* data, uint16_t len);
    void _handleData(uint8_t* data, uint16_t len);

    friend void bleancs_notification_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
    friend void bleancs_data_cb        (BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
};


#endif /* BLEANCS_H_ */
