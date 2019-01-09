/**************************************************************************/
/*!
    @file     HAPProcedure.h
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
#ifndef HAPPROCEDURE_H_
#define HAPPROCEDURE_H_

enum HAPOpcode_t
{
  HAP_OPCODE_CHR_SIGNATURE_READ = 0x01 ,
  HAP_OPCODE_CHR_WRITE                 ,
  HAP_OPCODE_CHR_READ                  ,
  HAP_OPCODE_CHR_TIMED_WRITE           ,
  HAP_OPCODE_CHR_EXECUTE_WRITE         ,
  HAP_OPCODE_SVC_SIGNATURE_READ
};

enum HAPParamType_t
{
  HAP_PARAM_VALUE = 1                     , // 1
  HAP_PARAM_ADDITIONAL_AUTHORIZATION_DATA , // 2
  HAP_PARAM_ORIGIN                        , // 3 local vs remote
  HAP_PARAM_CHR_TYPE                      , // 4
  HAP_PARAM_CHR_ID                        , // 5
  HAP_PARAM_SVC_TYPE                      , // 6
  HAP_PARAM_SVC_ID                        , // 7
  HAP_PARAM_TTL                           , // 8
  HAP_PARAM_RETURN_RESP                   , // 9
  HAP_PARAM_HAP_CHR_PROPERTIES_DESC       , // 10
  HAP_PARAM_GATT_USR_DESC                 , // 11
  HAP_PARAM_GATT_FORMAT_DESC              , // 12
  HAP_PARAM_GATT_VALID_RANGE              , // 13
  HAP_PARAM_HAP_STEP_VALUE_DESC           , // 14
  HAP_PARAM_HAP_SVC_PROPERTIES            , // 15
  HAP_PARAM_HAP_LINKED_SVC                , // 16
  HAP_PARAM_HAP_VALID_VALUES_DESC         , // 17
  HAP_PARAM_HAP_VALID_VALUES_RANGE_DESC     // 18
};

enum HAPStatusCode_t
{
  HAP_STATUS_SUCCESS = 0                 ,
  HAP_STATUS_UNSUPPORTED_PDU             ,
  HAP_STATUS_MAX_PROCEDURES              ,
  HAP_STATUS_INSUFFICIENT_AUTHORIZATION  ,
  HAP_STATUS_INVALID_INSTANCE_ID         ,
  HAP_STATUS_INSUFFICIENT_AUTHENTICATION ,
  HAP_STATUS_INVALID_REQUEST
};

enum HAPPduType_t
{
  HAP_PDU_REQUEST = 0,
  HAP_PDU_RESPONSE,
};


typedef struct
{
  uint8_t  type;
  uint16_t len;
  const void* value;
}TLV8_t;



typedef struct ATTR_PACKED
{
  uint8_t lenext   : 1; // not used
  uint8_t type     : 3; // 0b000 : request, 0b001 : response
  uint8_t          : 3; // reserved
  uint8_t fragment : 1; // 0 : first Fragment (or no fragment), 1 : Continuation of Fragment
} HAPControl_t;

VERIFY_STATIC(sizeof (HAPControl_t) == 1);

/*-------------  Request -------------*/

typedef struct ATTR_PACKED
{
  HAPControl_t control ;
  uint8_t  opcode      ;
  uint8_t  tid         ; // Transaction ID
  uint16_t instance_id ; // Service or Characteristic Instance ID
} HAPRequestHeader_t;

VERIFY_STATIC(sizeof (HAPRequestHeader_t) == 5);

typedef struct ATTR_PACKED
{
  HAPRequestHeader_t header;

  // Body (optional)
  uint16_t body_len;
  uint8_t  body_data[1];
} HAPRequest_t;

/*------------- Response -------------*/

typedef struct ATTR_PACKED
{
  HAPControl_t control;
  uint8_t tid;
  uint8_t status;
}HAPResponseHeader_t;

VERIFY_STATIC(sizeof (HAPResponseHeader_t) == 3);

typedef struct ATTR_PACKED
{
  HAPResponseHeader_t header;

  // Body (optional)
  uint16_t body_len;
  uint8_t  body_data[1];
} HAPResponse_t;


TLV8_t   tlv8_decode_next(uint8_t const** pp_data, uint16_t* p_len);
void     tlv8_decode_cleanup(TLV8_t tlv);
uint8_t  tlv8_decode_n(uint8_t const* buf, uint16_t bufsize, TLV8_t tlv[], uint8_t count);

bool     tlv8_encode_next(uint8_t** pp_buf, uint16_t* p_buflen, TLV8_t tlv);
uint16_t tlv8_encode_n(uint8_t* buf, uint16_t bufsize, TLV8_t tlv[], uint8_t count);
uint16_t tlv8_encode_calculate_len(TLV8_t tlv_para[], uint8_t count);

#endif /* HAPPROCEDURE_H_ */
