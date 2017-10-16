/**************************************************************************/
/*!
    @file     HAPProcedure.h
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
#ifndef HAPPROCEDURE_H_
#define HAPPROCEDURE_H_

enum HAPOpcode
{
  HAP_OPCODE_CHR_SIGNATURE_READ = 0x01 ,
  HAP_OPCODE_CHR_WRITE                 ,
  HAP_OPCODE_CHR_READ                  ,
  HAP_OPCODE_CHR_TIMED_WRITE           ,
  HAP_OPCODE_CHR_EXECUTE_WRITE         ,
  HAP_OPCODE_SVC_SIGNATURE_READ
};

enum HAPParamType
{
  HAP_PARAM_VALUE = 1                     ,
  HAP_PARAM_ADDITIONAL_AUTHORIZATION_DATA ,
  HAP_PARAM_ORIGIN                        , // local vs remote
  HAP_PARAM_CHR_TYPE                      ,
  HAP_PARAM_CHR_ID                        ,
  HAP_PARAM_SVC_TYPE                      ,
  HAP_PARAM_SVC_ID                        ,
  HAP_PARAM_TTL                           ,
  HAP_PARAM_RETURN_RESP                   ,
  HAP_PARAM_HAP_CHR_PROPERTIES_DESC       ,
  HAP_PARAM_GATT_USR_DESC                 ,
  HAP_PARAM_GATT_FORMAT_DESC              ,
  HAP_PARAM_GATT_VALID_RANGE              ,
  HAP_PARAM_HAP_STEP_VALUE_DESC           ,
  HAP_PARAM_HAP_SVC_PROPERTIES            ,
  HAP_PARAM_HAP_LINKED_SVC                ,
  HAP_PARAM_HAP_VALID_VALUES_DESC         ,
  HAP_PARAM_HAP_VALID_VALUES_RANGE_DESC
};

struct ATTR_PACKED
{
  uint8_t fragment : 1; // 0 : first Fragment (or no fragment), 1 : Continuation of Fragment
  uint8_t          : 3; // reserved
  uint8_t type     : 3; // 0b000 : request, 0b001 : response
  uint8_t lenext   : 1;
} HAPControl_t;

VERIFY_STATIC(sizeof (HAPControl_t) == 1);

struct ATTR_PACKED
{
  HAPControl_t control ;
  uint8_t opcode       ;
  uint8_t tid          ; // Transaction ID
  uint8_t instance_id  ; // Service or Characteristic Instance ID
} HAPRequest_t;

struct ATTR_PACKED
{
  HAPControl_t control;
  uint8_t tid;
  uint8_t status;
};


#endif /* HAPPROCEDURE_H_ */
