/**************************************************************************/
/*!
    @file     BLEDis.h
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
#ifndef BLEDIS_H_
#define BLEDIS_H_

#include "bluefruit_common.h"

#include "BLECharacteristic.h"
#include "BLEService.h"

class BLEDis : public BLEService
{
  protected:
    union {
      struct {
        const char * _system_id;     // UUID 0x2A23
        const char * _model;         // UUID 0x2A23
        const char * _serial;        // UUID 0x2A25
        const char * _firmware_rev;  // UUID 0x2A26
        const char * _hardware_rev;  // UUID 0x2A27
        const char * _software_rev;  // UUID 0x2A28
        const char * _manufacturer;  // UUID 0x2A29
        const char * _reg_cert_list; // UUID 0x2A2A

        const char * _pnp_id;        // UUID 0x2A50
      };

      const char * _strarr[9];
    };

    union {
      struct {
        uint8_t _system_id_length;
        uint8_t _model_length;
        uint8_t _serial_length;
        uint8_t _firmware_rev_length;
        uint8_t _hardware_rev_length;
        uint8_t _software_rev_length;
        uint8_t _manufacturer_length;
        uint8_t _reg_cert_list_length;
        uint8_t _pnp_id_length;
      };

      uint8_t _strarr_length[9];
    };

  public:
    BLEDis(void);

    void setSystemID     (const char* system_id, uint8_t length);
    void setModel        (const char* model,uint8_t length);
    void setSerialNum    (const char* serial_num, uint8_t length);
    void setFirmwareRev  (const char* firmware_rev, uint8_t length);
    void setHardwareRev  (const char* hw_rev,uint8_t length);
    void setSoftwareRev  (const char* sw_rev, uint8_t length);
    void setManufacturer (const char* manufacturer, uint8_t length);
    void setRegCertList  (const char* reg_cert_list, uint8_t length);
    void setPNPID        (const char* pnp_id, uint8_t pnp_id_length);

    void setSystemID     (const char* system_id     ) { setSystemID(system_id, strlen(system_id));            }
    void setModel        (const char* model         ) { setModel(model, strlen(model));                       }
    void setSerialNum    (const char* serial_num    ) { setSerialNum(serial_num, strlen(serial_num));         }
    void setFirmwareRev  (const char* firmware_rev  ) { setFirmwareRev(firmware_rev, strlen(firmware_rev));   }
    void setHardwareRev  (const char* hw_rev        ) { setHardwareRev(hw_rev, strlen(hw_rev));               }
    void setSoftwareRev  (const char* sw_rev        ) { setSoftwareRev(sw_rev, strlen(sw_rev));               }
    void setManufacturer (const char* manufacturer  ) { setManufacturer(manufacturer, strlen(manufacturer));  }
    void setRegCertList  (const char* reg_cert_list ) { setRegCertList(reg_cert_list, strlen(reg_cert_list)); }
    void setPNPID        (const char* pnp_id        ) { setPNPID(pnp_id, strlen(pnp_id));                     }

    virtual err_t begin(void);
};


#endif /* BLEDIS_H_ */
