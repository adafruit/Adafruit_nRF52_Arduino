/**************************************************************************/
/*!
    @file     usb.c
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

#ifdef NRF52840_XXAA

#include "nrfx.h"
#include "nrfx_power.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#include "nrf_usbd.h"
#include "tusb.h"
#include "usb.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
extern uint16_t usb_desc_str_serial[1+16];

// Init usb when starting up. Softdevice is not enabled yet
void usb_init(void)
{
  // Power module init
  const nrfx_power_config_t pwr_cfg = { 0 };
  nrfx_power_init(&pwr_cfg);

  // Register tusb function as USB power handler
  const nrfx_power_usbevt_config_t config = { .handler = (nrfx_power_usb_event_handler_t) tusb_hal_nrf_power_event };
  nrfx_power_usbevt_init(&config);

  nrfx_power_usbevt_enable();

  NVIC_SetPriority(USBD_IRQn, 2);

  // USB power may already be ready at this time -> no event generated
  // We need to invoke the handler based on the status initially
  uint32_t usb_reg = NRF_POWER->USBREGSTATUS;

  if ( usb_reg & POWER_USBREGSTATUS_VBUSDETECT_Msk ) tusb_hal_nrf_power_event(NRFX_POWER_USB_EVT_DETECTED);
  if ( usb_reg & POWER_USBREGSTATUS_OUTPUTRDY_Msk  ) tusb_hal_nrf_power_event(NRFX_POWER_USB_EVT_READY);

  // Create Serial string descriptor
  char tmp_serial[17];
  sprintf(tmp_serial, "%08lX%08lX", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0]);

  for(uint8_t i=0; i<16; i++)
  {
    usb_desc_str_serial[1+i] = tmp_serial[i];
  }

  // Init tinyusb stack
  tusb_init();
}

// Must be called before sd_softdevice_enable()
// NRF_POWER is restricted prph used by Softdevice, must be release before enable SD
void usb_softdevice_pre_enable(void)
{
  nrfx_power_usbevt_disable();
  nrfx_power_usbevt_uninit();
  nrfx_power_uninit();
}

// Must be called after sd_softdevice_enable()
// To re-enable USB
void usb_softdevice_post_enable(void)
{
  sd_power_usbdetected_enable(true);
  sd_power_usbpwrrdy_enable(true);
  sd_power_usbremoved_enable(true);
}


// Invoked when cdc when line state changed e.g connected/disconnected
// Use to reset to DFU when disconnect with 1200 bps
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;  // interface ID, not used

  // DTR = false is counted as disconnected
  if ( !dtr )
  {
    cdc_line_coding_t coding;
    tud_cdc_get_line_coding(&coding);

    if ( coding.bit_rate == 1200 ) enterSerialDfu();
  }
}



#endif
