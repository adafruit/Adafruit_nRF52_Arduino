/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach for Adafruit
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifdef NRF52840_XXAA

#include "nrfx.h"
#include "nrfx_power.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#include "nrf_usbd.h"
#include "tusb.h"
#include "usb.h"

#include "rtos.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#define USBD_STACK_SZ   (150)

// Serial is 64-bit DeviceID -> 16 chars len
uint16_t usb_desc_str_serial[1+16] = { TUD_DESC_STR_HEADER(16) };

static void usb_device_task(void* param);

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

  // Priorities 0, 1, 4 (nRF52) are reserved for SoftDevice
  // 2 is highest for application
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

  // Create a task for tinyusb device stack
  xTaskCreate( usb_device_task, "usbd", USBD_STACK_SZ, NULL, TASK_PRIO_HIGH, NULL);
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

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
static void usb_device_task(void* param)
{
  (void) param;

  // RTOS forever loop
  while (1)
  {
    // tinyusb device task
    tud_task();
  }
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
