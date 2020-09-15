/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019, hathach for Adafruit
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

#ifdef USE_TINYUSB

#include "nrfx.h"
#include "nrfx_power.h"

#include "Arduino.h"
#include "Adafruit_TinyUSB_Core.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#define USBD_STACK_SZ   (200)

// tinyusb function that handles power event (detected, ready, removed)
// We must call it within SD's SOC event handler, or set it as power event handler if SD is not enabled.
extern "C" void tusb_hal_nrf_power_event(uint32_t event);

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
extern "C" void USBD_IRQHandler(void)
{
#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordEnterISR();
#endif

  tud_int_handler(0);

#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordExitISR();
#endif
}

//--------------------------------------------------------------------+
// Core Init & Touch1200
//--------------------------------------------------------------------+

// Init usb hardware when starting up. Softdevice is not enabled yet
static void usb_hardware_init(void)
{
  // Priorities 0, 1, 4 (nRF52) are reserved for SoftDevice
  // 2 is highest for application
  NVIC_SetPriority(USBD_IRQn, 2);

  // USB power may already be ready at this time -> no event generated
  // We need to invoke the handler based on the status initially
  uint32_t usb_reg = NRF_POWER->USBREGSTATUS;

  // Power module init
  const nrfx_power_config_t pwr_cfg = { 0 };
  nrfx_power_init(&pwr_cfg);

  // Register tusb function as USB power handler
  const nrfx_power_usbevt_config_t config = { .handler = (nrfx_power_usb_event_handler_t) tusb_hal_nrf_power_event };
  nrfx_power_usbevt_init(&config);

  nrfx_power_usbevt_enable();

  if ( usb_reg & POWER_USBREGSTATUS_VBUSDETECT_Msk ) tusb_hal_nrf_power_event(NRFX_POWER_USB_EVT_DETECTED);
}

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
static void usb_device_task(void* param)
{
  (void) param;

  Serial.setStringDescriptor("TinyUSB Serial");
  USBDevice.addInterface(Serial);
  USBDevice.setID(USB_VID, USB_PID);
  USBDevice.begin();

  usb_hardware_init();

  // Init tinyusb stack
  tusb_init();

  // RTOS forever loop
  while (1)
  {
    // tinyusb device task
    tud_task();
  }
}

void Adafruit_TinyUSB_Core_init(void)
{
  // Create a task for tinyusb device stack
  xTaskCreate(usb_device_task, "usbd", USBD_STACK_SZ, NULL, TASK_PRIO_HIGH, NULL);
}

void Adafruit_TinyUSB_Core_touch1200(void)
{
  delay(5); // a few millisecond for USB control status completion
  enterSerialDfu();
}

//--------------------------------------------------------------------+
// Adafruit_USBD_Device platform dependent
//--------------------------------------------------------------------+

uint8_t Adafruit_USBD_Device::getSerialDescriptor(uint16_t* serial_str)
{
  // Serial is 64-bit DeviceID -> 16 chars len
  char tmp_serial[17];
  sprintf(tmp_serial, "%08lX%08lX", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0]);

  for(uint8_t i=0; i<16; i++) serial_str[i] = tmp_serial[i];
  return 16;
}

#endif // USE_TINYUSB
