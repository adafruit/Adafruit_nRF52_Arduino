/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach for Adafruit Industries
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

#include "Adafruit_USBD_Device.h"

extern "C"
{
extern uint16_t usb_desc_str_serial[1+16];

// array of pointer to string descriptors
uint16_t const * const string_desc_arr [] =
{
  // 0: is supported language = English
  TUD_DESC_STRCONV(0x0409),

  // 1: Manufacturer
  TUD_DESC_STRCONV('A','d','a','f','r','u','i','t',' ','I','n','d','u','s','t','r','i','e','s'),

  // 2: Product
  TUD_DESC_STRCONV('B','l','u','e','f','r','u','i','t',' ','n','R','F','5','2','8','4','0'),

  // 3: Serials
  usb_desc_str_serial,

  //    // 4: CDC Interface
  //    TUD_DESC_STRCONV('B','l','u','e','f','r','u','i','t',' ','S','e','r','i','a','l'),
  //
  //    // 5: MSC Interface
  //    TUD_DESC_STRCONV('B','l','u','e','f','r','u','i','t',' ','U','F','2'),
};

// tud_desc_set is required by tinyusb stack
tud_desc_set_t tud_desc_set =
{
  .device       = NULL, // update later
  .config       = NULL, // update later
  .string_arr   = (uint8_t const **) string_desc_arr,
  .string_count = sizeof(string_desc_arr)/sizeof(string_desc_arr[0]),

  .hid_report = NULL // update later
};

} // extern C

Adafruit_USBD_Device USBDevice;

Adafruit_USBD_Device::Adafruit_USBD_Device(void)
{
  tusb_desc_device_t  desc_dev =
  {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

#if CFG_TUD_CDC
    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
#else
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
#endif

    .bMaxPacketSize0    = CFG_TUD_ENDOINT0_SIZE,

    .idVendor           = 0,
    .idProduct          = 0,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
  };

  _desc_device = desc_dev;

  tusb_desc_configuration_t dev_cfg =
  {
    .bLength             = sizeof(tusb_desc_configuration_t),
    .bDescriptorType     = TUSB_DESC_CONFIGURATION,

    // Total Length & Interface Number will be updated later
    .wTotalLength        = 0,
    .bNumInterfaces      = 0,

    .bConfigurationValue = 1,
    .iConfiguration      = 0x00,
    .bmAttributes        = TU_BIT(7) | TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
    .bMaxPower           = TUSB_DESC_CONFIG_POWER_MA(100)
  };

  memcpy(_desc_cfg, &dev_cfg, sizeof(tusb_desc_configuration_t));

  _desc_cfglen = sizeof(tusb_desc_configuration_t);
  _itf_count = 0;
  _epin_count = _epout_count = 1;

  tud_desc_set.config = _desc_cfg;
  tud_desc_set.device = &_desc_device;
}

// Add interface descriptor
// - Interface number will be updated to match current count
// - Endpoint number is updated to be unique
bool Adafruit_USBD_Device::addInterface(Adafruit_USBD_Interface& itf)
{
  uint8_t* desc = _desc_cfg+_desc_cfglen;
  uint16_t const len = itf.getDescriptor(desc, sizeof(_desc_cfg)-_desc_cfglen);
  uint8_t* desc_end = desc+len;

  if ( !len ) return false;

  // Handle IAD
  if ( desc[1] == TUSB_DESC_INTERFACE_ASSOCIATION )
  {
    // update starting interface
    ((tusb_desc_interface_assoc_t*) desc)->bFirstInterface = _itf_count;

    desc += desc[0]; // next
  }

  while (desc < desc_end)
  {
    if (desc[1] == TUSB_DESC_INTERFACE)
    {
      // No alternate interface support
      ((tusb_desc_interface_t*) desc)->bInterfaceNumber = _itf_count++;
    }else if (desc[1] == TUSB_DESC_ENDPOINT)
    {
      tusb_desc_endpoint_t* desc_ep = (tusb_desc_endpoint_t*) desc;
      desc_ep->bEndpointAddress |= (desc_ep->bEndpointAddress & 0x80) ? _epin_count++ : _epout_count++;
    }

    if (desc[0] == 0) return false;
    desc += desc[0]; // next
  }

  _desc_cfglen += len;

  // Update config descriptor
  tusb_desc_configuration_t* config = (tusb_desc_configuration_t*)_desc_cfg;
  config->wTotalLength = _desc_cfglen;
  config->bNumInterfaces = _itf_count;

  return true;
}

void Adafruit_USBD_Device::setID(uint16_t vid, uint16_t pid)
{
  _desc_device.idVendor = vid;
  _desc_device.idProduct = pid;
}

bool Adafruit_USBD_Device::begin(void)
{
  return true;
}

#endif // USE_TINYUSB
