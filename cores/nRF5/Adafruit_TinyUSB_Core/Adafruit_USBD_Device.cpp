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

#ifndef USB_MANUFACTURER
  #define USB_MANUFACTURER "Unknown"
#endif

#ifndef USB_PRODUCT
  #define USB_PRODUCT "Unknown"
#endif

extern uint8_t load_serial_number(uint16_t* serial_str);

Adafruit_USBD_Device USBDevice;

Adafruit_USBD_Device::Adafruit_USBD_Device(void)
{
  tusb_desc_device_t  desc_dev =
  {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

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

extern "C"
{

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &USBDevice._desc_device;
}

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(void)
{
  return USBDevice._desc_cfg;
}

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index)
{
  uint8_t chr_count;

  switch (index)
  {
    case 0:
      // language = English
      _desc_str[1] = 0x0409;
      chr_count = 1;
    break;

    case 1: // Manufacturer
    case 2: // Product
    {
      char const * str = (index == 1) ? USB_MANUFACTURER : USB_PRODUCT;

      // cap at max char
      chr_count = strlen(str);
      if ( chr_count > 31 ) chr_count = 31;

      for(uint8_t i=0; i<chr_count; i++)
      {
        _desc_str[1+i] = str[i];
      }
    }
    break;

    case 3:
      // serial Number
      chr_count = load_serial_number(_desc_str+1);
    break;

    default: return NULL;
  }

  // first byte is len, second byte is string type
  _desc_str[0] = TUD_DESC_STR_HEADER(chr_count);

  return _desc_str;
}

} // extern C

#endif // USE_TINYUSB
