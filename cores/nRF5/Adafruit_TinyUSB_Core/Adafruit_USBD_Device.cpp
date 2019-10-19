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

#ifndef USB_LANGUAGE
  #define USB_LANGUAGE  0x0409 // default is English
#endif

#ifndef USB_CONFIG_POWER
  #define USB_CONFIG_POWER 100
#endif

extern uint8_t load_serial_number(uint16_t* serial_str);

Adafruit_USBD_Device USBDevice;

Adafruit_USBD_Device::Adafruit_USBD_Device(void)
{
  tusb_desc_device_t const desc_dev =
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

  tusb_desc_configuration_t const dev_cfg =
  {
    .bLength             = sizeof(tusb_desc_configuration_t),
    .bDescriptorType     = TUSB_DESC_CONFIGURATION,

    // Total Length & Interface Number will be updated later
    .wTotalLength        = 0,
    .bNumInterfaces      = 0,
    .bConfigurationValue = 1,
    .iConfiguration      = 0x00,
    .bmAttributes        = TU_BIT(7) | TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
    .bMaxPower           = TUSB_DESC_CONFIG_POWER_MA(USB_CONFIG_POWER)
  };

  memcpy(_desc_cfg_buffer, &dev_cfg, sizeof(tusb_desc_configuration_t));
  _desc_cfg        = _desc_cfg_buffer;
  _desc_cfg_maxlen = sizeof(_desc_cfg_buffer);
  _desc_cfg_len    = sizeof(tusb_desc_configuration_t);

  _itf_count    = 0;
  _epin_count   = _epout_count = 1;

  _language_id  = USB_LANGUAGE;
  _manufacturer = USB_MANUFACTURER;
  _product      = USB_PRODUCT;
}

// Add interface descriptor
// - Interface number will be updated to match current count
// - Endpoint number is updated to be unique
bool Adafruit_USBD_Device::addInterface(Adafruit_USBD_Interface& itf)
{
  uint8_t* desc = _desc_cfg+_desc_cfg_len;
  uint16_t const len = itf.getDescriptor(_itf_count, desc, _desc_cfg_maxlen-_desc_cfg_len);
  uint8_t* desc_end = desc+len;

  if ( !len ) return false;

  while (desc < desc_end)
  {
    if (desc[1] == TUSB_DESC_INTERFACE)
    {
      tusb_desc_interface_t* desc_itf = (tusb_desc_interface_t*) desc;
      if (desc_itf->bAlternateSetting == 0) _itf_count++;
    }else if (desc[1] == TUSB_DESC_ENDPOINT)
    {
      tusb_desc_endpoint_t* desc_ep = (tusb_desc_endpoint_t*) desc;
      desc_ep->bEndpointAddress |= (desc_ep->bEndpointAddress & 0x80) ? _epin_count++ : _epout_count++;
    }

    if (desc[0] == 0) return false;
    desc += desc[0]; // next
  }

  _desc_cfg_len += len;

  // Update configuration descriptor
  tusb_desc_configuration_t* config = (tusb_desc_configuration_t*)_desc_cfg;
  config->wTotalLength = _desc_cfg_len;
  config->bNumInterfaces = _itf_count;

  return true;
}

void Adafruit_USBD_Device::setDescriptorBuffer(uint8_t* buf, uint32_t buflen)
{
  if (buflen < _desc_cfg_maxlen)
    return;

  memcpy(buf, _desc_cfg, _desc_cfg_len);
  _desc_cfg        = buf;
  _desc_cfg_maxlen = buflen;
}

void Adafruit_USBD_Device::setID(uint16_t vid, uint16_t pid)
{
  _desc_device.idVendor  = vid;
  _desc_device.idProduct = pid;
}

void Adafruit_USBD_Device::setVersion(uint16_t bcd)
{
  _desc_device.bcdUSB = bcd;
}


void Adafruit_USBD_Device::setLanguageDescriptor (uint16_t language_id)
{
  _language_id = language_id;
}

void Adafruit_USBD_Device::setManufacturerDescriptor(const char *s)
{
  _manufacturer = s;
}

void Adafruit_USBD_Device::setProductDescriptor(const char *s)
{
  _product = s;
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
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations
  return USBDevice._desc_cfg;
}

static int utf8_to_unichar(const char *str8, int *unicharp)
{
  int unichar;
  int len;

  if (str8[0] < 0x80)
    len = 1;
  else if ((str8[0] & 0xe0) == 0xc0)
    len = 2;
  else if ((str8[0] & 0xf0) == 0xe0)
    len = 3;
  else if ((str8[0] & 0xf8) == 0xf0)
    len = 4;
  else if ((str8[0] & 0xfc) == 0xf8)
    len = 5;
  else if ((str8[0] & 0xfe) == 0xfc)
    len = 6;
  else
    return -1;

  switch (len) {
    case 1:
      unichar = str8[0];
      break;
    case 2:
      unichar = str8[0] & 0x1f;
      break;
    case 3:
      unichar = str8[0] & 0x0f;
      break;
    case 4:
      unichar = str8[0] & 0x07;
      break;
    case 5:
      unichar = str8[0] & 0x03;
      break;
    case 6:
      unichar = str8[0] & 0x01;
      break;
  }

  for (int i = 1; i < len; i++) {
          if ((str8[i] & 0xc0) != 0x80)
                  return -1;
          unichar <<= 6;
          unichar |= str8[i] & 0x3f;
  }

  *unicharp = unichar;
  return len;
}

// Simple UCS-2/16-bit coversion, which handles the Basic Multilingual Plane
static int strcpy_uni16(const char *s, uint16_t *buf, int bufsize) {
  int i = 0;
  int buflen = 0;

  while (i < bufsize) {
    int unichar;
    int utf8len = utf8_to_unichar(s + i, &unichar);

    if (utf8len < 0) {
      // Invalid utf8 sequence, skip it
      i++;
      continue;
    }

    i += utf8len;

    // If the codepoint is larger than 16 bit, skip it
    if (unichar <= 0xffff)
      buf[buflen++] = unichar;
  }

  buf[buflen] = '\0';
  return buflen;
}

// up to 32 unicode characters (header make it 33)
static uint16_t _desc_str[33];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index)
{
  uint8_t chr_count;

  switch (index)
  {
    case 0:
      _desc_str[1] = USBDevice.getLanguageDescriptor();
      chr_count = 1;
    break;

    case 1:
      chr_count = strcpy_uni16(USBDevice.getManufacturerDescriptor(), _desc_str + 1, 32);
    break;

    case 2:
      chr_count = strcpy_uni16(USBDevice.getProductDescriptor(), _desc_str + 1, 32);
    break;

    case 3:
      // serial Number
      chr_count = load_serial_number(_desc_str+1);
    break;

    default: return NULL;
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}

} // extern C

#endif // USE_TINYUSB
