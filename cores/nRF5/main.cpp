/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define ARDUINO_MAIN
#include "Arduino.h"

// weak function to avoid compilation error with
// non-Bluefruit library sketch such as ADC read test
void Bluefruit_printInfo() __attribute__((weak));
void Bluefruit_printInfo() {}

// DEBUG Level 3
#if CFG_SYSVIEW
  #include "SEGGER_SYSVIEW.h"
#endif

static TaskHandle_t  _loopHandle;


// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

#define LOOP_STACK_SZ       (256*4)
#define CALLBACK_STACK_SZ   (256*3)

static void loop_task(void* arg)
{
  (void) arg;

#if CFG_DEBUG && (CFG_LOGGER & ADALOG_TYPE_SERIAL)
  // initialize this before setup() to allow simplified logging in setup
  // If Serial is not begin(), call it to avoid hard fault
  if ( !Serial ) Serial.begin(115200);
#endif

  setup();

  dbgPrintVersion();
  // dbgMemInfo();

#if CFG_DEBUG
  Bluefruit_printInfo();
#endif

  while (1)
  {
    loop();
    yield(); // yield to run other task

    // Serial events
    if (serialEvent && serialEventRun) serialEventRun();

  }
}

// \brief Main entry point of Arduino application
int main( void )
{

#if (CFG_LOGGER & ADALOG_TYPE_RTT)
  SEGGER_RTT_Init();
  SEGGER_RTT_ConfigUpBuffer(0, nullptr, nullptr, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
  SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Initialized");
#endif

  init();
  initVariant();

#ifdef USE_TINYUSB
  Adafruit_TinyUSB_Core_init();
#endif

  // Create a task for loop()
  xTaskCreate( loop_task, "loop", LOOP_STACK_SZ, NULL, TASK_PRIO_LOW, &_loopHandle);

  // Initialize callback task
  ada_callback_init(CALLBACK_STACK_SZ);

  // Start FreeRTOS scheduler.
  vTaskStartScheduler();

  NVIC_SystemReset();

  return 0;
}

void suspendLoop(void)
{
  vTaskSuspend(_loopHandle);
}





#if CFG_DEBUG
  // DESIGN CHOICE -- Order of preference for logging is first of these enabled:
  // 1. RTT, if enabled
  // 2. SWO, if enabled
  // 3. Serial, if enabled

  #if (CFG_LOGGER & ADALOG_TYPE_RTT)
    // _write overload provided in SEGGER_RTT_Print
    class Segger_RTT_Stream_t : public Stream {
    public:
      Segger_RTT_Stream_t(void);
      virtual size_t write(uint8_t b) override;
      virtual size_t write(const uint8_t *buffer, size_t size) override;
      virtual int availableForWrite() override;
      virtual int available() override;
      virtual int read() override;
      virtual int peek() override;
      virtual void flush() override;
      // Additional non-virtual functions to emulate `Serial`
      operator bool();
      void begin(uint32_t baud);
      void begin(uint32_t baud, uint8_t config);
      void end(void);
      // serialEvent() -- No support for SVC when host sends data

    };
    Segger_RTT_Stream_t::Segger_RTT_Stream_t(void) {}
    size_t Segger_RTT_Stream_t::write(uint8_t b) {
      return SEGGER_RTT_PutChar(0, (char)b);
    }
    size_t Segger_RTT_Stream_t::write(const uint8_t *buffer, size_t size) {
      return SEGGER_RTT_Write(0, buffer, size);
    }
    int Segger_RTT_Stream_t::availableForWrite() {
      return SEGGER_RTT_GetAvailWriteSpace(0);
    }
    int Segger_RTT_Stream_t::available() { // to read
      return SEGGER_RTT_HasData(0);
    }
    int Segger_RTT_Stream_t::read() {
      return SEGGER_RTT_GetKey();
    }
    int Segger_RTT_Stream_t::peek() {
      return SEGGER_RTT_Peek();
    }
    void Segger_RTT_Stream_t::flush() {
      // no-op -- cannot flush as cannot control host
    }
    // Baud and config are ignored for RTT
    void Segger_RTT_Stream_t::begin (uint32_t baud)
    {
      (void) baud;
    }
    void Segger_RTT_Stream_t::begin (uint32_t baud, uint8_t config)
    {
      (void) baud;
      (void) config;
    }
    void Segger_RTT_Stream_t::end(void)
    {
      // nothing to do
    }
    Segger_RTT_Stream_t::operator bool()
    {
      return true;
    }


    Segger_RTT_Stream_t s_Segger_RTT_Stream;
    Stream& Adalog_Default_Logger = s_Segger_RTT_Stream;

  #elif (CFG_LOGGER & ADALOG_TYPE_SERIAL)

    extern "C"
    {
      // nanolib printf() retarget
      int _write (int fd, const void *buf, size_t count)
      {
        (void) fd;

        if ( Serial )
        {
          return Serial.write( (const uint8_t *) buf, count);
        }
        return 0;
      }
    }
    Stream& Adalog_Default_Logger = Serial;

  #endif
#endif
