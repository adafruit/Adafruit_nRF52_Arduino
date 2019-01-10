/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.

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

#include "Uart.h"
#include "Arduino.h"
#include "wiring_private.h"


void serialEventRun(void)
{
  if (serialEvent && Serial.available() ) serialEvent();
}

Uart::Uart(NRF_UART_Type *_nrfUart, IRQn_Type _IRQn, uint8_t _pinRX, uint8_t _pinTX)
{
  nrfUart = _nrfUart;
  IRQn = _IRQn;
  uc_pinRX = g_ADigitalPinMap[_pinRX];
  uc_pinTX = g_ADigitalPinMap[_pinTX];
  uc_hwFlow = 0;

  _mutex = NULL;
  _begun = false;
}

Uart::Uart(NRF_UART_Type *_nrfUart, IRQn_Type _IRQn, uint8_t _pinRX, uint8_t _pinTX, uint8_t _pinCTS, uint8_t _pinRTS)
{
  nrfUart = _nrfUart;
  IRQn = _IRQn;
  uc_pinRX = g_ADigitalPinMap[_pinRX];
  uc_pinTX = g_ADigitalPinMap[_pinTX];
  uc_pinCTS = g_ADigitalPinMap[_pinCTS];
  uc_pinRTS = g_ADigitalPinMap[_pinRTS];
  uc_hwFlow = 1;

  _mutex = NULL;
  _begun = false;
}

void Uart::setPins(uint8_t pin_rx, uint8_t pin_tx)
{
  uc_pinRX = pin_rx;
  uc_pinTX = pin_tx;
}

void Uart::begin(unsigned long baudrate)
{
  begin(baudrate, (uint8_t)SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint16_t /*config*/)
{
  // skip if already begun
  if ( _begun ) return;

  nrfUart->PSELTXD = uc_pinTX;
  nrfUart->PSELRXD = uc_pinRX;

  if (uc_hwFlow == 1) {
    nrfUart->PSELCTS = uc_pinCTS;
    nrfUart->PSELRTS = uc_pinRTS;
    nrfUart->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) | UART_CONFIG_HWFC_Enabled;
  } else {
    nrfUart->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) | UART_CONFIG_HWFC_Disabled;
  }

  uint32_t nrfBaudRate;

  if (baudrate <= 1200) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud1200;
  } else if (baudrate <= 2400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud2400;
  } else if (baudrate <= 4800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud4800;
  } else if (baudrate <= 9600) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud9600;
  } else if (baudrate <= 14400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud14400;
  } else if (baudrate <= 19200) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud19200;
  } else if (baudrate <= 28800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud28800;
  } else if (baudrate <= 38400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud38400;
  } else if (baudrate <= 57600) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud57600;
  } else if (baudrate <= 76800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud76800;
  } else if (baudrate <= 115200) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud115200;
  } else if (baudrate <= 230400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud230400;
  } else if (baudrate <= 250000) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud250000;
  } else if (baudrate <= 460800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud460800;
  } else if (baudrate <= 921600) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud921600;
  } else {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud1M;
  }

  nrfUart->BAUDRATE = nrfBaudRate;

  nrfUart->ENABLE = UART_ENABLE_ENABLE_Enabled;

  nrfUart->EVENTS_RXDRDY = 0x0UL;
  nrfUart->EVENTS_TXDRDY = 0x0UL;

  nrfUart->TASKS_STARTRX = 0x1UL;
  nrfUart->TASKS_STARTTX = 0x1UL;

  nrfUart->INTENSET = UART_INTENSET_RXDRDY_Msk;

  NVIC_ClearPendingIRQ(IRQn);
  NVIC_SetPriority(IRQn, 3);
  NVIC_EnableIRQ(IRQn);

  _mutex = xSemaphoreCreateMutex();
  _begun = true;
}

void Uart::end()
{
  NVIC_DisableIRQ(IRQn);

  nrfUart->INTENCLR = UART_INTENCLR_RXDRDY_Msk;

  nrfUart->TASKS_STOPRX = 0x1UL;
  nrfUart->TASKS_STOPTX = 0x1UL;

  nrfUart->ENABLE = UART_ENABLE_ENABLE_Disabled;

  nrfUart->PSELTXD = 0xFFFFFFFF;
  nrfUart->PSELRXD = 0xFFFFFFFF;

  nrfUart->PSELRTS = 0xFFFFFFFF;
  nrfUart->PSELCTS = 0xFFFFFFFF;

  rxBuffer.clear();

  vSemaphoreDelete(_mutex);
  _mutex = NULL;
  _begun = false;
}

void Uart::flush()
{
	rxBuffer.clear();
}

void Uart::IrqHandler()
{
  if (nrfUart->EVENTS_RXDRDY)
  {
    nrfUart->EVENTS_RXDRDY = 0x0UL;
    rxBuffer.store_char(nrfUart->RXD);
  }
}

int Uart::available()
{
  return rxBuffer.available();
}

int Uart::peek()
{
  return rxBuffer.peek();
}

int Uart::read()
{
  return rxBuffer.read_char();
}

size_t Uart::write(const uint8_t data)
{
  xSemaphoreTake(_mutex, portMAX_DELAY);

  nrfUart->TXD = data;

  while(!nrfUart->EVENTS_TXDRDY);

  nrfUart->EVENTS_TXDRDY = 0x0UL;

  xSemaphoreGive(_mutex);

  return 1;
}

Uart SERIAL_PORT_HARDWARE( NRF_UART0, UARTE0_UART0_IRQn, PIN_SERIAL_RX, PIN_SERIAL_TX );

#ifdef HAVE_HWSERIAL2
// TODO UART1 is UARTE only, need update class Uart to work
Uart Serial2( NRF_UARTE1, UARTE1_IRQn, PIN_SERIAL2_RX, PIN_SERIAL2_TX );
#endif

extern "C"
{
  void UARTE0_UART0_IRQHandler()
  {
    SERIAL_PORT_HARDWARE.IrqHandler();
  }
}
