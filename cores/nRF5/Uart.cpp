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

#if defined(PIN_SERIAL1_RX) && defined(PIN_SERIAL1_TX)
  if (serialEvent1 && Serial1.available() ) serialEvent1();
#endif

#if defined(PIN_SERIAL2_RX) && defined(PIN_SERIAL2_TX)
  if (serialEvent2 && Serial2.available() ) serialEvent2();
#endif
}

Uart::Uart(NRF_UARTE_Type *_nrfUart, IRQn_Type _IRQn, uint8_t _pinRX, uint8_t _pinTX)
{
  nrfUart = _nrfUart;
  IRQn = _IRQn;
  uc_pinRX = g_ADigitalPinMap[_pinRX];
  uc_pinTX = g_ADigitalPinMap[_pinTX];
  uc_hwFlow = 0;

  _end_tx_sem = NULL;
  _begun = false;
}

Uart::Uart(NRF_UARTE_Type *_nrfUart, IRQn_Type _IRQn, uint8_t _pinRX, uint8_t _pinTX, uint8_t _pinCTS, uint8_t _pinRTS)
{
  nrfUart = _nrfUart;
  IRQn = _IRQn;
  uc_pinRX = g_ADigitalPinMap[_pinRX];
  uc_pinTX = g_ADigitalPinMap[_pinTX];
  uc_pinCTS = g_ADigitalPinMap[_pinCTS];
  uc_pinRTS = g_ADigitalPinMap[_pinRTS];
  uc_hwFlow = 1;

  _end_tx_sem = NULL;
  _begun = false;
}

void Uart::setPins(uint8_t pin_rx, uint8_t pin_tx)
{
  uc_pinRX = g_ADigitalPinMap[pin_rx];
  uc_pinTX = g_ADigitalPinMap[pin_tx];
}

void Uart::begin(unsigned long baudrate)
{
  begin(baudrate, (uint16_t)SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint16_t config)
{
  // skip if already begun
  if ( _begun ) return;

  nrfUart->PSEL.TXD = uc_pinTX;
  nrfUart->PSEL.RXD = uc_pinRX;

  if (uc_hwFlow == 1) {
    nrfUart->PSEL.CTS = uc_pinCTS;
    nrfUart->PSEL.RTS = uc_pinRTS;
    nrfUart->CONFIG = config | (UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
  } else {
    nrfUart->CONFIG = config | (UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos);
  }
 
  const double regValue = 268.43f*(double)baudrate; /* 2^32 / 16000000 = 268.43f. more details here: https://devzone.nordicsemi.com/f/nordic-q-a/391/uart-baudrate-register-values */
  const uint32_t nrfBaudRate = ((uint32_t)regValue + 0x800) & 0xFFFFF000;

  nrfUart->BAUDRATE = nrfBaudRate;

  nrfUart->ENABLE = UARTE_ENABLE_ENABLE_Enabled;

  nrfUart->TXD.PTR = (uint32_t)txBuffer;
  nrfUart->EVENTS_ENDTX = 0x0UL;

  nrfUart->RXD.PTR = (uint32_t)&rxRcv;
  nrfUart->RXD.MAXCNT = 1;
  nrfUart->TASKS_STARTRX = 0x1UL;

  nrfUart->INTENSET = UARTE_INTENSET_ENDRX_Msk | UARTE_INTENSET_ENDTX_Msk;

  NVIC_ClearPendingIRQ(IRQn);
  NVIC_SetPriority(IRQn, 3);
  NVIC_EnableIRQ(IRQn);

  _end_tx_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(_end_tx_sem);
  _begun = true;
}

void Uart::end()
{
  NVIC_DisableIRQ(IRQn);

  nrfUart->INTENCLR = UARTE_INTENSET_ENDRX_Msk | UARTE_INTENSET_ENDTX_Msk;

  nrfUart->EVENTS_RXTO = 0;
  nrfUart->EVENTS_TXSTOPPED = 0;

  nrfUart->TASKS_STOPRX = 0x1UL;
  nrfUart->TASKS_STOPTX = 0x1UL;

  // Wait for TXSTOPPED event and for RXTO event
  // This is required before disabling UART to fully power down transceiver PHY.
  // Otherwise transceiver will continue to consume ~900uA
  while ( !(nrfUart->EVENTS_TXSTOPPED && nrfUart->EVENTS_RXTO) ) yield();

  nrfUart->ENABLE = UARTE_ENABLE_ENABLE_Disabled;

  nrfUart->PSEL.TXD = 0xFFFFFFFF;
  nrfUart->PSEL.RXD = 0xFFFFFFFF;

  nrfUart->PSEL.RTS = 0xFFFFFFFF;
  nrfUart->PSEL.CTS = 0xFFFFFFFF;

  rxBuffer.clear();

  vSemaphoreDelete(_end_tx_sem);
  _end_tx_sem = NULL;
  _begun = false;
}

void Uart::flush()
{
  if ( _begun ) {
    xSemaphoreTake(_end_tx_sem, portMAX_DELAY);
    xSemaphoreGive(_end_tx_sem);
  }
}

void Uart::IrqHandler()
{
  if (nrfUart->EVENTS_ENDRX)
  {
    nrfUart->EVENTS_ENDRX = 0x0UL;
    if (nrfUart->RXD.AMOUNT)
    {
      rxBuffer.store_char(rxRcv);
    }
    nrfUart->TASKS_STARTRX = 0x1UL;
  }

  if (nrfUart->EVENTS_ENDTX)
  {
    nrfUart->EVENTS_ENDTX = 0x0UL;
    xSemaphoreGiveFromISR(_end_tx_sem, NULL);
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

size_t Uart::write(uint8_t data)
{
  return write(&data, 1);
}

size_t Uart::write(const uint8_t *buffer, size_t size)
{
  if(size == 0) return 0;

  size_t sent = 0;

  do
  {
    size_t remaining = size - sent;
    size_t txSize = min(remaining, (size_t)SERIAL_BUFFER_SIZE);

    xSemaphoreTake(_end_tx_sem, portMAX_DELAY);

    memcpy(txBuffer, buffer + sent, txSize);

    nrfUart->TXD.MAXCNT = txSize;
    nrfUart->TASKS_STARTTX = 0x1UL;
    sent += txSize;

  } while (sent < size);

  return sent;
}

//------------- Serial1 (or Serial in case of nRF52832) -------------//
#ifdef NRF52832_XXAA
  Uart Serial( NRF_UARTE0, UARTE0_UART0_IRQn, PIN_SERIAL_RX, PIN_SERIAL_TX );
#else
  Uart Serial1( NRF_UARTE0, UARTE0_UART0_IRQn, PIN_SERIAL1_RX, PIN_SERIAL1_TX );
#endif

extern "C"
{
  void UARTE0_UART0_IRQHandler()
  {
    SERIAL_PORT_HARDWARE.IrqHandler();
  }
}

//------------- Serial2 -------------//
#if defined(PIN_SERIAL2_RX) && defined(PIN_SERIAL2_TX)
Uart Serial2( NRF_UARTE1, UARTE1_IRQn, PIN_SERIAL2_RX, PIN_SERIAL2_TX );

extern "C"
{
  void UARTE1_IRQHandler()
  {
    Serial2.IrqHandler();
  }
}
#endif

