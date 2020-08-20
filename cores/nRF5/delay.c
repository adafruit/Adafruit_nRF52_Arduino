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

#include "nrf.h"

#include "delay.h"
#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t millis( void )
{
  return tick2ms(xTaskGetTickCount());
}

void delay( uint32_t ms )
{
  uint32_t ticks = ms2tick(ms);

#ifdef USE_TINYUSB
  // Take chance to flush usb cdc
  uint32_t flush_tick = xTaskGetTickCount();
  tud_cdc_write_flush();

  flush_tick = xTaskGetTickCount()-flush_tick;
  if (flush_tick >= ticks) return;

  ticks -= flush_tick;
#endif

  vTaskDelay(ticks);
}

void dwt_enable(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* Global Enable for DWT */
  DWT->CYCCNT = 0;                                /* Reset the counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /* Enable cycle counter */
}

void dwt_disable(void)
{
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
}


#ifdef __cplusplus
}
#endif
