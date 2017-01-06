/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "app_util.h"

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM4F port.
 * CMSIS compatible layer to menage SysTick ticking source.
 *----------------------------------------------------------*/

#if configTICK_SOURCE == FREERTOS_USE_SYSTICK

#ifdef SOFTDEVICE_PRESENT
#include "nrf_soc.h"
#endif

#ifndef configSYSTICK_CLOCK_HZ
    #define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
    /* Ensure the SysTick is clocked at the same frequency as the core. */
    #define portNVIC_SYSTICK_CLK_BIT    ( SysTick_CTRL_CLKSOURCE_Msk )
#else
    /* The way the SysTick is clocked is not modified in case it is not the same
    as the core. */
    #define portNVIC_SYSTICK_CLK_BIT    ( 0 )
#endif


#if configUSE_TICKLESS_IDLE == 1
    #error SysTick port for RF52 does not support tickless idle. Use RTC mode instead.
#endif /* configUSE_TICKLESS_IDLE */

/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
    /* The SysTick runs at the lowest interrupt priority, so when this interrupt
    executes all interrupts must be unmasked.  There is therefore no need to
    save and then restore the interrupt mask value as its value is already
    known. */
    ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
    {
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            /* A context switch is required.  Context switching is performed in
            the PendSV interrupt.  Pend the PendSV interrupt. */
            SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
            __SEV();
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );
}

/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
void vPortSetupTimerInterrupt( void )
{
    /* Set interrupt priority */
    NVIC_SetPriority(SysTick_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    /* Configure SysTick to interrupt at the requested rate. */
    SysTick->LOAD = ROUNDED_DIV(configSYSTICK_CLOCK_HZ, configTICK_RATE_HZ) - 1UL;
    SysTick->CTRL = ( portNVIC_SYSTICK_CLK_BIT | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk );
}

/*-----------------------------------------------------------*/

#elif configTICK_SOURCE == FREERTOS_USE_RTC

#if configUSE_16_BIT_TICKS == 1
#error This port does not support 16 bit ticks.
#endif

#include "nrf_rtc.h"
#include "nrf_drv_clock.h"


/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
    nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_TICK);
#if configUSE_TICKLESS_IDLE == 1
    nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_COMPARE_0);
#endif

    /* The SysTick runs at the lowest interrupt priority, so when this interrupt
    executes all interrupts must be unmasked.  There is therefore no need to
    save and then restore the interrupt mask value as its value is already
    known. */
    ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
    do{
#if configUSE_TICKLESS_IDLE == 1
        TickType_t diff;
        TickType_t actualTicks = xTaskGetTickCountFromISR();
        TickType_t hwTicks     = nrf_rtc_counter_get(portNRF_RTC_REG);

        diff = (hwTicks - actualTicks) & portNRF_RTC_MAXTICKS;
        if(diff <= 0)
        {
            break;
        }

        if(diff > 2) // Correct internat ticks
        {
            vTaskStepTick(diff - 1);
        }
#endif
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            /* A context switch is required.  Context switching is performed in
            the PendSV interrupt.  Pend the PendSV interrupt. */
            SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
            __SEV();
        }
    }while(0);
    portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );
}

/*
 * Setup the RTC time to generate the tick interrupts at the required
 * frequency.
 */
void vPortSetupTimerInterrupt( void )
{
    /* Request LF clock */
    nrf_drv_clock_lfclk_request(NULL);

    /* Configure SysTick to interrupt at the requested rate. */
    nrf_rtc_prescaler_set(portNRF_RTC_REG, portNRF_RTC_PRESCALER);
    nrf_rtc_int_enable   (portNRF_RTC_REG, RTC_INTENSET_TICK_Msk);
    nrf_rtc_task_trigger (portNRF_RTC_REG, NRF_RTC_TASK_CLEAR);
    nrf_rtc_task_trigger (portNRF_RTC_REG, NRF_RTC_TASK_START);

    NVIC_SetPriority(portNRF_RTC_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(portNRF_RTC_IRQn);
}

#if configUSE_TICKLESS_IDLE == 1

void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    TickType_t wakeupTime;

    /* Make sure the SysTick reload value does not overflow the counter. */
    if( xExpectedIdleTime > portNRF_RTC_MAXTICKS - configEXPECTED_IDLE_TIME_BEFORE_SLEEP )
    {
        xExpectedIdleTime = portNRF_RTC_MAXTICKS - configEXPECTED_IDLE_TIME_BEFORE_SLEEP;
    }
    /* Block the scheduler now */
    portDISABLE_INTERRUPTS();

    /* Stop tick events */
    nrf_rtc_int_disable(portNRF_RTC_REG, NRF_RTC_INT_TICK_MASK);

    /* Configure CTC interrupt */
    wakeupTime = nrf_rtc_counter_get(portNRF_RTC_REG) + xExpectedIdleTime;
    wakeupTime &= portNRF_RTC_MAXTICKS;
    nrf_rtc_cc_set(portNRF_RTC_REG, 0, wakeupTime);
    nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_COMPARE_0);
    nrf_rtc_int_enable(portNRF_RTC_REG, NRF_RTC_INT_COMPARE0_MASK);

    if( eTaskConfirmSleepModeStatus() == eAbortSleep )
    {
        portENABLE_INTERRUPTS();
    }
    else
    {
        TickType_t xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
        if( xModifiableIdleTime > 0 )
        {
            __DSB();
#ifdef SOFTDEVICE_PRESENT
            /* With SD there is no problem with possibility of interrupt lost.
             * every interrupt is counted and the counter is processed inside
             * sd_app_evt_wait function. */
            portENABLE_INTERRUPTS();
            sd_app_evt_wait();
#else
            /* No SD -  we would just block interrupts globally.
             * BASEPRI cannot be used for that because it would prevent WFE from wake up.
             */
            __disable_irq();
            portENABLE_INTERRUPTS();
            do{
                __WFE();
            } while(0 == (NVIC->ISPR[0] | NVIC->ISPR[1]));
            __enable_irq();
#endif
        }
        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );
        portENABLE_INTERRUPTS();
    }
    // We can do operations below safely, because when we are inside vPortSuppressTicksAndSleep
    // scheduler is already suspended.
    nrf_rtc_int_disable(portNRF_RTC_REG, NRF_RTC_INT_COMPARE0_MASK);
    nrf_rtc_int_enable (portNRF_RTC_REG, NRF_RTC_INT_TICK_MASK);
}

#endif // configUSE_TICKLESS_IDLE

#else // configTICK_SOURCE
    #error  Unsupported configTICK_SOURCE value
#endif // configTICK_SOURCE == FREERTOS_USE_SYSTICK
