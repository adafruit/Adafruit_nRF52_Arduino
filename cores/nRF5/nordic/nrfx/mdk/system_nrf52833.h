/*

Copyright (c) 2009-2024 ARM Limited. All rights reserved.

    SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the License); you may
not use this file except in compliance with the License.
You may obtain a copy of the License at

    www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an AS IS BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

NOTICE: This file has been modified by Nordic Semiconductor ASA.

*/

#ifndef SYSTEM_NRF52833_H
#define SYSTEM_NRF52833_H
#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  \brief Exception / Interrupt Handler Function Prototype
*/
typedef void(*VECTOR_TABLE_Type)(void);

/**
  \brief System Clock Frequency (Core Clock)
*/
extern uint32_t SystemCoreClock;

/**
  \brief Setup the microcontroller system.
   Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);


/**
  \brief  Update SystemCoreClock variable.
   Updates the SystemCoreClock with current core Clock retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);


#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* SYSTEM_NRF52833_H */
