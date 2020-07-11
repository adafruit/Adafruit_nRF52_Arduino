/* Tone.cpp

  A Tone Generator Library

  Written by Brett Hagman

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Version Modified By Date     Comments
------- ----------- -------- --------
0001    B Hagman    09/08/02 Initial coding
0002    B Hagman    09/08/18 Multiple pins
0003    B Hagman    09/08/18 Moved initialization from constructor to begin()
0004    B Hagman    09/09/26 Fixed problems with ATmega8
0005    B Hagman    09/11/23 Scanned prescalars for best fit on 8 bit timers
                    09/11/25 Changed pin toggle method to XOR
                    09/11/25 Fixed timer0 from being excluded
0006    D Mellis    09/12/29 Replaced objects with functions
0007    M Sproul    10/08/29 Changed #ifdefs from cpu to register
0008    S Kanemoto  12/06/22 Fixed for Leonardo by @maris_HY
0009    Arduino.org 15/06/30 Add M0/M0 Pro support
0010	Arduino.org 16/07/27 Added Arduino Primo support
*************************************************/

#include "Arduino.h"
#include "Tone.h"
#include "WVariant.h"

volatile unsigned long int count_duration=0;
volatile bool no_stop = false;
uint8_t pin_sound=0;
static uintptr_t _toneToken = 0x656e6f54; // 'T' 'o' 'n' 'e'

// NOTE: Currently hard-coded to only use PWM2 ...
//       These are the relvant hard-coded variables
//       (plus the ISR PWM2_IRQHandler)
static IRQn_Type      const _IntNo       = PWM2_IRQn;
static NRF_PWM_Type * const _PWMInstance = NRF_PWM2;
static HardwarePWM  * const _HwPWM       = HwPWMx[2];

void tone(uint8_t pin, unsigned int frequency, unsigned long duration)
{
	bool new_no_stop;
	unsigned long int new_count_duration = (unsigned long int)-1L;
	unsigned int time_per=0;

	// limit frequency to reasonable audible range	
	if((frequency < 20) | (frequency > 25000)) {
		LOG_LV1("TON", "frequency outside range [20..25000] -- ignoring");
		return;
	}

	// set nostop to true to avoid race condition.
	// Specifically, race between a tone finishing
	// after checking for ownership (which releases ownership)
	// No effect if a tone is not playing....
	no_stop = true;

	// Use fixed PWM2 (due to need to connect interrupt)
	if (!_HwPWM->isOwner(_toneToken) &&
	    !_HwPWM->takeOwnership(_toneToken)) {
		LOG_LV1("TON", "unable to allocate PWM2 to Tone");
		return;
	}

	float per=float(1)/frequency;
	time_per=per/0.000008;
	unsigned int duty=time_per/2;
	if(duration > 0) {
		new_no_stop = false;
		float mil=float(duration)/1000;
 		if(per>mil) {
			new_count_duration = 1;
		} else {
			new_count_duration = mil/per;
		}
	} else {
		new_no_stop = true;
	}

	// Configure PWM
	static uint16_t seq_values[]={0};
	//In each value, the most significant bit (15) determines the polarity of the output
	//0x8000 is MSB = 1
	seq_values[0]= duty | 0x8000;
	nrf_pwm_sequence_t const seq={
								seq_values,
								NRF_PWM_VALUES_LENGTH(seq_values),
								0,
								0
    };

	uint32_t pins[NRF_PWM_CHANNEL_COUNT]={NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED};
	pins[0] = g_ADigitalPinMap[pin];

	// enable interrupt
	count_duration = 0x6FFF; // large enough to avoid hitting zero in next few lines
	no_stop = new_no_stop;

	nrf_pwm_pins_set(_PWMInstance, pins);
	nrf_pwm_enable(_PWMInstance);
	nrf_pwm_configure(_PWMInstance, NRF_PWM_CLK_125kHz, NRF_PWM_MODE_UP, time_per);
	nrf_pwm_decoder_set(_PWMInstance, NRF_PWM_LOAD_COMMON, NRF_PWM_STEP_AUTO);
	nrf_pwm_sequence_set(_PWMInstance, 0, &seq);
	nrf_pwm_shorts_enable(_PWMInstance, NRF_PWM_SHORT_SEQEND0_STOP_MASK); // shortcut for when SEQ0 ends, PWM output will automatically stop
	nrf_pwm_event_clear(_PWMInstance, NRF_PWM_EVENT_PWMPERIODEND);
	nrf_pwm_int_enable(_PWMInstance, NRF_PWM_INT_PWMPERIODEND_MASK);
	NVIC_SetPriority(_IntNo, 6); //low priority
	NVIC_ClearPendingIRQ(_IntNo);
	NVIC_EnableIRQ(_IntNo);
	count_duration = new_count_duration;
	nrf_pwm_task_trigger(_PWMInstance, NRF_PWM_TASK_SEQSTART0);
}


void noTone(uint8_t pin)
{
	if (!_HwPWM->isOwner(_toneToken)) {
		LOG_LV1("TON", "Attempt to set noTone when not the owner of the PWM peripheral.  Ignoring call....");
		return;
	}
	nrf_pwm_task_trigger(_PWMInstance, NRF_PWM_TASK_STOP);
	nrf_pwm_disable(_PWMInstance);
	_PWMInstance->PSEL.OUT[0] = NRF_PWM_PIN_NOT_CONNECTED;
	NVIC_DisableIRQ(_IntNo);
	_HwPWM->releaseOwnership(_toneToken);
	if (_HwPWM->isOwner(_toneToken)) {
		LOG_LV1("TON", "stopped tone, but failed to release ownership of PWM peripheral?");
		return;
	}
}

#ifdef __cplusplus
extern "C"{
#endif	

void PWM2_IRQHandler(void){
	nrf_pwm_event_clear(NRF_PWM2, NRF_PWM_EVENT_PWMPERIODEND);
	if(!no_stop){
		count_duration--;
		if(count_duration == 0) {
			nrf_pwm_task_trigger(NRF_PWM2, NRF_PWM_TASK_STOP);
			nrf_pwm_disable(NRF_PWM2);
			_PWMInstance->PSEL.OUT[0] = NRF_PWM_PIN_NOT_CONNECTED;
			NVIC_DisableIRQ(PWM2_IRQn);
			_HwPWM->releaseOwnership(_toneToken);
		} else {
			nrf_pwm_task_trigger(NRF_PWM2, NRF_PWM_TASK_SEQSTART0);
		}
	} else {
		nrf_pwm_task_trigger(NRF_PWM2, NRF_PWM_TASK_SEQSTART0);
	}
}

#ifdef __cplusplus
}
#endif
