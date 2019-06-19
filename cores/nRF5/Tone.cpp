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


#include "Tone.h"
#include "WVariant.h"

unsigned long int count_duration=0;
volatile bool no_stop = false;
uint8_t pin_sound=0;


void tone(uint8_t pin, unsigned int frequency, unsigned long duration)
{
	unsigned int time_per=0;
	
	if((frequency < 20) | (frequency > 25000)) return;
	

	float per=float(1)/frequency;
	time_per=per/0.000008;
	unsigned int duty=time_per/2;
	if(duration > 0){
		no_stop = false;
		float mil=float(duration)/1000;
		if(per>mil)
			count_duration=1;
		else
			count_duration= mil/per;
	}
	else
		no_stop = true;

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
	
#if 0
	//assign pin to pwm channel - look at WVariant.h for details about ulPWMChannel attribute
	uint8_t pwm_type=g_APinDescription[pin].ulPWMChannel;
	if(pwm_type == NOT_ON_PWM)
		return;
	
	uint32_t pins[NRF_PWM_CHANNEL_COUNT]={NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED};
	pins[pwm_type & 0x0F]=g_APinDescription[pin].ulPin;
	IRQn_Type IntNo = PWM0_IRQn;
	NRF_PWM_Type * PWMInstance = NRF_PWM0;
	switch(pwm_type &0xF0){
		case 16://0x10
			PWMInstance = NRF_PWM1;
			IntNo = PWM1_IRQn;
			break;
		case 32://0x20
			PWMInstance = NRF_PWM2;
			IntNo = PWM2_IRQn;
			break;
	}
#else
	// Use fixed PWM2, TODO could conflict with other usage
	uint32_t pins[NRF_PWM_CHANNEL_COUNT]={NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED};
	pins[0] = g_ADigitalPinMap[pin];

	IRQn_Type IntNo = PWM2_IRQn;
	NRF_PWM_Type * PWMInstance = NRF_PWM2;
#endif

	nrf_pwm_pins_set(PWMInstance, pins);
	nrf_pwm_enable(PWMInstance);
	nrf_pwm_configure(PWMInstance, NRF_PWM_CLK_125kHz, NRF_PWM_MODE_UP, time_per);
	nrf_pwm_decoder_set(PWMInstance, NRF_PWM_LOAD_COMMON, NRF_PWM_STEP_AUTO);
	nrf_pwm_sequence_set(PWMInstance, 0, &seq);
	nrf_pwm_shorts_enable(PWMInstance, NRF_PWM_SHORT_SEQEND0_STOP_MASK);
	
	// enable interrupt
	nrf_pwm_event_clear(PWMInstance, NRF_PWM_EVENT_PWMPERIODEND);
	nrf_pwm_int_enable(PWMInstance, NRF_PWM_INT_PWMPERIODEND_MASK);
	NVIC_SetPriority(IntNo, 6); //low priority
	NVIC_ClearPendingIRQ(IntNo);
	NVIC_EnableIRQ(IntNo);

	nrf_pwm_task_trigger(PWMInstance, NRF_PWM_TASK_SEQSTART0);
}


void noTone(uint8_t pin)
{
#if 0
	uint8_t pwm_type=g_APinDescription[pin].ulPWMChannel;
	NRF_PWM_Type * PWMInstance = NRF_PWM0;
	switch(pwm_type &0xF0){
		case 16://0x10
			PWMInstance = NRF_PWM1;
			break;
		case 32://0x20
			PWMInstance = NRF_PWM2;
			break;
	}
#else
	NRF_PWM_Type * PWMInstance = NRF_PWM2;
#endif

	nrf_pwm_task_trigger(PWMInstance, NRF_PWM_TASK_STOP);
	nrf_pwm_disable(PWMInstance);
}

#ifdef __cplusplus
extern "C"{
#endif	

#if 0
void PWM0_IRQHandler(void){
	nrf_pwm_event_clear(NRF_PWM0, NRF_PWM_EVENT_PWMPERIODEND);
	if(!no_stop){
		count_duration--;
		if(count_duration == 0)
			noTone(pin_sound);
		else
			nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
	}
	else
		nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
}

void PWM1_IRQHandler(void){
	nrf_pwm_event_clear(NRF_PWM1, NRF_PWM_EVENT_PWMPERIODEND);
	if(!no_stop){
		count_duration--;
		if(count_duration == 0)
			noTone(pin_sound);
		else
			nrf_pwm_task_trigger(NRF_PWM1, NRF_PWM_TASK_SEQSTART0);	
	}
	else
		nrf_pwm_task_trigger(NRF_PWM1, NRF_PWM_TASK_SEQSTART0);
}
#endif

void PWM2_IRQHandler(void){
	nrf_pwm_event_clear(NRF_PWM2, NRF_PWM_EVENT_PWMPERIODEND);
	if(!no_stop){
		count_duration--;
		if(count_duration == 0)
			noTone(pin_sound);
		else
			nrf_pwm_task_trigger(NRF_PWM2, NRF_PWM_TASK_SEQSTART0);	
	}
	else
		nrf_pwm_task_trigger(NRF_PWM2, NRF_PWM_TASK_SEQSTART0);
}

#ifdef __cplusplus
}
#endif
