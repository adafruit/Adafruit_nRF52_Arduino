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
0011 Henry Gabryjelski 20/08/25 Rework/Rewrite the library to use no interrupt handler and
                                support HwPWM ownership
*************************************************/

#include "Arduino.h"
#include "Tone.h"
#include "WVariant.h"
#include "limits.h"

// NOTE: Currently hard-coded to only use PWM2 ...
//       These are the relvant hard-coded variables
// TODO: Consider allowing dynamic use of any available PWM peripheral (but start with #2 for compatibility)
static NRF_PWM_Type * const _PWMInstance = NRF_PWM2;
static HardwarePWM  * const _HwPWM       = HwPWMx[2];

// Defined a struct, to simplify validation testing ... also provides context when debugging
class TonePwmConfig {
    private:
        enum { TONE_TOKEN = 0x656e6f54 }; //< 'T' 'o' 'n' 'e'
        uint64_t pulse_count;         //< total number of PWM pulses
        uint32_t seq0_refresh;        //< count of pulses for each SEQ0 iteration
        uint32_t seq1_refresh;        //< count of pulses for each SEQ1 iteration
        uint16_t loop_count;          //< how many times to restart SEQ0/SEQ1?
        uint16_t time_period;         //< how many clock cycles allocated to each PWM pulse?
        uint16_t duty_with_polarity;  //< SEQ[N].PTR will point here, length == 1
        nrf_pwm_task_t task_to_start; //< Whether to start playback at SEQ0 or SEQ1
        nrf_pwm_short_mask_t shorts;  //< shortcuts to enable

    public:
        bool ensurePwmPeripheralOwnership(void);
        bool initializeFromPulseCountAndTimePeriod(uint64_t pulse_count, uint16_t time_period);
        bool applyConfiguration(uint32_t pin);
        bool startPlayback(void);
        bool stopPlayback(bool releaseOwnership = false);
};
TonePwmConfig _pwm_config;

static bool _is_pwm_enabled(NRF_PWM_Type const * pwm_instance) {
    bool isEnabled =
        (pwm_instance->ENABLE & PWM_ENABLE_ENABLE_Msk) ==
        (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    return isEnabled;
}

/*  These two functions were verified as equivalent to
    the prior calculations (except where the old floating-point
    based code resulted in rounding errors ... and thus there
    are some one-off differences ... but the integer-based
    functions are more accurate).
    
    These functions entirely avoid floating point math, and all
    the nasty edge cases they can create... (NaN, INF, etc.)

    See https://gist.github.com/henrygab/6b570ebd51354bf247633c72b8dc383b
    for code that compares the new lambdas to the old calculations.
*/
constexpr static uint16_t _calculate_time_period(uint32_t frequency) {
    // range for frequency == [20..25000],
    // so range of result  == [ 5..62500]
    // which fits in 16 bits.
    return 125000 / frequency;
};

constexpr static uint64_t _calculate_pulse_count(uint32_t frequency, uint32_t duration) {
    // range for frequency == [20..25000],
    // range for duration  == [ 1..0xFFFF_FFFF]
    // so range of result  == [ 1..0x18_FFFF_FFE7] (requires 37 bits)
    return
        (duration == 0) ?
            0 :
        ((duration < 1000ULL) && (duration * frequency < 1000ULL)) ?
            1ULL :
        (UINT64_MAX / frequency < duration) ?
            (duration / 1000ULL) * frequency :
        (((uint64_t)duration) * frequency / 1000ULL);
};

static int _bits_used(unsigned long      x) {
    if (0 == x) return 0;
    unsigned int result = 0;
    do {
        result++;
    } while (x >>= 1);
    return result;
}

static int _bits_used(unsigned long long x) {
    if (0 == x) return 0;
    unsigned int result = 0;
    do {
        result++;
    } while (x >>= 1);
    return result;
}

/*
* In older versions of the BSP, tone() would cause an interrupt for every cycle, tracking whether
* the playback should be infinite or duration-based via two global, volatile variables, "nostop"
* and "count_duration".  The interrupt would then trigger the STARTSEQ0 task, or stop playback
* by calling noTone().
* 
* What is available to configure looping with minimal memory usage:
* 1. SEQ[n].REFRESH === how many PWM period before loading next sample from sequence
*                       Thus, setting to 99 will cause 100 pulses per item
*                       Treat as a 23-bit value.
* 2. LOOP           === how many times to loop back to SEQ[0]
*                       SEQ[0] will play the same count if started PWM at SEQ[0]
*                       SEQ[0] will play one fewer times if started PWM at SEQ[1]
*                       Treat as a 15-bit value.
*
* Therefore, between REFRESH and LOOP, can support up to 40-bit repeat WITHOUT INTERRUPTS.
*
* The value of duration is given in milliseconds.
* Frequency is limited to range [20 ... 25000].
* 
* Therefore, maximum pulse count is limited as follows:
*     (32-bit duration) * (20 ... 25000) / 1000UL
*     (0xFFFF_FFFF) * 25
*     0x18_FFFF_FFE7 ... which is 37 bits
* 
* Therefore, all possible values for tone() can be supported
* via a single one-time configuration of the PWM peripheral.
*
* PWM peripheral can be de-allocated by sketch call to noTone().
* 
* Design:
* 0. At each call to tone(), unconditionally stop any existing playback.
* 1. For infinite duration, configure large REFRESH and LOOP (to minimize reading of RAM / AHB traffic),
*    and setup shortcut to repeat infinitely.
* 2. For specified duration, configure both SEQ0 and SEQ1:
*    1a. SEQ[1].REFRESH <-- total count % _iterations_per_loop
*    1b. SEQ[0].REFRESH <-- _iterations_per_loop - SEQ[1].CNT
*    1c. LOOP           <-- duration_count / _iterations_per_loop
* 
* Result: Zero CPU usage, minimal AHB traffic
*/
void tone(uint8_t pin, unsigned int frequency, unsigned long duration)
{
    // limit frequency to reasonable audible range	
    if((frequency < 20) | (frequency > 25000)) {
        LOG_LV1("Tone", "frequency outside range [20..25000] -- ignoring");
        return;
    }
    uint64_t pulse_count = _calculate_pulse_count(frequency, duration);
    uint16_t time_period = _calculate_time_period(frequency);
    if (!_pwm_config.ensurePwmPeripheralOwnership()) {
        LOG_LV1("Tone", "Unable to acquire PWM peripheral");
    } else if (!_pwm_config.stopPlayback(false)) {
        LOG_LV1("Tone", "Unable to stop playback");
    } else if (!_pwm_config.initializeFromPulseCountAndTimePeriod(pulse_count, time_period)) {
        LOG_LV1("Tone", "Failed calculating configuration");
    } else if (!_pwm_config.applyConfiguration(pin)) {
        LOG_LV1("Tone", "Failed applying configuration");
    } else if (!_pwm_config.startPlayback()) {
        LOG_LV1("Tone", "Failed attempting to start PWM peripheral");
    } else {
        //LOG_LV2("Tone", "Started playback of tone at frequency %d duration %ld", frequency, duration);
    }
    return;
}

void noTone(uint8_t pin)
{
    ( void )pin; // avoid unreferenced parameter compiler warning
    _pwm_config.stopPlayback(true); // release ownership of PWM peripheral
}

bool TonePwmConfig::ensurePwmPeripheralOwnership(void) {
    if (!_HwPWM->isOwner(TonePwmConfig::TONE_TOKEN) && !_HwPWM->takeOwnership(TonePwmConfig::TONE_TOKEN)) {
        LOG_LV1("Tone", "unable to allocate PWM2 to Tone");
        return false;
    }
    return true;
}

//
// The final loop's SEQ1 will ALWAYS output one pulse ...
// In other words,  SEQ1's refresh is *IGNORED* for the final loop.
//
// Visually, with each sequence length set to one as is done with tone():
// ======================================================================
//
// Starting at SEQ0, loopCnt = 2, SEQ0 refresh == 4, SEQ1 refresh = 2:
//
// [----SEQ0-----] [SEQ1-] [----SEQ0-----] [1]
//  0   0   0   0   1   1   0   0   0   0   1
//
// ======================================================================
//
// Starting as SEQ1, loopCnt = 2, SEQ0 refresh == 4, SEQ1 refresh = 2:
//
// [SEQ1-] [----SEQ0-----] [1]
//  1   1   0   0   0   0   1
//
// ======================================================================
//
// Therefore, the total count of pulses that will be emitted by the
// PWM peripheral (per the configuration of tone() API):
//
// COUNT  = (SEQ0.CNT * (SEQ0.REFRESH+1);
// COUNT += (SEQ1.CNT * (SEQ1.REFRESH+1);
// COUNT *= (loopCount-1);  // the number of times SEQ0 and SEQ1 both run
// COUNT += (start at SEQ0) ? (SEQ0.CNT * (SEQ0.REFRESH+1) : 0;
// COUNT += 1; // final SEQ1 emits single pulse
//
bool TonePwmConfig::initializeFromPulseCountAndTimePeriod(uint64_t pulse_count_x, uint16_t time_period) {

    if (_bits_used(pulse_count_x) > 37) {
        LOG_LV1("Tone", "pulse count is limited to 37 bit long value");
        return false;
    }

    this->pulse_count = pulse_count_x;
    this->time_period = time_period;
    this->duty_with_polarity = 0x8000U | (time_period / 2U);

    if (this->pulse_count == 0) {
        this->seq0_refresh  = 0xFFFFFFU; // 24-bit maximum value
        this->seq1_refresh  = 0xFFFFFFU; // 24-bit maximum value
        this->loop_count    = 0xFFFFU;   // 16-bit maximum value
        this->task_to_start = NRF_PWM_TASK_SEQSTART0;
        this->shorts        = NRF_PWM_SHORT_LOOPSDONE_SEQSTART0_MASK;
    }
    else if (this->pulse_count == 1) {
        // yes, this is an edge case; e.g., frequency == 100, duration == 100 causes this
        this->seq0_refresh  = 0;
        this->seq1_refresh  = 0;
        this->loop_count    = 1;
        this->task_to_start = NRF_PWM_TASK_SEQSTART1;
        this->shorts        = NRF_PWM_SHORT_LOOPSDONE_STOP_MASK;
    }
    else {
        // This is the interesting section.
        //
        // To ensure refresh value stays within 24 bits, the maximum number of bits
        // for the pulse_count is ((24 * 3) / 2) + 1 == (72/2) + 1 == 36 + 1 == 37.
        //
        // Validation:
        //   37 * 2 / 3 == 74 / 3 == 24 (OK)    -- leaves 13 bits for loop count
        //   38 * 2 / 3 == 76 / 3 == 25 (fail)  -- leaves 13 bits for loop count
        //
        // NOTE: theoretically possible to support up to 40 bit pulse count, but
        //       would require more complex logic.
        //
        unsigned int bits_needed    = _bits_used(this->pulse_count); // bits_used is now in range [2..37]

        // split the number of bits between refresh and loop_count in 2:1 ratio
        // so that, no matter what inputs are given, guaranteed to have interrupt-free solution
        unsigned int bits_for_refresh    = bits_needed * 2 / 3;             // range is [1 .. 24]
        //unsigned int bits_for_loop_count = bits_needed - bits_for_refresh;  // range is [1 .. 13]

        // NOTE: Due to final SEQ1 outputting exactly one pulse, may need one additional bit for loop count
        //       ... but that will still be within the 16 bits available, because top of range is 13 bits.

        // now determine how many PWM pulses should occur per loop (when both SEQ0 and SEQ1 are played)
        uint32_t total_refresh_count = 1 << bits_for_refresh; // range is [2 .. 2^24]
        uint32_t full_loops          = (this->pulse_count - 1) / total_refresh_count; // == loopCount - 1

        // if (pulses - 1) % total_refresh_count == 0, then start at SEQ1 and split refresh evenly
        // else, start at SEQ0, and set SEQ0 to extra pulses needed...
        uint32_t extraPulsesNeededIfStartingAtSequence1 = (this->pulse_count - 1) % total_refresh_count;
        uint32_t seq0_count;

        if (extraPulsesNeededIfStartingAtSequence1 == 0) {
            seq0_count = total_refresh_count / 2; // range is [1 .. 2^23]
            this->task_to_start = NRF_PWM_TASK_SEQSTART1;
        }
        else {
            seq0_count = extraPulsesNeededIfStartingAtSequence1;
            this->task_to_start = NRF_PWM_TASK_SEQSTART0;
        }
        this->loop_count   = full_loops + 1;
        this->seq0_refresh = seq0_count - 1;
        this->seq1_refresh = (total_refresh_count - seq0_count) - 1;
        this->shorts        = NRF_PWM_SHORT_LOOPSDONE_STOP_MASK;
    }
    return true;
}

bool TonePwmConfig::applyConfiguration(uint32_t pin) {
    if (pin >= PINS_COUNT) {
        return false;
    }
    if (!this->ensurePwmPeripheralOwnership()) {
        return false;
    }
    this->stopPlayback(false);

    uint32_t pins[NRF_PWM_CHANNEL_COUNT] = {
        g_ADigitalPinMap[pin],
        NRF_PWM_PIN_NOT_CONNECTED,
        NRF_PWM_PIN_NOT_CONNECTED,
        NRF_PWM_PIN_NOT_CONNECTED
    };

    nrf_pwm_pins_set(_PWMInstance, pins); // must set pins before enabling
    nrf_pwm_enable(_PWMInstance);
    nrf_pwm_configure(_PWMInstance, NRF_PWM_CLK_125kHz, NRF_PWM_MODE_UP, this->time_period);
    nrf_pwm_decoder_set(_PWMInstance, NRF_PWM_LOAD_COMMON, NRF_PWM_STEP_AUTO);
    nrf_pwm_shorts_set(_PWMInstance, this->shorts);
    nrf_pwm_int_set(_PWMInstance, 0);

    nrf_pwm_seq_ptr_set(_PWMInstance, 0, &this->duty_with_polarity);
    nrf_pwm_seq_ptr_set(_PWMInstance, 1, &this->duty_with_polarity);
    nrf_pwm_seq_cnt_set(_PWMInstance, 0, 1);
    nrf_pwm_seq_cnt_set(_PWMInstance, 1, 1);

    nrf_pwm_seq_refresh_set(_PWMInstance, 0, seq0_refresh);
    nrf_pwm_seq_refresh_set(_PWMInstance, 1, seq1_refresh);
    nrf_pwm_seq_end_delay_set(_PWMInstance, 0, 0);
    nrf_pwm_seq_end_delay_set(_PWMInstance, 1, 0);
    nrf_pwm_loop_set(_PWMInstance, loop_count);

    nrf_pwm_event_clear(_PWMInstance, NRF_PWM_EVENT_STOPPED);
    nrf_pwm_event_clear(_PWMInstance, NRF_PWM_EVENT_SEQSTARTED0);
    nrf_pwm_event_clear(_PWMInstance, NRF_PWM_EVENT_SEQSTARTED1);
    nrf_pwm_event_clear(_PWMInstance, NRF_PWM_EVENT_SEQEND0);
    nrf_pwm_event_clear(_PWMInstance, NRF_PWM_EVENT_SEQEND1);
    nrf_pwm_event_clear(_PWMInstance, NRF_PWM_EVENT_PWMPERIODEND);
    nrf_pwm_event_clear(_PWMInstance, NRF_PWM_EVENT_LOOPSDONE);
    return true;
}

bool TonePwmConfig::startPlayback(void) {
    if (!this->ensurePwmPeripheralOwnership()) {
        LOG_LV1("Tone", "PWM peripheral not available for playback");
        return false;
    }
    nrf_pwm_task_trigger(_PWMInstance, this->task_to_start);
    return true;
}

bool TonePwmConfig::stopPlayback(bool releaseOwnership) {

    if (!_HwPWM->isOwner(TonePwmConfig::TONE_TOKEN)) {
      LOG_LV2("Tone", "Attempt to set noTone when not the owner of the PWM peripheral.  Ignoring call....");
      return false;
    }
    // ensure stopped and then disable
    if (_is_pwm_enabled(_PWMInstance)) {
        nrf_pwm_task_trigger(_PWMInstance, NRF_PWM_TASK_STOP);
        nrf_pwm_disable(_PWMInstance);
        _PWMInstance->PSEL.OUT[0] = NRF_PWM_PIN_NOT_CONNECTED;
    }

    if (releaseOwnership) {
        _HwPWM->releaseOwnership(TonePwmConfig::TONE_TOKEN);
    }

    return true;
}
