/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#if defined(USE_FREQ_SENSOR)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/dshot.h"
#include "drivers/freq.h"

#include "pg/freq.h"


// Enable DEBUG
#define FREQ_DEBUG

// Accepted frequence range
#define FREQ_RANGE_MIN        10.0
#define FREQ_RANGE_MAX        10000.0

// Scaling limits
#define FREQ_SCALE_MIN        0
#define FREQ_SCALE_MAX        15
#define FREQ_SCALE_DEF        12
#define FREQ_SCALE_CNT        (FREQ_SCALE_MAX + 1)

// Prescaler shift points
#define FREQ_SHIFT_MIN        0x1000
#define FREQ_SHIFT_MAX        0x4000

// Period init value
#define FREQ_PERIOD_INIT      0x2000

// Freq filtering coefficient - 6 is best for 3-phase motor
#define FREQ_FILTER_COEFF     6

// Maximum number of overflow failures
#define FREQ_MAX_FAILURES     3

// Input signal max deviation from average 75%..150%
#define FREQ_PERIOD_MIN(p)    ((p)*3/4)
#define FREQ_PERIOD_MAX(p)    ((p)*3/2)

// Filter macros
#define FILTER_UPDATE(_var,_value,_coef) \
    ((_var) += ((_value)-(_var))/(_coef))

#define UPDATE_FREQ_FILTER(_input,_freq) \
    FILTER_UPDATE((_input)->freq, _freq, FREQ_FILTER_COEFF)

#define UPDATE_PERIOD_FILTER(_input,_period) \
    FILTER_UPDATE((_input)->period, (int32_t)_period, (_input)->perfilt)


typedef struct {

    bool enabled;

    float freq;
    float clock;

    uint8_t  findex;

    int32_t  period;
    int32_t  perfilt;
    uint32_t capture;
    uint32_t failures;
    uint32_t overflows;
    uint32_t rejections;

    timerCCHandlerRec_t edgeCb;
    timerOvrHandlerRec_t overflowCb;

    const timerHardware_t *timerHardware;
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef *timerHandle;
#endif

} freqInputPort_t;

static FAST_RAM_ZERO_INIT freqInputPort_t freqInputPorts[FREQ_SENSOR_PORT_COUNT];


static inline void freqDebug(freqInputPort_t *input)
{
#ifdef FREQ_DEBUG
    DEBUG_SET(DEBUG_FREQ_SENSOR, 0, input->freq);
    DEBUG_SET(DEBUG_FREQ_SENSOR, 1, input->period);
    DEBUG_SET(DEBUG_FREQ_SENSOR, 2, input->findex);
    DEBUG_SET(DEBUG_FREQ_SENSOR, 3, input->rejections);
#else
    UNUSED(input);
#endif
}

/*
 * Set the base clock to a frequency that gives a reading in range
 * RANGE_MIN..RANGE_MAX [0x1000..0x4000]. This gives enough resolution,
 * while allowing the signal to change four times slower or faster in one cycle.
 *
 * Set the input prescaler so that a "typical" motor speed will use
 * the divider depending on the #define FREQ_SENSOR_DIV
 *
 * For example, a 6-pole 1800KV motor @ 22.2V has eRPM or 2kHz.
 * Depending on the #define FREQ_SENSOR_DIV, this would give an update
 * speed of 2kHz, 1kHz, 500Hz or 250Hz.
 *
 * Also, set the period filter coefficient so that it allows very quick change
 * on low frequencies, but slower change on higher. This is needed because
 * electric motors have lots of torque on low speeds, especially when starting up.
 * We need to be able to adjust to the startup quickly enough.
 */

#ifdef USE_HAL_DRIVER
#define ICPSC_DIV1  LL_TIM_ICPSC_DIV1
#define ICPSC_DIV2  LL_TIM_ICPSC_DIV2
#define ICPSC_DIV4  LL_TIM_ICPSC_DIV4
#define ICPSC_DIV8  LL_TIM_ICPSC_DIV8
#else
#define ICPSC_DIV1  TIM_ICPSC_DIV1
#define ICPSC_DIV2  TIM_ICPSC_DIV2
#define ICPSC_DIV4  TIM_ICPSC_DIV4
#define ICPSC_DIV8  TIM_ICPSC_DIV8
#endif

typedef struct {
    uint16_t  PSC;
    uint32_t  ICPSC;
    uint16_t  FK;
} freqScaling_t;

static const freqScaling_t fScale[FREQ_SCALE_CNT] =
{   //        PSC      ICPSC        Fk       Fkk    eRPM (Hz)   IRQ period
    [ 0] = {  0x0001,  ICPSC_DIV8,  4 },  // 32     211k        38us
    [ 1] = {  0x0002,  ICPSC_DIV8,  4 },  // 32     105k        76us
    [ 2] = {  0x0004,  ICPSC_DIV8,  4 },  // 32     52.7k       0.15ms
    [ 3] = {  0x0008,  ICPSC_DIV8,  3 },  // 24     26.4k       0.30ms
    [ 4] = {  0x0010,  ICPSC_DIV8,  2 },  // 16     13.1k       0.61ms
    [ 5] = {  0x0020,  ICPSC_DIV8,  1 },  // 8      6.6k        1.2ms
    [ 6] = {  0x0020,  ICPSC_DIV4,  2 },  // 8      3.2k        1.2ms
    [ 7] = {  0x0020,  ICPSC_DIV2,  4 },  // 8      1648        1.2ms
    [ 8] = {  0x0020,  ICPSC_DIV1,  4 },  // 4      824         1.2ms
    [ 9] = {  0x0040,  ICPSC_DIV1,  4 },  // 4      412         2.4ms
    [10] = {  0x0080,  ICPSC_DIV1,  3 },  // 3      206         4.9ms
    [11] = {  0x0100,  ICPSC_DIV1,  2 },  // 2      103         9.7ms
    [12] = {  0x0200,  ICPSC_DIV1,  2 },  // 2      51.5        19ms
    [13] = {  0x0400,  ICPSC_DIV1,  2 },  // 2      25.7        39ms
    [14] = {  0x0800,  ICPSC_DIV1,  2 },  // 2      12.9        77ms
    [15] = {  0x1000,  ICPSC_DIV1,  2 },  // 2      6.4         0.15s
};

static void freqSetScale(freqInputPort_t *input, uint8_t findex)
{
    TIM_TypeDef *tim = input->timerHardware->tim;

    input->findex = findex;
    input->perfilt = fScale[findex].FK;

    tim->PSC = fScale[findex].PSC - 1;
    tim->EGR = TIM_EGR_UG;

#if defined(USE_HAL_DRIVER)
    if (input->timerHardware->channel == TIM_CHANNEL_1)
        LL_TIM_IC_SetPrescaler(tim, LL_TIM_CHANNEL_CH1, fScale[findex].ICPSC);
    else if (input->timerHardware->channel == TIM_CHANNEL_2)
        LL_TIM_IC_SetPrescaler(tim, LL_TIM_CHANNEL_CH2, fScale[findex].ICPSC);
    else if (input->timerHardware->channel == TIM_CHANNEL_3)
        LL_TIM_IC_SetPrescaler(tim, LL_TIM_CHANNEL_CH3, fScale[findex].ICPSC);
    else if (input->timerHardware->channel == TIM_CHANNEL_4)
        LL_TIM_IC_SetPrescaler(tim, LL_TIM_CHANNEL_CH4, fScale[findex].ICPSC);
#else
    if (input->timerHardware->channel == TIM_Channel_1)
        TIM_SetIC1Prescaler(tim, fScale[findex].ICPSC);
    else if (input->timerHardware->channel == TIM_Channel_2)
        TIM_SetIC2Prescaler(tim, fScale[findex].ICPSC);
    else if (input->timerHardware->channel == TIM_Channel_3)
        TIM_SetIC3Prescaler(tim, fScale[findex].ICPSC);
    else if (input->timerHardware->channel == TIM_Channel_4)
        TIM_SetIC4Prescaler(tim, fScale[findex].ICPSC);
#endif
}

static void freqReset(freqInputPort_t *input)
{
    input->freq = 0.0f;
    input->period = FREQ_PERIOD_INIT;
    input->capture = 0;
    input->failures = 0;
    input->overflows = 0;

    freqSetScale(input, FREQ_SCALE_DEF);
    freqDebug(input);
}

static FAST_CODE void freqEdgeCallback(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    freqInputPort_t *input = container_of(cbRec, freqInputPort_t, edgeCb);

    if (input->capture) {

        // Must use uint16 here because of wraparound
        uint16_t period = capture - input->capture;

        UPDATE_PERIOD_FILTER(input, period);

        // Signal conditioning. Update freq filter only if period within acceptable range.
        if (period > FREQ_PERIOD_MIN(input->period) && period < FREQ_PERIOD_MAX(input->period)) {
            float freq = input->clock / ((uint32_t)period << input->findex);
            if (freq > FREQ_RANGE_MIN && freq < FREQ_RANGE_MAX) {
                UPDATE_FREQ_FILTER(input, freq);
            }
        }
        else {
            input->rejections++;
        }

        freqDebug(input);

        // Filtered period out of range. Change prescaler.
        if (input->period < FREQ_SHIFT_MIN && input->findex > FREQ_SCALE_MIN) {
            freqSetScale(input, input->findex - 1);
            input->period <<= 1;
            capture = 0;
        }
        else if (input->period > FREQ_SHIFT_MAX && input->findex < FREQ_SCALE_MAX) {
            freqSetScale(input, input->findex + 1);
            input->period >>= 1;
            capture = 0;
        }

        input->failures = 0;
    }

    input->overflows = 0;
    input->capture = capture;
}

static FAST_CODE void freqOverflowCallback(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    freqInputPort_t *input = container_of(cbRec, freqInputPort_t, overflowCb);

    input->overflows++;

    // Two overflows means no signal for a whole period
    if (input->overflows > 1) {

        input->failures++;

        // Reset after too many dead periods
        if (input->failures > FREQ_MAX_FAILURES) {
            freqReset(input);
        }

        input->overflows = 0;
        input->capture = 0;
    }
}

#if defined(USE_HAL_DRIVER)
void freqICConfig(freqInputPort_t *input, bool rising, uint16_t filter)
{
    const timerHardware_t *timer = input->timerHardware;

    TIM_IC_InitTypeDef sInitStructure;

    memset(&sInitStructure, 0, sizeof(sInitStructure));
    sInitStructure.ICPolarity = rising ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
    sInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
    sInitStructure.ICFilter = filter;
    HAL_TIM_IC_ConfigChannel(input->timerHandle, &sInitStructure, timer->channel);
    HAL_TIM_IC_Start_IT(input->timerHandle, timer->channel);
}
#else
void freqICConfig(freqInputPort_t *input, bool rising, uint16_t filter)
{
    const timerHardware_t *timer = input->timerHardware;

    TIM_ICInitTypeDef sInitStructure;

    TIM_ICStructInit(&sInitStructure);
    sInitStructure.TIM_Channel = timer->channel;
    sInitStructure.TIM_ICPolarity = rising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    sInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    sInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    sInitStructure.TIM_ICFilter = filter;

    TIM_ICInit(timer->tim, &sInitStructure);
}
#endif

void freqInit(const freqConfig_t *freqConfig)
{
    for (int port = 0; port < FREQ_SENSOR_PORT_COUNT; port++) {
        freqInputPort_t *input = &freqInputPorts[port];
        const timerHardware_t *timer = timerAllocate(freqConfig->ioTag[port], OWNER_FREQ, RESOURCE_INDEX(port));
        if (timer) {
            input->timerHardware = timer;
#if defined(USE_HAL_DRIVER)
            input->timerHandle = timerFindTimerHandle(timer->tim);
#endif
            IO_t io = IOGetByTag(freqConfig->ioTag[port]);
            IOInit(io, OWNER_FREQ, RESOURCE_INDEX(port));
            IOConfigGPIOAF(io, IOCFG_AF_PP_PD, timer->alternateFunction);

            timerConfigure(timer, 0, timerClock(timer->tim));

            timerChCCHandlerInit(&input->edgeCb, freqEdgeCallback);
            timerChOvrHandlerInit(&input->overflowCb, freqOverflowCallback);
            timerChConfigCallbacks(timer, &input->edgeCb, &input->overflowCb);

            freqICConfig(input, true, 4);
            freqReset(input);

            input->clock = 8 * timerClock(input->timerHardware->tim);

            input->enabled = true;
        }
    }
}

// RTFL: The freq sensor number MUST match the motor number.
// The resource configuration should reflect this requirement.

float FAST_CODE freqRead(uint8_t port)
{
    if (port < FREQ_SENSOR_PORT_COUNT) {
        return freqInputPorts[port].freq;
    }
    return 0.0f;
}

uint16_t FAST_CODE getFreqSensorRPM(uint8_t port)
{
    if (port < FREQ_SENSOR_PORT_COUNT) {
        // Return eRPM/100 as expected by RPM filter, msp, etc.
        return (uint16_t) (freqInputPorts[port].freq * 60.0f / 100.0f);
    }
    return 0;
}

bool isFreqSensorPortInitialized(uint8_t port)
{
    if (port < FREQ_SENSOR_PORT_COUNT) {
        return freqInputPorts[port].enabled;
    }
    return false;
}

// Now, return true if at least one sensor is enabled
bool isFreqSensorInitialized(void)
{
    for (int port = 0; port < FREQ_SENSOR_PORT_COUNT; port++) {
        if (freqInputPorts[port].enabled) {
            return true;
        }
    }
    return false;
}

#endif
