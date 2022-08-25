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

#include "build/build_config.h"
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/gyro.h"

#include "flight/pid.h"
#include "flight/agc.h"
#include "flight/mixer.h"

#if 0

// 1k delay line for each primary axis
#define AGC_DELAY_LINE   (1<<10)
#define AGC_DELAY_MASK   (AGC_DELAY_LINE-1)

static float dline[4][AGC_DELAY_LINE];

static FAST_RAM_ZERO_INIT uint16_t dindex;

static FAST_RAM_ZERO_INIT float updateRate;

static FAST_RAM_ZERO_INIT float collToYawGain;

static FAST_RAM_ZERO_INIT float ctrlGain[3];

static FAST_RAM_ZERO_INIT uint16_t ctrlDelay;

static FAST_RAM_ZERO_INIT biquadFilter_t iFilter[4];
static FAST_RAM_ZERO_INIT biquadFilter_t oFilter[4];


static inline void agcDelayUpdate(uint8_t axis, float ctrl)
{
    dline[axis][dindex] = ctrl;
    dindex = (dindex + 1) & AGC_DELAY_MASK;
}

static inline float agcGetSample(uint8_t axis, uint16_t delay)
{
    return dline[axis][(dindex - delay) & AGC_DELAY_MASK];
}

void agcUpdateYawPrecompGain(void)
{
    float ctrl = mixerGetInput(MIXER_IN_STABILIZED_YAW) * mixerRotationSign();
    float coll = agcGetSample(3, ctrlDelay);

    float collRate = coll - biquadFilterApply(&iFilter[4], coll);
    float ctrlRate = ctrl - biquadFilterApply(&oFilter[4], ctrl);

    DEBUG_SET(DEBUG_YAW_PRECOMP_AGC, 0, collRate * 10);
    DEBUG_SET(DEBUG_YAW_PRECOMP_AGC, 1, ctrlRate * 10);
    DEBUG_SET(DEBUG_YAW_PRECOMP_AGC, 2, 0);
    DEBUG_SET(DEBUG_YAW_PRECOMP_AGC, 3, collToYawGain * 1000);

    if (fabsf(ctrlRate) > 0.05f && fabsf(collRate) > 0.05f) {
        float ratio = ctrlRate / collRate;
        float cgain = fabsf(collRate * ctrlRate);

        collToYawGain += (ratio - collToYawGain) * updateRate * cgain;

        DEBUG_SET(DEBUG_YAW_PRECOMP_AGC, 2, ratio * 1000);
    }
}

void agcUpdateControlGain(uint8_t axis)
{
    float rate = gyro.gyroADCf[axis] / 360.0f;
    float ctrl = agcGetSample(axis, ctrlDelay) / 360.0f;

    if (axis == FD_YAW)
        ctrl *= mixerRotationSign();

    float ctrlRate = ctrl - biquadFilterApply(&iFilter[axis], ctrl);
    float gyroRate = rate - biquadFilterApply(&oFilter[axis], rate);

    DEBUG_SET(DEBUG_ROLL_GYRO_AGC + axis, 0, ctrlRate * 10);
    DEBUG_SET(DEBUG_ROLL_GYRO_AGC + axis, 1, gyroRate * 10);
    DEBUG_SET(DEBUG_ROLL_GYRO_AGC + axis, 2, 0);
    DEBUG_SET(DEBUG_ROLL_GYRO_AGC + axis, 3, ctrlGain[axis] * 1000);

    if (fabsf(ctrlRate) > 45 && fabsf(gyroRate) > 45) {
        float ratio = gyroRate / ctrlRate;
        float cgain = fabsf(ctrlRate * gyroRate);

        ctrlGain[axis] += (ratio - ctrlGain[axis]) * updateRate * cgain;

        DEBUG_SET(DEBUG_ROLL_GYRO_AGC + axis, 2, ratio * 1000);
    }
}

void agcUpdateGains(void)
{
    agcDelayUpdate(0, mixerGetInput(MIXER_IN_STABILIZED_ROLL));
    agcDelayUpdate(1, mixerGetInput(MIXER_IN_STABILIZED_PITCH));
    agcDelayUpdate(2, mixerGetInput(MIXER_IN_STABILIZED_YAW));
    agcDelayUpdate(3, mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE));

    agcUpdateControlGain(FD_ROLL);
    agcUpdateControlGain(FD_PITCH);
    agcUpdateControlGain(FD_YAW);
}

void agcInit(const pidProfile_t *pidProfile)
{
    ctrlDelay = constrain(lrintf((pidProfile->agc_delay / 1000.0f) / pidGetDT()), 0, AGC_DELAY_MASK);

    updateRate = 0.1f / constrainf(pidProfile->agc_rate, 1, 50000);

    float cutoff = constrainf(pidProfile->agc_freq, 1, 1000) / 10.0f;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        ctrlGain[axis] = 1.0f;
    }

    for (int axis = 0; axis < 4; axis++) {
        biquadFilterInitLPF(&iFilter[axis], cutoff, gyro.targetLooptime);
        biquadFilterInitLPF(&oFilter[axis], cutoff, gyro.targetLooptime);
        for (int i = 0; i<AGC_DELAY_LINE; i++) {
            dline[axis][i] = 0;
        }
    }
}

#else

void agcUpdateGains(void)
{
}

void agcInit(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);
}

#endif
