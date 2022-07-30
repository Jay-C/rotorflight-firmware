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


#define AGC_DELAY_LINE   (1<<10)
#define AGC_DELAY_MASK   (AGC_DELAY_LINE-1)

static float dline[3][AGC_DELAY_LINE];

static FAST_RAM_ZERO_INIT float updateRate;

static FAST_RAM_ZERO_INIT float gain[3];

static FAST_RAM_ZERO_INIT uint16_t dindex;
static FAST_RAM_ZERO_INIT uint16_t dtotal;

static FAST_RAM_ZERO_INIT biquadFilter_t ctrlFilter[3];
static FAST_RAM_ZERO_INIT biquadFilter_t gyroFilter[3];




static float agcDelayUpdate(uint8_t axis, float ctrl)
{
    dline[axis][dindex] = ctrl;

    dindex = (dindex + 1) & AGC_DELAY_MASK;

    return dline[axis][(dindex - dtotal) & AGC_DELAY_MASK];
}


void agcUpdate(uint8_t axis, float ctrl, float gyro)
{
    float ctrlRate = biquadFilterApply(&ctrlFilter[axis], ctrl);
    float gyroRate = biquadFilterApply(&gyroFilter[axis], gyro);

    ctrlRate = agcDelayUpdate(axis, ctrlRate);

    if (axis == FD_PITCH) {
        DEBUG_SET(DEBUG_AGC, 0, ctrlRate * 10);
        DEBUG_SET(DEBUG_AGC, 1, gyroRate * 10);
        DEBUG_SET(DEBUG_AGC, 2, 0);
        DEBUG_SET(DEBUG_AGC, 3, gain[axis] * 1000);
    }

    if (fabsf(ctrlRate) > 45 && fabs(gyroRate) > 45) {
        float ratio = gyroRate / ctrlRate;
        float R = fabsf(ctrlRate / 360);

        if (ratio > 0.2f && ratio < 5.0f) {
            gain[axis] += (ratio - gain[axis]) * updateRate * R * R;

            if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_AGC, 2, ratio * 1000);
            }
        }
    }
}

void agcInit(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);

    dindex = 0;
    dtotal = 200;

    updateRate = 0.1f / constrainf(pidProfile->agc_rate, 1, 50000);

    float cutoff = pidProfile->agc_freq / 10.0f;
    float Q = pidProfile->agc_q / 10.0f;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {

        gain[axis] = 1.0f;

        //biquadFilterInit(&ctrlFilter[axis], cutoff, gyro.targetLooptime, Q, FILTER_NOTCH);
        //biquadFilterInit(&gyroFilter[axis], cutoff, gyro.targetLooptime, Q, FILTER_NOTCH);

        biquadFilterInitLPF(&ctrlFilter[axis], cutoff, gyro.targetLooptime);
        biquadFilterInitLPF(&gyroFilter[axis], cutoff, gyro.targetLooptime);

        for (int i = 0; i<512; i++) {
            dline[axis][i] = 0;
        }
    }

    UNUSED(Q);
}

