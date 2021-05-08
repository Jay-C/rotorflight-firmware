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

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/pwm_output.h"

#include "sensors/gyro.h"

#include "fc/runtime_config.h"

#include "flight/servos.h"
#include "flight/mixer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"


static FAST_RAM_ZERO_INIT uint8_t  servoCount;

static FAST_RAM_ZERO_INIT float    servoSpeed[MAX_SUPPORTED_SERVOS];
static FAST_RAM_ZERO_INIT float    servoOutput[MAX_SUPPORTED_SERVOS];
static FAST_RAM_ZERO_INIT int16_t  servoOverride[MAX_SUPPORTED_SERVOS];


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoPwmRate = DEFAULT_SERVO_UPDATE;

    for (unsigned i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoConfig->dev.ioTags[i] = timerioTagGetByUsage(TIM_USE_SERVO, i);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
                     .min   = DEFAULT_SERVO_MIN,
                     .max   = DEFAULT_SERVO_MAX,
                     .mid   = DEFAULT_SERVO_CENTER,
                     .trim  = DEFAULT_SERVO_TRIM,
                     .rate  = DEFAULT_SERVO_RATE,
                     .speed = DEFAULT_SERVO_SPEED,
        );
    }
}


uint8_t getServoCount(void)
{
    return servoCount;
}

int16_t getServoOutput(uint8_t servo)
{
    return lrintf(servoOutput[servo]);
}

bool hasServoOverride(uint8_t servo)
{
    return (servoOverride[servo] >= SERVO_OVERRIDE_MIN && servoOverride[servo] <= SERVO_OVERRIDE_MAX);
}

int16_t getServoOverride(uint8_t servo)
{
    return servoOverride[servo];
}

int16_t setServoOverride(uint8_t servo, int16_t val)
{
    return servoOverride[servo] = val;
}

void servoInit(void)
{
    const ioTag_t *ioTags = servoConfig()->dev.ioTags;

    for (servoCount = 0;
         servoCount < MAX_SUPPORTED_SERVOS && ioTags[servoCount] != IO_TAG_NONE;
         servoCount++);

    servoDevInit(&servoConfig()->dev, servoCount);

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoOutput[i] = servoParams(i)->mid;
        servoOverride[i] = SERVO_OVERRIDE_OFF;

        if (servoParams(i)->speed > 0) {
            float range = servoParams(i)->max - servoParams(i)->min;
            float speed = servoParams(i)->speed * 1000;
            servoSpeed[i] = range * gyro.targetLooptime / speed;
        }
    }
}

static inline float limitSpeed(float old, float new, float rate)
{
    float diff = new - old;

    if (diff > rate)
        return old + rate;
    else if (diff < -rate)
        return old - rate;

    return new;
}

void servoUpdate(void)
{
    float pos, trim;

    for (int i = 0; i < servoCount; i++)
    {
        trim = (servoParams(i)->rate >= 0) ? servoParams(i)->trim : -servoParams(i)->trim;

        if (!ARMING_FLAG(ARMED) && hasServoOverride(i))
            pos = servoOverride[i] / 1000.0f;
        else
            pos = 0; // TODO mixerGetServoOutput(i);

        pos = servoParams(i)->mid + pos * servoParams(i)->rate;
        pos = constrainf(pos, servoParams(i)->min, servoParams(i)->max) + trim;

        if (servoSpeed[i] > 0)
            pos = limitSpeed(servoOutput[i], pos, servoSpeed[i]);

        servoOutput[i] = pos;

        pwmWriteServo(i, servoOutput[i]);
    }
}

#endif
