/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"

#include "fc/controlrate_profile.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "mixer_init.h"

PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .unused = 0,
);

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

FAST_DATA_ZERO_INIT mixerRuntime_t mixerRuntime;

uint8_t getMotorCount(void)
{
    return mixerRuntime.motorCount;
}

bool areMotorsRunning(void)
{
    bool motorsRunning = false;
    if (ARMING_FLAG(ARMED)) {
        motorsRunning = true;
    } else {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            if (motor_disarmed[i] != mixerRuntime.disarmMotorOutput) {
                motorsRunning = true;

                break;
            }
        }
    }

    return motorsRunning;
}

// All PWM motor scaling is done to standard PWM range of 1000-2000 for easier tick conversion with legacy code / configurator
// DSHOT scaling is done to the actual dshot range
void initEscEndpoints(void)
{
    float motorOutputLimit = 1.0f;
    motorInitEndpoints(motorConfig(), motorOutputLimit, &mixerRuntime.motorOutputLow, &mixerRuntime.motorOutputHigh, &mixerRuntime.disarmMotorOutput, &mixerRuntime.deadbandMotor3dHigh, &mixerRuntime.deadbandMotor3dLow);
}

// Initialize pidProfile related mixer settings

void mixerInitProfile(void)
{
#ifdef USE_DYN_IDLE
    if (motorConfigMutable()->dev.useDshotTelemetry) {
        mixerRuntime.dynIdleMinRps = currentPidProfile->dyn_idle_min_rpm * 100.0f / 60.0f;
    } else {
        mixerRuntime.dynIdleMinRps = 0.0f;
    }
    mixerRuntime.dynIdlePGain = currentPidProfile->dyn_idle_p_gain * 0.00015f;
    mixerRuntime.dynIdleIGain = currentPidProfile->dyn_idle_i_gain * 0.01f * pidGetDT();
    mixerRuntime.dynIdleDGain = currentPidProfile->dyn_idle_d_gain * 0.0000003f * pidGetPidFrequency();
    mixerRuntime.dynIdleMaxIncrease = currentPidProfile->dyn_idle_max_increase * 0.001f;
    mixerRuntime.minRpsDelayK = 800 * pidGetDT() / 20.0f; //approx 20ms D delay, arbitrarily suits many motors
    if (!mixerRuntime.feature3dEnabled && mixerRuntime.dynIdleMinRps) {
        mixerRuntime.motorOutputLow = DSHOT_MIN_THROTTLE; // Override value set by initEscEndpoints to allow zero motor drive
    }
#endif
}

static void mixerConfigureOutput(void)
{
    mixerRuntime.motorCount = QUAD_MOTOR_COUNT;
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        mixerRuntime.currentMixer[i] = mixerQuadX[i];
    }
    mixerResetDisarmedMotors();
}

void mixerInit(void)
{
    initEscEndpoints();

#ifdef USE_DYN_IDLE
    mixerRuntime.dynIdleI = 0.0f;
    mixerRuntime.prevMinRps = 0.0f;
#endif

    mixerConfigureOutput();
}

void mixerResetDisarmedMotors(void)
{
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = mixerRuntime.disarmMotorOutput;
    }
}

float getMotorOutputLow(void)
{
    return mixerRuntime.motorOutputLow;
}

float getMotorOutputHigh(void)
{
    return mixerRuntime.motorOutputHigh;
}
