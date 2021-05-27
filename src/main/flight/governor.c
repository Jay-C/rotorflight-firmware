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
#include "fc/rc_controls.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rx/rx.h"

#include "flight/governor.h"
#include "flight/mixer.h"
#include "flight/pid.h"


// Throttle mapping in IDLE state
#define GOV_THROTTLE_OFF_LIMIT          0.05f
#define GOV_THROTTLE_IDLE_LIMIT         0.20f
#define GOV_THROTTLE_IDLE_RANGE         0.10f

// Minimum throttle output from PI algorithms
#define GOV_MIN_THROTTLE_OUTPUT         0.05f

// Motor RPM quality levels
#define GOV_RPM_DETECT_DELAY            100
#define GOV_RPM_DETECT_SPEED            2000

// Lost RPM levels
#define GOV_RPM_INVALID_SPEED           1000
#define GOV_RPM_INVALID_THROTTLE        0.10f

// Feedforward variance threshold
#define GOV_FF_VAR_THRESHOLD            0.001f

// Minimum runtime for saving calibration
#define GOV_CALIB_MIN_RUNTIME           30


PG_REGISTER_WITH_RESET_TEMPLATE(governorConfig_t, governorConfig, PG_GOVERNOR_CONFIG, 0);

PG_RESET_TEMPLATE(governorConfig_t, governorConfig,
    .gov_mode = GM_PASSTHROUGH,
    .gov_max_headspeed = 2000,
    .gov_gear_ratio = 1000,
    .gov_spoolup_time = 100,
    .gov_tracking_time = 20,
    .gov_recovery_time = 20,
    .gov_lost_throttle_timeout = 30,
    .gov_lost_headspeed_timeout = 10,
    .gov_autorotation_timeout = 0,
    .gov_autorotation_bailout_time = 0,
    .gov_autorotation_min_entry_time = 10,
    .gov_pwr_filter = 20,
    .gov_rpm_filter = 20,
    .gov_gain = 100,
    .gov_p_gain = 20,
    .gov_i_gain = 20,
    .gov_d_gain = 0,
    .gov_f_gain = 0,
    .gov_cyclic_ff_weight = 40,
    .gov_collective_ff_weight = 100,
    .gov_ff_exponent = 150,
    .gov_vbat_offset = 0,
    .gov_st_filter = 1000,
    .gov_cs_filter = 5000,
    .gov_cf_filter = 2500,
    .gov_cg_filter = 5000,
);


static FAST_RAM_ZERO_INIT uint8_t govMode;

static FAST_RAM_ZERO_INIT uint8_t govState;
static FAST_RAM_ZERO_INIT timeMs_t govStateEntryTime;

static FAST_RAM_ZERO_INIT float govGearRatio;

static FAST_RAM_ZERO_INIT float govOutput;
static FAST_RAM_ZERO_INIT float govThrottle;
static FAST_RAM_ZERO_INIT bool  govThrottleLow;

static FAST_RAM_ZERO_INIT float govNominalVoltage;

static FAST_RAM_ZERO_INIT float govVoltage;
static FAST_RAM_ZERO_INIT biquadFilter_t govVoltageFilter;

static FAST_RAM_ZERO_INIT float govCurrent;
static FAST_RAM_ZERO_INIT biquadFilter_t govCurrentFilter;

static FAST_RAM_ZERO_INIT float govMotorRPM;
static FAST_RAM_ZERO_INIT uint32_t govMotorRPMGood;
static FAST_RAM_ZERO_INIT biquadFilter_t govMotorRPMFilter;

static FAST_RAM_ZERO_INIT float govSetpoint;
static FAST_RAM_ZERO_INIT float govHeadSpeed;
static FAST_RAM_ZERO_INIT float govMaxHeadSpeed;
static FAST_RAM_ZERO_INIT float govTargetHeadSpeed;

static FAST_RAM_ZERO_INIT bool  govHeadSpeedError;
static FAST_RAM_ZERO_INIT bool  govHeadSpeedPresent;

static FAST_RAM_ZERO_INIT bool  govAutoEnabled;
static FAST_RAM_ZERO_INIT long  govAutoTimeout;
static FAST_RAM_ZERO_INIT long  govAutoMinEntry;

static FAST_RAM_ZERO_INIT long  govLostThrottleTimeout;
static FAST_RAM_ZERO_INIT long  govLostHeadspeedTimeout;

static FAST_RAM_ZERO_INIT float govThrottleSpoolupRate;
static FAST_RAM_ZERO_INIT float govThrottleBailoutRate;
static FAST_RAM_ZERO_INIT float govThrottleRecoveryRate;
static FAST_RAM_ZERO_INIT float govThrottleTrackingRate;

static FAST_RAM_ZERO_INIT float govSetpointSpoolupRate;
static FAST_RAM_ZERO_INIT float govSetpointBailoutRate;
static FAST_RAM_ZERO_INIT float govSetpointRecoveryRate;
static FAST_RAM_ZERO_INIT float govSetpointTrackingRate;

static FAST_RAM_ZERO_INIT float govK;
static FAST_RAM_ZERO_INIT float govKp;
static FAST_RAM_ZERO_INIT float govKi;
static FAST_RAM_ZERO_INIT float govKd;
static FAST_RAM_ZERO_INIT float govKf;

static FAST_RAM_ZERO_INIT float govP;
static FAST_RAM_ZERO_INIT float govI;
static FAST_RAM_ZERO_INIT float govC;
static FAST_RAM_ZERO_INIT float govD;
static FAST_RAM_ZERO_INIT float govF;
static FAST_RAM_ZERO_INIT float govPidSum;
static FAST_RAM_ZERO_INIT float govError;

static FAST_RAM_ZERO_INIT float govCycWeight;
static FAST_RAM_ZERO_INIT float govColWeight;

static FAST_RAM_ZERO_INIT float govCyclicFF;
static FAST_RAM_ZERO_INIT float govCollectiveFF;
static FAST_RAM_ZERO_INIT float govFeedForward;
static FAST_RAM_ZERO_INIT float govFFexponent;

static FAST_RAM_ZERO_INIT float govBatOffset;


//// Statistics

static FAST_RAM_ZERO_INIT float stFilter;
static FAST_RAM_ZERO_INIT float csFilter;
static FAST_RAM_ZERO_INIT float cfFilter;
static FAST_RAM_ZERO_INIT float cgFilter;

static FAST_RAM_ZERO_INIT float hsValue;
static FAST_RAM_ZERO_INIT float hsDiff;
static FAST_RAM_ZERO_INIT float hsMean;
static FAST_RAM_ZERO_INIT float hsVar;

static FAST_RAM_ZERO_INIT float vtValue;
static FAST_RAM_ZERO_INIT float vtDiff;
static FAST_RAM_ZERO_INIT float vtMean;
static FAST_RAM_ZERO_INIT float vtVar;

static FAST_RAM_ZERO_INIT float cxValue;
static FAST_RAM_ZERO_INIT float cxDiff;
static FAST_RAM_ZERO_INIT float cxMean;
static FAST_RAM_ZERO_INIT float cxVar;

static FAST_RAM_ZERO_INIT float ffValue;
static FAST_RAM_ZERO_INIT float ffDiff;
static FAST_RAM_ZERO_INIT float ffMean;
static FAST_RAM_ZERO_INIT float ffVar;

static FAST_RAM_ZERO_INIT float hsvtCovar;
static FAST_RAM_ZERO_INIT float cxffCovar;

static FAST_RAM_ZERO_INIT float ccValue;
static FAST_RAM_ZERO_INIT float csValue;
static FAST_RAM_ZERO_INIT float cfValue;

static FAST_RAM_ZERO_INIT float csEstimate;
static FAST_RAM_ZERO_INIT float cfEstimate;

static FAST_RAM_ZERO_INIT uint32_t stCount;


//// Prototypes

static void govPIDInit(void);
static void govMode1Init(void);
static void govMode2Init(void);
static void govMode3Init(void);

static float govPIDControl(void);
static float govMode1Control(void);
static float govMode2Control(void);
static float govMode3Control(void);

static void governorUpdateState(void);
static void governorUpdatePassthrough(void);


//// Handler functions

typedef void  (*govVoidFn)(void);
typedef float (*govFloatFn)(void);

static FAST_RAM_ZERO_INIT govVoidFn   govStateUpdate;

static FAST_RAM_ZERO_INIT govVoidFn   govSpoolupInit;
static FAST_RAM_ZERO_INIT govFloatFn  govSpoolupCalc;

static FAST_RAM_ZERO_INIT govVoidFn   govActiveInit;
static FAST_RAM_ZERO_INIT govFloatFn  govActiveCalc;



//// Access functions

float getHeadSpeed(void)
{
    return govHeadSpeed;
}

float getHeadSpeedRatio(void)
{
    return govHeadSpeed / govMaxHeadSpeed;
}

uint8_t getGovernorState(void)
{
    return govState;
}

float getGovernorOutput(void)
{
    return govOutput;
}

bool isSpooledUp(void)
{
    switch (govState)
    {
        case GS_ACTIVE:
        case GS_LOST_THROTTLE:
        case GS_LOST_HEADSPEED:
        case GS_AUTOROTATION:
        case GS_AUTOROTATION_BAILOUT:
            return true;

        case GS_THROTTLE_OFF:
        case GS_THROTTLE_IDLE:
        case GS_SPOOLING_UP:
        case GS_RECOVERY:
            return false;
    }

    return false;
}


//// Internal functions

static inline float rampUpLimit(float target, float current, float rate)
{
    if (target > current)
        return MIN(current + rate, target);

    return target;
}

static inline float rampLimit(float target, float current, float rate)
{
    if (target > current)
        return MIN(current + rate, target);
    else
        return MAX(current - rate, target);
}

static inline float idleMap(float throttle)
{
    // Map throttle in IDLE state
    //     0%..5%   => 0%
    //     5%..20%  => 0%..10%
    //     >20%     => 10%
    throttle = (throttle - GOV_THROTTLE_OFF_LIMIT) * GOV_THROTTLE_IDLE_RANGE /
        (GOV_THROTTLE_IDLE_LIMIT - GOV_THROTTLE_OFF_LIMIT);

    return constrainf(throttle, 0, GOV_THROTTLE_IDLE_RANGE);
}

static inline void govChangeState(uint8_t futureState)
{
    govState = futureState;
    govStateEntryTime = millis();
}

static inline long govStateTime(void)
{
    return cmp32(millis(),govStateEntryTime);
}

static void govSaveCalibration(void)
{
    governorConfigMutable()->gov_calibration[0] = 1e6f * csEstimate;
    governorConfigMutable()->gov_calibration[1] = 1e6f * cfEstimate;
    governorConfigMutable()->gov_calibration[2] = 1e3f * cfEstimate / csEstimate;

    saveConfigAndNotify();
}

static void govResetStats(void)
{
    vtValue   = cxValue   = 0;
    ccValue   = csValue   = cfValue  = 0;
    vtMean    = cxMean    = hsMean   = ffMean = 0;
    vtDiff    = cxDiff    = hsDiff   = ffDiff = 0;
    vtVar     = cxVar     = hsVar    = ffVar  = 0;
    cxffCovar = hsvtCovar = 0;
    stCount = 0;
}

static void govDebugStats(void)
{
#ifdef USE_DEBUG32

#define DEBUG32U(value)  (debug32[index++] = (uint32_t)(value))
#define DEBUG32S(value)  (debug32[index++] = (int32_t)(value))
#define DEBUG32F(value)  (debug32[index++] = lrintf(value))

    if (debugMode == DEBUG_GOVERNOR)
    {
        int index = 0;

        DEBUG32F(govSetpoint * 1e3f);
        DEBUG32F(govHeadSpeed * 1e3f);
        DEBUG32F(govOutput * 1e6f);
        DEBUG32F(govPidSum * 1e6f);
        DEBUG32F(govP * 1e6f);
        DEBUG32F(govI * 1e6f);
        DEBUG32F(govD * 1e6f);
        DEBUG32F(govF * 1e6f);

#ifdef USE_GOV_DEBUG32
        DEBUG32F(govError * 1e6f);
        DEBUG32F(govThrottle * 1e6f);
        DEBUG32F(govTargetHeadSpeed * 1e3f);

        DEBUG32F(govCyclicFF * 1e6f);
        DEBUG32F(govCollectiveFF * 1e6f);
        DEBUG32F(govFeedForward * 1e6f);

        DEBUG32F(govMotorRPM);
        DEBUG32F(govVoltage * 1e3f);
        DEBUG32F(govCurrent * 1e3f);

        DEBUG32F(hsValue * 1e3f);
        DEBUG32F(hsDiff * 1e3f);
        DEBUG32F(hsMean * 1e3f);
        DEBUG32F(hsVar * 1e0f);

        DEBUG32F(vtValue * 1e3f);
        DEBUG32F(vtDiff * 1e3f);
        DEBUG32F(vtMean * 1e3f);
        DEBUG32F(vtVar * 1e0f);

        DEBUG32F(cxValue * 1e6f);
        DEBUG32F(cxDiff * 1e6f);
        DEBUG32F(cxMean * 1e6f);
        DEBUG32F(cxVar * 1e12);

        DEBUG32F(ffValue * 1e6f);
        DEBUG32F(ffDiff * 1e6f);
        DEBUG32F(ffMean * 1e6f);
        DEBUG32F(ffVar * 1e6f);

        DEBUG32F(hsvtCovar * 1e9f);
        DEBUG32F(cxffCovar * 1e9f);

        DEBUG32F(ccValue * 1e3f);
        DEBUG32F(csValue * 1e6f);
        DEBUG32F(cfValue * 1e6f);

        DEBUG32F(csEstimate * 1e6f);
        DEBUG32F(cfEstimate * 1e6f);
#endif
    }

#endif /* USE_DEBUG32 */

    DEBUG_SET(DEBUG_GOVERNOR,  0, govTargetHeadSpeed);
    DEBUG_SET(DEBUG_GOVERNOR,  1, govSetpoint);
    DEBUG_SET(DEBUG_GOVERNOR,  2, govPidSum * 1000);
    DEBUG_SET(DEBUG_GOVERNOR,  3, govFeedForward * 1000);
}

static void govUpdateStats(void)
{
    if (govOutput > 0.10f && govHeadSpeed > 100)
    {
        hsValue = govHeadSpeed;
        vtValue = (govVoltage - govBatOffset) * govOutput;
        cxValue = vtValue / hsValue;
        ffValue = govFeedForward;

        hsDiff = hsValue - hsMean;
        vtDiff = vtValue - vtMean;
        cxDiff = cxValue - cxMean;
        ffDiff = ffValue - ffMean;

        hsMean += hsDiff * stFilter;
        vtMean += vtDiff * stFilter;
        cxMean += cxDiff * stFilter;
        ffMean += ffDiff * stFilter;

        float tsFilter = 1.0f - stFilter;

        hsVar = (hsDiff * hsDiff * stFilter + hsVar) * tsFilter;
        vtVar = (vtDiff * vtDiff * stFilter + vtVar) * tsFilter;
        cxVar = (cxDiff * cxDiff * stFilter + cxVar) * tsFilter;
        ffVar = (ffDiff * ffDiff * stFilter + ffVar) * tsFilter;

        hsvtCovar = (hsDiff * vtDiff * stFilter + hsvtCovar) * tsFilter;
        cxffCovar = (cxDiff * ffDiff * stFilter + cxffCovar) * tsFilter;
    }
}

static void govUpdateSpoolupStats(float govMain)
{
    if (govMain > 0.10f && govHeadSpeed > 100 &&
        stCount < pidGetPidFrequency() * GOV_CALIB_MIN_RUNTIME)
    {
        // For analysis -- not used in the model
        if (hsVar > 100) {
            csValue = hsvtCovar / hsVar;
            ccValue = vtMean - hsMean * csValue;
        }

        // Initial value for Cs
        csEstimate = cxMean;

        // Initial value for Cf
        if (governorConfig()->gov_calibration[2] > 0)
            cfEstimate = csEstimate * governorConfig()->gov_calibration[2] / 1000.0f;
        else
            cfEstimate = csEstimate;
    }
}

static void govUpdateActiveStats(float govMain)
{
    bool mainCheck = true, hsCheck = true;

    // throttle must be between 10%..90%
    mainCheck = (govMain > 0.10f && govMain < 0.90f);

    // Headspeed must be reasonable
    if (govMode == GM_PASSTHROUGH)
        hsCheck = (hsValue > 100);
    else
        hsCheck = (hsValue > govMaxHeadSpeed * 0.1f && hsValue < govMaxHeadSpeed * 1.1f);

    // System must stay in linear & controllable area for the stats to be correct
    if (mainCheck && hsCheck)
    {
        // Increment statistics count
        stCount++;

        if (ffVar > GOV_FF_VAR_THRESHOLD)
            cfValue = cxffCovar / ffVar;
        else
            cfValue = cfEstimate;

        cfValue = constrainf(cfValue, 0, 2*csEstimate);

        if (cfValue > cfEstimate)
            cfEstimate += (cfValue - cfEstimate) * ffVar * ffMean * cfFilter;
        else
            cfEstimate += (cfValue - cfEstimate) * ffVar * ffMean * cgFilter;

        csValue = cxMean - ffMean * cfValue;

        csEstimate += (csValue - csEstimate) * csFilter;
    }
}

static void govUpdateInputs(void)
{
    // Update throttle state
    govThrottle = (float)(rcCommand[THROTTLE] - PWM_RANGE_MIN) / (PWM_RANGE_MAX - PWM_RANGE_MIN);
    govThrottleLow = (calculateThrottleStatus() == THROTTLE_LOW);

    // Update headspeed target
    govTargetHeadSpeed = govThrottle * govMaxHeadSpeed;

    // Assume motor[0]
    govMotorRPM = getMotorRawRPMf(0);

    // RPM signal is noisy - filtering is required
    float filteredRPM = biquadFilterApply(&govMotorRPMFilter, govMotorRPM);

    // Evaluate RPM signal quality
    if (govMotorRPM > 0 && filteredRPM > GOV_RPM_DETECT_SPEED) {
        if (!govMotorRPMGood) {
            govMotorRPMGood = millis();
        }
    } else {
        if (govMotorRPMGood) {
            govMotorRPMGood = 0;
        }
    }

    // Headspeed is present if RPM is stable
    govHeadSpeedPresent = (govMotorRPMGood && cmp32(millis(),govMotorRPMGood) > GOV_RPM_DETECT_DELAY);

    // Headspeed should be available if throttle is high enough
    govHeadSpeedError = (govMotorRPM < GOV_RPM_INVALID_SPEED && govOutput > GOV_RPM_INVALID_THROTTLE);

    // Calculate headspeed from filtered motor speed
    govHeadSpeed = filteredRPM / govGearRatio;

    // Battery state - zero when battery unplugged
    govNominalVoltage = getBatteryCellCount() * 3.70f;

    // Voltage & current filters
    govVoltage = biquadFilterApply(&govVoltageFilter, getBatteryVoltageLatest() * 0.01f);
    govCurrent = biquadFilterApply(&govCurrentFilter, getAmperageLatest() * 0.01f);

    // Calculate feedforward from collective deflection
    govCollectiveFF = govColWeight * getCollectiveDeflection();

    // Calculate feedforward from cyclic deflection
    govCyclicFF = govCycWeight * getCyclicDeflection();

    // Angle-of-attack vs. FeedForward curve
    govFeedForward = powf(govCollectiveFF + govCyclicFF, govFFexponent);

    // Normalized RPM error
    float newError = (govSetpoint - govHeadSpeed) / govMaxHeadSpeed;

    // Update PIDF terms
    govP = govK * govKp * newError;
    govC = govK * govKi * newError * pidGetDT();
    govD = govK * govKd * (newError - govError) / pidGetDT();
    govF = govK * govKf * govFeedForward;

    // Update error term
    govError = newError;
}


/*
 * Throttle passthrough with rampup limits and extra stats.
 */

static void governorUpdatePassthrough(void)
{
    float govPrev = govOutput;
    float govMain = 0;

    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED) || getBatteryCellCount() == 0) {
        govChangeState(GS_THROTTLE_OFF);
        govResetStats();
    }
    else {
        switch (govState)
        {
            // Throttle is OFF
            case GS_THROTTLE_OFF:
                govMain = 0;
                if (!govThrottleLow)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (stCount > pidGetPidFrequency() * GOV_CALIB_MIN_RUNTIME) {
                    govSaveCalibration();
                    govResetStats();
                }
                break;

            // Throttle is IDLE, follow with a limited ramupup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > 20%, move to SPOOLUP
            case GS_THROTTLE_IDLE:
                govMain = rampUpLimit(govThrottle, govPrev, govThrottleTrackingRate);
                if (govThrottleLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (govThrottle > GOV_THROTTLE_IDLE_LIMIT)
                    govChangeState(GS_SPOOLING_UP);
                break;

            // Follow the throttle, with a limited rampup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If 0% < throttle < 20%, stay in spoolup
            //  -- Once throttle >20% and not ramping up, move to ACTIVE
            case GS_SPOOLING_UP:
                govMain = rampUpLimit(govThrottle, govPrev, govThrottleSpoolupRate);
                if (govThrottleLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (govThrottle < GOV_THROTTLE_IDLE_LIMIT)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (govMain >= govThrottle)
                    govChangeState(GS_ACTIVE);
                else
                    govUpdateSpoolupStats(govMain);
                break;

            // Follow the throttle without ramp limits.
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If throttle <20%, move to AUTO or SPOOLING_UP
            case GS_ACTIVE:
                govMain = govThrottle;
                if (govThrottleLow)
                    govChangeState(GS_LOST_THROTTLE);
                else if (govMain < GOV_THROTTLE_IDLE_LIMIT) {
                    if (govAutoEnabled && govStateTime() > govAutoMinEntry)
                        govChangeState(GS_AUTOROTATION);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                }
                else
                    govUpdateActiveStats(govMain);
                break;

            // Throttle is low. If it is a mistake, give a chance to recover.
            //  -- If throttle returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GS_LOST_THROTTLE:
                govMain = 0;
                if (!govThrottleLow)
                    govChangeState(GS_RECOVERY);
                else if (govStateTime() > govLostThrottleTimeout)
                    govChangeState(GS_THROTTLE_OFF);
                break;

            // Follow throttle with high(er) ramp rate.
            //  -- If throttle is >20%, move to AUTOROTATION_BAILOUT
            //  -- If timer expires, move to IDLE
            case GS_AUTOROTATION:
                govMain = rampUpLimit(govThrottle, govPrev, govThrottleTrackingRate);
                if (govThrottleLow)
                    govChangeState(GS_LOST_THROTTLE);
                else if (govThrottle > GOV_THROTTLE_IDLE_LIMIT)
                    govChangeState(GS_AUTOROTATION_BAILOUT);
                else if (govStateTime() > govAutoTimeout)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle <20%, move back to AUTO
            case GS_AUTOROTATION_BAILOUT:
                govMain = rampUpLimit(govThrottle, govPrev, govThrottleBailoutRate);
                if (govThrottleLow)
                    govChangeState(GS_LOST_THROTTLE);
                else if (govThrottle < GOV_THROTTLE_IDLE_LIMIT)
                    govChangeState(GS_AUTOROTATION);
                else if (govMain >= govThrottle)
                    govChangeState(GS_ACTIVE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle <20%, move to IDLE
            case GS_RECOVERY:
                govMain = rampUpLimit(govThrottle, govPrev, govThrottleRecoveryRate);
                if (govThrottleLow)
                    govChangeState(GS_LOST_THROTTLE);
                else if (govThrottle < GOV_THROTTLE_IDLE_LIMIT)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (govMain >= govThrottle)
                    govChangeState(GS_ACTIVE);
                break;

            // Should not be here
            default:
                govChangeState(GS_THROTTLE_OFF);
                govResetStats();
                break;
        }
    }

    // Update output variable
    govOutput = govMain;

    // Set debug
    govDebugStats();
}


/*
 * State machine for governed speed control
 */

static inline void govEnterSpoolupState(uint8_t state)
{
    govChangeState(state);
    govSpoolupInit();
}

static inline void govEnterActiveState(uint8_t state)
{
    govChangeState(state);
    govActiveInit();
}

static void governorUpdateState(void)
{
    float govPrev = govOutput;
    float govMain = 0;

    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED) || getBatteryCellCount() == 0) {
        govChangeState(GS_THROTTLE_OFF);
        govResetStats();
    }
    else {
        switch (govState)
        {
            // Throttle is OFF
            case GS_THROTTLE_OFF:
                govMain = 0;
                govSetpoint = 0;
                if (!govThrottleLow)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (stCount > pidGetPidFrequency() * GOV_CALIB_MIN_RUNTIME) {
                    govSaveCalibration();
                    govResetStats();
                }
                break;

            // Throttle is IDLE, follow with a limited ramupup rate
            //  -- Map throttle to motor output
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > 20%, move to SPOOLUP
            case GS_THROTTLE_IDLE:
                govMain = rampUpLimit(idleMap(govThrottle), govPrev, govThrottleSpoolupRate);
                govSetpoint = govHeadSpeed;
                if (govThrottleLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (govThrottle > GOV_THROTTLE_IDLE_LIMIT && govHeadSpeedPresent)
                    govEnterSpoolupState(GS_SPOOLING_UP);
                break;

            // Ramp up throttle until headspeed target is reached
            //  -- Once 99% headspeed reached, move to ACTIVE
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE
            //  -- If throttle <20%, move back to IDLE
            //  -- If no headspeed detected, move to LOST_HEADSPEED
            //  -- If NO throttle, move to THROTTLE_OFF
            case GS_SPOOLING_UP:
                govMain = govPrev;
                if (govThrottleLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (govThrottle < GOV_THROTTLE_IDLE_LIMIT)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (govHeadSpeedError)
                    govChangeState(GS_LOST_HEADSPEED);
                else if (govHeadSpeed > govTargetHeadSpeed * 0.99f || govMain > 0.95f)
                    govEnterActiveState(GS_ACTIVE);
                else {
                    govMain = govSpoolupCalc();
                    govSetpoint = rampLimit(govTargetHeadSpeed, govSetpoint, govSetpointSpoolupRate);
                    govUpdateSpoolupStats(govMain);
                }
                break;

            // Governor active, maintain headspeed
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If no headspeed signal, move to LOST_HEADSPEED
            //  -- If throttle <20%, move to AUTOROTATION or IDLE
            case GS_ACTIVE:
                govMain = govPrev;
                if (govThrottleLow)
                    govChangeState(GS_LOST_THROTTLE);
                else if (govHeadSpeedError)
                    govChangeState(GS_LOST_HEADSPEED);
                else if (govThrottle < GOV_THROTTLE_IDLE_LIMIT) {
                    if (govAutoEnabled && govStateTime() > govAutoMinEntry)
                        govChangeState(GS_AUTOROTATION);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                } else {
                    govMain = govActiveCalc();
                    govSetpoint = rampLimit(govTargetHeadSpeed, govSetpoint, govSetpointTrackingRate);
                    govUpdateActiveStats(govMain);
                }
                break;

            // Throttle is off or low. If it is a mistake, give a chance to recover
            //  -- When throttle and *headspeed* returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GS_LOST_THROTTLE:
                govMain = rampUpLimit(idleMap(govThrottle), govPrev, govThrottleTrackingRate);
                govSetpoint = govHeadSpeed;
                if (govThrottle > GOV_THROTTLE_IDLE_LIMIT && govHeadSpeedPresent)
                    govEnterSpoolupState(GS_RECOVERY);
                else if (govStateTime() > govLostThrottleTimeout)
                    govChangeState(GS_THROTTLE_OFF);
                break;

            // No headspeed signal. Ramp down throttle slowly.
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If headspeed recovers, move to RECOVERY or IDLE
            //  -- When timer expires, move to OFF
            case GS_LOST_HEADSPEED:
                govMain = rampLimit(idleMap(govThrottle), govPrev, govThrottleSpoolupRate);
                govSetpoint = govHeadSpeed;
                if (govThrottleLow)
                    govChangeState(GS_LOST_THROTTLE);
                else if (govHeadSpeedPresent) {
                    if (govThrottle > GOV_THROTTLE_IDLE_LIMIT)
                        govEnterSpoolupState(GS_RECOVERY);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                }
                else if (govStateTime() > govLostHeadspeedTimeout)
                    govChangeState(GS_THROTTLE_OFF);
                break;

            // Throttle passthrough with rampup limit
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If throttle >20%, move to AUTOROTATION_BAILOUT
            //  -- If timer expires, move to IDLE
            //  -- Map throttle to motor output
            case GS_AUTOROTATION:
                govMain = rampUpLimit(idleMap(govThrottle), govPrev, govThrottleTrackingRate);
                govSetpoint = govHeadSpeed;
                if (govThrottleLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (govThrottle > GOV_THROTTLE_IDLE_LIMIT && govHeadSpeedPresent)
                    govEnterSpoolupState(GS_AUTOROTATION_BAILOUT);
                else if (govStateTime() > govAutoTimeout)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) rampup.
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If no headspeed detected, move to LOST_HEADSPEED
            //  -- If throttle <20%, move back to AUTOROTATION
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GS_AUTOROTATION_BAILOUT:
                govMain = govPrev;
                if (govThrottleLow)
                    govChangeState(GS_LOST_THROTTLE);
                else if (govHeadSpeedError)
                    govChangeState(GS_LOST_HEADSPEED);
                else if (govThrottle < GOV_THROTTLE_IDLE_LIMIT)
                    govChangeState(GS_AUTOROTATION);
                else if (govHeadSpeed > govTargetHeadSpeed * 0.99f || govMain > 0.95f)
                    govEnterActiveState(GS_ACTIVE);
                else {
                    govMain = govSpoolupCalc();
                    govSetpoint = rampLimit(govTargetHeadSpeed, govSetpoint, govSetpointBailoutRate);
                }
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) rampup.
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If no headspeed detected, move to LOST_HEADSPEED
            //  -- If throttle <20%, move to IDLE
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GS_RECOVERY:
                govMain = govPrev;
                if (govThrottleLow)
                    govChangeState(GS_LOST_THROTTLE);
                else if (govHeadSpeedError)
                    govChangeState(GS_LOST_HEADSPEED);
                else if (govThrottle < GOV_THROTTLE_IDLE_LIMIT)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (govHeadSpeed > govTargetHeadSpeed * 0.99f || govMain > 0.95f)
                    govEnterActiveState(GS_ACTIVE);
                else {
                    govMain = govSpoolupCalc();
                    govSetpoint = rampLimit(govTargetHeadSpeed, govSetpoint, govSetpointRecoveryRate);
                }
                break;

            // Should not be here
            default:
                govChangeState(GS_THROTTLE_OFF);
                govResetStats();
                break;
        }
    }

    // Update output variables
    govOutput = govMain;

    // Set debug
    govDebugStats();
}


/*
 * Standard PID controller
 */

static void govPIDInit(void)
{
    // Use govI to reach the target
    govI = govOutput - govP;

    // Realistic bounds
    govI = constrainf(govI, 0, 0.95f);
}

static float govPIDControl(void)
{
    float output;

    // PID limits -- TODO
    govP = constrainf(govP, -0.25f, 0.25f);
    govI = constrainf(govI,      0, 0.95f);
    govD = constrainf(govD, -0.25f, 0.25f);

    // Governor PID sum
    govPidSum = govP + govI + govD + govC;

    // Generate throttle signal
    output = govPidSum;

    // Apply govC if output not saturated
    if (!((output > 1 && govC > 0) || (output < GOV_MIN_THROTTLE_OUTPUT && govC < 0)))
        govI += govC;

    // Limit output to 10%..100%
    output = constrainf(output, GOV_MIN_THROTTLE_OUTPUT, 1);

    return output;
}


/*
 * Mode1: PIDF controller
 */

static void govMode1Init(void)
{
    // Use govI to reach the target
    govI = govOutput - (govP + govD + govF);

    // Realistic bounds
    govI = constrainf(govI, 0, 0.95f);
}

static float govMode1Control(void)
{
    float output;

    // PID limits -- TODO
    govP = constrainf(govP, -0.25f, 0.25f);
    govI = constrainf(govI,      0, 0.95f);
    govD = constrainf(govD, -0.25f, 0.25f);
    govF = constrainf(govF,      0, 0.25f);

    // Governor PIDF sum
    govPidSum = govP + govI + govC + govD + govF;

    // Generate throttle signal
    output = govPidSum;

    // Apply govC if output not saturated
    if (!((output > 1 && govC > 0) || (output < GOV_MIN_THROTTLE_OUTPUT && govC < 0)))
        govI += govC;

    // Limit output to 10%..100%
    output = constrainf(output, GOV_MIN_THROTTLE_OUTPUT, 1);

    return output;
}


/*
 * Mode2: PIDF with Battery voltage compensation
 */

static void govMode2Init(void)
{
    // Normalized battery voltage
    float pidGain = govNominalVoltage / (govVoltage - govBatOffset);

    // Expected PID output
    float pidTarget = govOutput / pidGain;

    // Use govI to reach the target
    govI = pidTarget - (govP + govD + govF);

    // Realistic bounds
    govI = constrainf(govI, 0, 0.95f);
}

static float govMode2Control(void)
{
    float output;

    // Normalized battery voltage
    float pidGain = govNominalVoltage / (govVoltage - govBatOffset);

    // PID limits -- TODO
    govP = constrainf(govP, -0.25f, 0.25f);
    govI = constrainf(govI,      0, 0.95f);
    govD = constrainf(govD, -0.25f, 0.25f);
    govF = constrainf(govF,      0, 0.25f);

    // Governor PIDF sum
    govPidSum = govP + govI + govC + govD + govF;

    // Generate throttle signal
    output = govPidSum * pidGain;

    // Apply govC if output not saturated
    if (!((output > 1 && govC > 0) || (output < GOV_MIN_THROTTLE_OUTPUT && govC < 0)))
        govI += govC;

    // Limit output to 10%..100%
    output = constrainf(output, GOV_MIN_THROTTLE_OUTPUT, 1);

    return output;
}


/*
 * Mode3: PIDF with physic modeling and parameter estimation
 */

static void govMode3Init(void)
{
    // Common PID gain
    float pidGain = govNominalVoltage;

    // Calculate Vk estimate from the model
    float motorVk = govSetpoint * (cfEstimate * govFeedForward + csEstimate);

    // Expected PID output
    float pidTarget = (govOutput * (govVoltage - govBatOffset) - motorVk) / pidGain;

    // Use govI to reach the target
    govI = pidTarget - (govP + govD + govF);

    // Realistic bounds
    govI = constrainf(govI, -0.25f, 0.25f);
}

static float govMode3Control(void)
{
    float output;

    // Common PID gain
    float pidGain = govNominalVoltage;

    // Calculate Vk estimate from the model
    float motorVk = govSetpoint * (cfEstimate * govFeedForward + csEstimate);

    // PID limits
    govP = constrainf(govP, -0.25f, 0.25f);
    govI = constrainf(govI, -0.25f, 0.25f);
    govD = constrainf(govD, -0.25f, 0.25f);
    govF = constrainf(govF,      0, 0.25f);

    // Governor PID sum
    govPidSum = govP + govI + govC + govD + govF;

    // Generate throttle signal
    output = (motorVk + pidGain * govPidSum) / (govVoltage - govBatOffset);

    // Apply govC if output not saturated
    if (!((output > 1 && govC > 0) || (output < GOV_MIN_THROTTLE_OUTPUT && govC < 0)))
        govI += govC;

    // Limit output to 10%..100%
    output = constrainf(output, GOV_MIN_THROTTLE_OUTPUT, 1);

    return output;
}


//// Interface functions

void governorUpdate(void)
{
    // Governor is active
    if (govMode)
    {
        // Calculate all governor inputs
        govUpdateInputs();

        // Update statistics
        govUpdateStats();

        // Run state machine
        govStateUpdate();
    }
}


void governorInit(void)
{
    // Must have at least one motor
    if (getMotorCount() > 0)
    {
        govMode         = governorConfig()->gov_mode;
        govState        = GS_THROTTLE_OFF;

        // Mode specific handler functions
        switch (govMode) {
            case GM_PASSTHROUGH:
                govStateUpdate = governorUpdatePassthrough;
                govSpoolupInit = NULL;
                govSpoolupCalc = NULL;
                govActiveInit  = NULL;
                govActiveCalc  = NULL;
                break;
            case GM_STANDARD:
                govStateUpdate = governorUpdateState;
                govSpoolupInit = govPIDInit;
                govSpoolupCalc = govPIDControl;
                govActiveInit  = govPIDInit;
                govActiveCalc  = govPIDControl;
                break;
            case GM_MODE1:
                govStateUpdate = governorUpdateState;
                govSpoolupInit = govPIDInit;
                govSpoolupCalc = govPIDControl;
                govActiveInit  = govMode1Init;
                govActiveCalc  = govMode1Control;
                break;
            case GM_MODE2:
                govStateUpdate = governorUpdateState;
                govSpoolupInit = govPIDInit;
                govSpoolupCalc = govPIDControl;
                govActiveInit  = govMode2Init;
                govActiveCalc  = govMode2Control;
                break;
            case GM_MODE3:
                govStateUpdate = governorUpdateState;
                govSpoolupInit = govPIDInit;
                govSpoolupCalc = govPIDControl;
                govActiveInit  = govMode3Init;
                govActiveCalc  = govMode3Control;
                break;
        }

        govGearRatio    = (float)governorConfig()->gov_gear_ratio / 1000.0f;
        govK            = (float)governorConfig()->gov_gain / 100.0f;
        govKp           = (float)governorConfig()->gov_p_gain / 10.0f;
        govKi           = (float)governorConfig()->gov_i_gain / 10.0f;
        govKd           = (float)governorConfig()->gov_d_gain / 1000.0f;
        govKf           = (float)governorConfig()->gov_f_gain / 100.0f;
        govCycWeight    = (float)governorConfig()->gov_cyclic_ff_weight / 100.0f;
        govColWeight    = (float)governorConfig()->gov_collective_ff_weight / 100.0f;
        govFFexponent   = (float)governorConfig()->gov_ff_exponent / 100.0f;
        govBatOffset    = (float)governorConfig()->gov_vbat_offset / 100.0f;

        govMaxHeadSpeed = constrainf(governorConfig()->gov_max_headspeed, 100, 10000);

        govAutoEnabled  = (governorConfig()->gov_autorotation_timeout > 0 &&
                           governorConfig()->gov_autorotation_bailout_time > 0);
        govAutoTimeout  = governorConfig()->gov_autorotation_timeout * 100;
        govAutoMinEntry = governorConfig()->gov_autorotation_min_entry_time * 1000;

        govThrottleSpoolupRate  = pidGetDT() / constrainf(governorConfig()->gov_spoolup_time, 1, 600) * 10;
        govThrottleTrackingRate = pidGetDT() / constrainf(governorConfig()->gov_tracking_time, 1, 50) * 10;
        govThrottleRecoveryRate = pidGetDT() / constrainf(governorConfig()->gov_recovery_time, 1, 50) * 10;
        govThrottleBailoutRate  = pidGetDT() / constrainf(governorConfig()->gov_autorotation_bailout_time, 1, 100) * 10;

        govSetpointSpoolupRate  = govThrottleSpoolupRate  * govMaxHeadSpeed;
        govSetpointTrackingRate = govThrottleTrackingRate * govMaxHeadSpeed;
        govSetpointRecoveryRate = govThrottleRecoveryRate * govMaxHeadSpeed;
        govSetpointBailoutRate  = govThrottleBailoutRate  * govMaxHeadSpeed;

        govLostThrottleTimeout  = governorConfig()->gov_lost_throttle_timeout * 100;
        govLostHeadspeedTimeout = governorConfig()->gov_lost_headspeed_timeout * 100;

        biquadFilterInitLPF(&govVoltageFilter, governorConfig()->gov_pwr_filter, pidGetLooptime());
        biquadFilterInitLPF(&govCurrentFilter, governorConfig()->gov_pwr_filter, pidGetLooptime());
        biquadFilterInitLPF(&govMotorRPMFilter, governorConfig()->gov_rpm_filter, pidGetLooptime());

        stFilter = 1000.0f  / (pidGetPidFrequency() * governorConfig()->gov_st_filter);
        csFilter = 1000.0f  / (pidGetPidFrequency() * governorConfig()->gov_cs_filter);
        cfFilter = 10000.0f / (pidGetPidFrequency() * governorConfig()->gov_cf_filter);
        cgFilter = 10000.0f / (pidGetPidFrequency() * governorConfig()->gov_cg_filter);
    }
}

