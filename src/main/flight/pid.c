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
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/dshot_command.h"
#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/motors.h"
#include "flight/trainer.h"
#include "flight/leveling.h"
#include "flight/setpoint.h"
#include "flight/gps_rescue.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

#undef USE_ABSOLUTE_CONTROL


PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 15);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .profileName = { 0, },
        .pid = {
            [PID_ROLL]    = { 10, 50,  0, 50, 100, 100, 100 },
            [PID_PITCH]   = { 10, 50,  0, 50, 100, 100, 100 },
            [PID_YAW]     = { 25, 50, 10,  0, 100, 100, 100 },
            [PID_WAY]     = { 25, 50, 10,  0, 100, 100, 100 },
        },
        .pid_mode = 1,
        .debug_axis = FD_ROLL,
        .error_cutoff = {  0,  0,  0 },
        .dterm_cutoff = { 20, 20, 40 },
        .fterm_cutoff = { 10, 10,  0 },
        .angle_level_strength = 50,
        .angle_level_limit = 55,
        .horizon_level_strength = 50,
        .horizon_transition = 75,
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .iterm_limit = { 120, 120, 120 },
        .iterm_decay = 25,
        .iterm_rotation = true,
        .iterm_relax = ITERM_RELAX_RPY,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .iterm_relax_cutoff = { 10, 10, 15 },
        .acro_trainer_gain = 75,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .abs_control = false,
        .abs_control_gain = 10,
        .abs_control_limit = 120,
        .abs_control_error_limit = 45,
        .abs_control_cutoff = 6,
        .ff_interpolate_sp = 0,
        .ff_spike_limit = 60,
        .ff_max_rate_limit = 100,
        .ff_smooth_factor = 37,
        .ff_boost = 15,
        .yaw_center_offset = 0,
        .yaw_cw_stop_gain = 100,
        .yaw_ccw_stop_gain = 100,
        .yaw_cyclic_ff_gain = 50,
        .yaw_collective_ff_gain = 100,
        .yaw_collective_ff_impulse_gain = 20,
        .yaw_collective_ff_impulse_freq = 100,
        .pitch_collective_ff_gain = 20,
        .pitch_collective_ff_impulse_gain = 0,
        .cyclic_normalization = NORM_ABSOLUTE,
        .collective_normalization = NORM_NATURAL,
        .normalization_min_ratio = 50,
        .rescue_collective = 0,
        .rescue_boost = 0,
        .rescue_delay = 35,
        .gov_headspeed = 1000,
        .gov_gain = 50,
        .gov_p_gain = 40,
        .gov_i_gain = 50,
        .gov_d_gain = 0,
        .gov_f_gain = 15,
        .gov_tta_gain = 0,
        .gov_tta_limit = 0,
        .gov_cyclic_ff_weight = 40,
        .gov_collective_ff_weight = 100,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}


static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;
static FAST_RAM_ZERO_INIT uint32_t pidLooptime;

static FAST_RAM_ZERO_INIT uint8_t pidDebugAxis;
static FAST_RAM_ZERO_INIT uint8_t pidMode;

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[PID_ITEM_COUNT];

static FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float pidSetPoint[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT pt1Filter_t errorFilter[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT pt1Filter_t dtermFilter[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT pt1Filter_t ftermFilter[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float Derr[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float Ierr[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float tailCWStopGain;
static FAST_RAM_ZERO_INIT float tailCCWStopGain;

static FAST_RAM_ZERO_INIT float tailCenterOffset;

static FAST_RAM_ZERO_INIT float tailCyclicFFGain;
static FAST_RAM_ZERO_INIT float tailCollectiveFFGain;
static FAST_RAM_ZERO_INIT float tailCollectiveImpulseFFGain;

static FAST_RAM_ZERO_INIT float collectiveDeflectionLPF;
static FAST_RAM_ZERO_INIT float collectiveDeflectionHPF;
static FAST_RAM_ZERO_INIT float collectiveImpulseFilterGain;

static FAST_RAM_ZERO_INIT float collectiveCommand;

static FAST_RAM_ZERO_INIT float pitchCollectiveFFGain;
static FAST_RAM_ZERO_INIT float pitchCollectiveImpulseFFGain;


#ifdef USE_ITERM_RELAX
static FAST_RAM_ZERO_INIT uint8_t itermRelax;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;
static FAST_RAM_ZERO_INIT pt1Filter_t itermRelaxLpf[XYZ_AXIS_COUNT];
#endif

#ifdef USE_ABSOLUTE_CONTROL
static FAST_RAM_ZERO_INIT bool  absoluteControl;
static FAST_RAM_ZERO_INIT pt1Filter_t acFilter[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acErrorLimit;
static FAST_RAM_ZERO_INIT float acLimit;
static FAST_RAM_ZERO_INIT float acGain;
#endif

static FAST_RAM_ZERO_INIT float errorLimit[XYZ_AXIS_COUNT];

#ifdef USE_ITERM_DECAY
static FAST_RAM_ZERO_INIT float itermDecay;
#endif

#ifdef USE_ITERM_ROTATION
static FAST_RAM_ZERO_INIT bool itermRotation;
#endif


float pidGetDT()
{
    return dT;
}

float pidGetPidFrequency()
{
    return pidFrequency;
}

uint32_t pidGetLooptime(void)
{
    return pidLooptime;
}

static void pidSetLooptime(uint32_t looptime)
{
    pidLooptime = looptime;
    dT = pidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
}

bool pidAxisDebug(int axis)
{
    return (axis == pidDebugAxis);
}

float getPidSum(int axis)
{
    return pidData[axis].Sum / PIDSUM_SCALING;
}

float getPidSetpoint(int axis)
{
    return pidSetPoint[axis];
}

const pidAxisData_t * getPidData(int axis)
{
    return &pidData[axis];
}

float pidGetStabilizedCollective(void)
{
    return collectiveCommand;
}


void pidInitFilters(const pidProfile_t *pidProfile)
{
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pt1FilterInit(&errorFilter[i], pt1FilterGain(constrain(pidProfile->error_cutoff[i], 1, 250), dT));
        pt1FilterInit(&dtermFilter[i], pt1FilterGain(constrain(pidProfile->dterm_cutoff[i], 1, 250), dT));
        pt1FilterInit(&ftermFilter[i], pt1FilterGain(constrain(pidProfile->fterm_cutoff[i], 1, 250), dT));
    }

#ifdef USE_ITERM_RELAX
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            uint8_t freq = constrain(pidProfile->iterm_relax_cutoff[i], 1, 50);
            pt1FilterInit(&itermRelaxLpf[i], pt1FilterGain(freq, dT));
        }

#ifdef USE_ABSOLUTE_CONTROL
        if (absoluteControl) {
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pt1FilterInit(&acFilter[i], pt1FilterGain(pidProfile->abs_control_cutoff, dT));
            }
        }
#endif
    }
#endif
}

void pidInitProfile(const pidProfile_t *pidProfile)
{
    pidDebugAxis = pidProfile->debug_axis;

    pidMode = constrain(pidProfile->pid_mode, 1, 6);

    // Roll axis
    pidCoefficient[PID_ROLL].Kp = ROLL_P_TERM_SCALE * pidProfile->pid[PID_ROLL].P *
        pidProfile->pid[PID_ROLL].PID_gain *
        pidProfile->pid[PID_ROLL].PI_gain *
        pidProfile->pid[PID_ROLL].PD_gain / 1000000.0f;

    pidCoefficient[PID_ROLL].Ki = ROLL_I_TERM_SCALE * pidProfile->pid[PID_ROLL].I *
        pidProfile->pid[PID_ROLL].PID_gain *
        pidProfile->pid[PID_ROLL].PI_gain / 10000.0f;

    pidCoefficient[PID_ROLL].Kd = ROLL_D_TERM_SCALE * pidProfile->pid[PID_ROLL].D *
        pidProfile->pid[PID_ROLL].PID_gain *
        pidProfile->pid[PID_ROLL].PD_gain / 10000.0f;

    pidCoefficient[PID_ROLL].Kf = ROLL_F_TERM_SCALE * pidProfile->pid[PID_ROLL].F;

    // Pitch axis
    pidCoefficient[PID_PITCH].Kp = PITCH_P_TERM_SCALE * pidProfile->pid[PID_PITCH].P *
        pidProfile->pid[PID_PITCH].PID_gain *
        pidProfile->pid[PID_PITCH].PI_gain *
        pidProfile->pid[PID_PITCH].PD_gain / 1000000.0f;

    pidCoefficient[PID_PITCH].Ki = PITCH_I_TERM_SCALE * pidProfile->pid[PID_PITCH].I *
        pidProfile->pid[PID_PITCH].PID_gain *
        pidProfile->pid[PID_PITCH].PI_gain / 10000.0f;

    pidCoefficient[PID_PITCH].Kd = PITCH_D_TERM_SCALE * pidProfile->pid[PID_PITCH].D *
        pidProfile->pid[PID_PITCH].PID_gain *
        pidProfile->pid[PID_PITCH].PD_gain / 10000.0f;

    pidCoefficient[PID_PITCH].Kf = PITCH_F_TERM_SCALE * pidProfile->pid[PID_PITCH].F;

    // Yaw axis
    pidCoefficient[PID_YAW].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_YAW].P *
        pidProfile->pid[PID_YAW].PID_gain *
        pidProfile->pid[PID_YAW].PI_gain *
        pidProfile->pid[PID_YAW].PD_gain / 1000000.0f;

    pidCoefficient[PID_YAW].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_YAW].I *
        pidProfile->pid[PID_YAW].PID_gain *
        pidProfile->pid[PID_YAW].PI_gain / 10000.0f;

    pidCoefficient[PID_YAW].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_YAW].D *
        pidProfile->pid[PID_YAW].PID_gain *
        pidProfile->pid[PID_YAW].PD_gain / 10000.0f;

    pidCoefficient[PID_YAW].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_YAW].F;

    // Yaw alt. axis
    pidCoefficient[PID_WAY].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_WAY].P *
        pidProfile->pid[PID_YAW].PID_gain *
        pidProfile->pid[PID_YAW].PI_gain *
        pidProfile->pid[PID_YAW].PD_gain / 1000000.0f;

    pidCoefficient[PID_WAY].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_WAY].I *
        pidProfile->pid[PID_YAW].PID_gain *
        pidProfile->pid[PID_YAW].PI_gain / 10000.0f;

    pidCoefficient[PID_WAY].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_WAY].D *
        pidProfile->pid[PID_YAW].PID_gain *
        pidProfile->pid[PID_YAW].PD_gain / 10000.0f;

    pidCoefficient[PID_WAY].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_WAY].F;

    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        errorLimit[i] = constrain(pidProfile->iterm_limit[i], 0, 1000);

#ifdef USE_ITERM_ROTATION
    itermRotation = pidProfile->iterm_rotation;
#endif
#ifdef USE_ITERM_DECAY
    itermDecay = (pidProfile->iterm_decay) ? dT * 10.0f / pidProfile->iterm_decay : 0;
#endif
#ifdef USE_ITERM_RELAX
    itermRelax = pidProfile->iterm_relax;
    itermRelaxType = pidProfile->iterm_relax_type;
#endif

#ifdef USE_ACC
    pidLevelInit(pidProfile);
#endif
#ifdef USE_ACRO_TRAINER
    acroTrainerInit(pidProfile);
#endif

#ifdef USE_ABSOLUTE_CONTROL
    absoluteControl = pidProfile->abs_control && pidProfile->abs_control_gain > 0;
    acGain = pidProfile->abs_control_gain / 10.0f;
    acLimit = pidProfile->abs_control_limit;
    acErrorLimit = pidProfile->abs_control_error_limit;

    float RC = acGain * pidCoefficient[PID_ROLL].Kp * ROLL_P_TERM_SCALE / ROLL_I_TERM_SCALE;
    float PC = acGain * pidCoefficient[PID_PITCH].Kp * PITCH_P_TERM_SCALE / PITCH_I_TERM_SCALE;
    float YC = acGain * pidCoefficient[PID_YAW].Kp * YAW_P_TERM_SCALE / YAW_I_TERM_SCALE;

    pidCoefficient[PID_ROLL].Ki  = MAX(0, pidCoefficient[PID_ROLL].Ki  - RC);
    pidCoefficient[PID_PITCH].Ki = MAX(0, pidCoefficient[PID_PITCH].Ki - PC);
    pidCoefficient[PID_YAW].Ki  = MAX(0, pidCoefficient[PID_YAW].Ki - YC);
#endif

#ifdef USE_INTERPOLATED_SP
    interpolatedSpInit(pidProfile);
#endif

    // Collective impulse high-pass filter
    collectiveImpulseFilterGain = pt1FilterGain(pidProfile->yaw_collective_ff_impulse_freq / 100.0f, dT);

    // Tail/yaw parameters
    tailCWStopGain = pidProfile->yaw_cw_stop_gain / 100.0f;
    tailCCWStopGain = pidProfile->yaw_ccw_stop_gain / 100.0f;
    tailCenterOffset = pidProfile->yaw_center_offset / 1000.0f;
    tailCyclicFFGain = pidProfile->yaw_cyclic_ff_gain;
    tailCollectiveFFGain = pidProfile->yaw_collective_ff_gain;
    tailCollectiveImpulseFFGain = pidProfile->yaw_collective_ff_impulse_gain;

    // Pitch parameters
    pitchCollectiveFFGain = pidProfile->pitch_collective_ff_gain;
    pitchCollectiveImpulseFFGain = pidProfile->pitch_collective_ff_impulse_gain;

    // Governor profile
    governorInitProfile(pidProfile);

    // PID filters
    pidInitFilters(pidProfile);
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetLooptime(gyro.targetLooptime);

    pidInitProfile(pidProfile);
}

static void pidReset(void)
{
    memset(pidData, 0, sizeof(pidData));
}

void pidResetError(int axis)
{
    pidData[axis].I = 0;
#ifdef USE_ABSOLUTE_CONTROL
    acError[axis] = 0;
#endif
}


void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

/*
 * 2D Rotation matrix
 *
 *        | cos(r)   -sin r |
 *    R = |                 |
 *        | sin(r)    cos r |
 *
 *
 *                3     5     7     9
 *               x     x     x     x
 * sin(x) = x - --- + --- - --- + --- - ...
 *               3!    5!    7!    9!
 *
 *                2     4     6     8
 *               x     x     x     x
 * cos(x) = 1 - --- + --- - --- + --- - ...
 *               2!    4!    6!    8!
 *
 *
 * For very small values of x, sin(x) ~= x and cos(x) ~= 1.
 *
 * In the use case below, using an additional term gives nearly 24bits of
 * resolution, which is close to what can be stored in a float anyway.
 *
 */

#ifdef __ZERO_ORDER_APPROX__
static inline void rotateVector(float *x, float *y, float r)
{
    float a,b;

    a = x[0] + y[0] * r;
    b = y[0] - x[0] * r;

    x[0] = a;
    y[0] = b;
}
#else

#define SIN2(R) ((R)-(R)*(R)*(R)/6)
#define COS2(R) (1.0f-(R)*(R)/2)

static inline void rotateVector(float *x, float *y, float r)
{
    float a,b,s,c;

    s = SIN2(r);
    c = COS2(r);

    a = x[0]*c + y[0]*s;
    b = y[0]*c - x[0]*s;

    x[0] = a;
    y[0] = b;
}
#endif

#ifdef USE_ITERM_ROTATION
static inline void rotateIterm(void)
{
    if (itermRotation) {
        rotateVector(&Ierr[X], &Ierr[Y], gyro.gyroADCf[Z]*dT*RAD);
    }
}
#endif

#ifdef USE_ABSOLUTE_CONTROL
static inline void rotateAxisError(void)
{
    if (itermRelax && absoluteControl) {
        rotateVector(&acError[X], &acError[Y], gyro.gyroADCf[Z]*dT*RAD);
    }
}

static FAST_CODE float applyAbsoluteControl(const int axis,
	const float gyroRate, const float currentPidSetpoint)
{
    const float acLpf = pt1FilterApply(&acFilter[axis], currentPidSetpoint);
    const float acHpf = fabsf(currentPidSetpoint - acLpf);
    const float acGmax = acLpf + 2 * acHpf;
    const float acGmin = acLpf - 2 * acHpf;
    const float acError1 = acGmax - gyroRate;
    const float acError2 = acGmin - gyroRate;

    float acErrorRate = 0, acCorrection = 0;

    if (gyroRate > acGmax) {
        acErrorRate = acError1; // < 0
    }
    else if (gyroRate < acGmin) {
        acErrorRate = acError2; // > 0
    }
    else {
        if (acError[axis] < 0)
            acErrorRate = acError1; // > 0
        else
            acErrorRate = acError2; // < 0
    }

    if (fabsf(acErrorRate * dT) > fabsf(acError[axis]))
        acErrorRate = -acError[axis] * pidFrequency;

    if (pidAxisSaturated(axis) || !isSpooledUp())
        acErrorRate = 0;

    acError[axis] = constrainf(acError[axis] + acErrorRate * dT, -acErrorLimit, acErrorLimit);

    acCorrection = constrainf(acError[axis] * acGain, -acLimit, acLimit);

    DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(acError[axis] * 10));
    DEBUG_SET(DEBUG_AC_CORRECTION, axis, lrintf(acCorrection * 10));

    if (pidAxisDebug(axis)) {
        DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(acCorrection * 10));
        DEBUG32_SET(DEBUG_ITERM_RELAX, 7, acCorrection * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 0, gyroRate * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 1, currentPidSetpoint * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 2, acLpf * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 3, acHpf * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 4, acError1 * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 5, acError2 * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 6, acErrorRate * 1000);
        DEBUG32_SET(DEBUG_AC_ERROR, 7, acCorrection * 1000);
    }

    return acCorrection;
}
#endif

#ifdef USE_ITERM_RELAX
static FAST_CODE float applyItermRelax(const int axis, const float iterm,
    float itermErrorRate, const float gyroRate, const float currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&itermRelaxLpf[axis], currentPidSetpoint);
    const float setpointHpf = fabsf(currentPidSetpoint - setpointLpf);

    // Always active on ROLL & PITCH; active also on YAW if _RPY
    if (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY || itermRelax == ITERM_RELAX_RPY_INC)
    {
        const float itermRelaxFactor = MAX(0, 1.0f - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);

        if ((itermRelax == ITERM_RELAX_RP_INC || itermRelax == ITERM_RELAX_RPY_INC) &&
            (((iterm > 0) && (itermErrorRate < 0)) || ((iterm < 0) && (itermErrorRate > 0)))) {
            // Iterm decreasing, no change
        }
        else {
            if (itermRelaxType == ITERM_RELAX_SETPOINT) {
                itermErrorRate *= itermRelaxFactor;
            } else if (itermRelaxType == ITERM_RELAX_GYRO ) {
                itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            }
        }

        if (pidAxisDebug(axis)) {
            DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
            DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
            DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(itermErrorRate));
            DEBUG32_SET(DEBUG_ITERM_RELAX, 0, currentPidSetpoint * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 1, gyroRate * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 2, setpointLpf * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 3, setpointHpf * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 4, itermRelaxFactor * 1000);
            DEBUG32_SET(DEBUG_ITERM_RELAX, 5, itermErrorRate * 1000);
        }
    }

    return itermErrorRate;
}
#endif


static FAST_CODE void pidApplyPrecomp(void)
{
    // Yaw precompensation direction
    float rotSign = mixerRotationSign();

    // Get stick throws (from previous cycle)
    float cyclicDeflection = getCyclicDeflection();
    float collectiveDeflection = getCollectiveDeflection();

    // Collective pitch impulse filter
    collectiveDeflectionLPF += (collectiveDeflection - collectiveDeflectionLPF) * collectiveImpulseFilterGain;
    collectiveDeflectionHPF = collectiveDeflection - collectiveDeflectionLPF;

    // Pitch precomp
    float pitchCollectiveFF = collectiveDeflection * pitchCollectiveFFGain;
    float pitchCollectiveImpulseFF = collectiveDeflectionHPF * pitchCollectiveImpulseFFGain;

    // Total pitch precomp
    float pitchPrecomp = pitchCollectiveFF + pitchCollectiveImpulseFF;

    // Add to PITCH feedforward
    pidData[FD_PITCH].F   += pitchPrecomp;
    pidData[FD_PITCH].Sum += pitchPrecomp;

    DEBUG_SET(DEBUG_PITCH_PRECOMP, 0, lrintf(collectiveDeflectionHPF * 1000));
    DEBUG_SET(DEBUG_PITCH_PRECOMP, 1, lrintf(pitchCollectiveFF));
    DEBUG_SET(DEBUG_PITCH_PRECOMP, 2, lrintf(pitchCollectiveImpulseFF));
    DEBUG_SET(DEBUG_PITCH_PRECOMP, 3, lrintf(pitchPrecomp));

    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 0, collectiveDeflection * 1000);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 1, collectiveDeflectionLPF * 1000);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 2, collectiveDeflectionHPF * 1000);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 3, pitchCollectiveFF * 10);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 4, pitchCollectiveImpulseFF * 10);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 5, pitchPrecomp * 10);

    // Collective components
    float tailCollectiveFF = fabsf(collectiveDeflection) * tailCollectiveFFGain;
    float tailCollectiveImpulseFF = fabsf(collectiveDeflectionHPF) * tailCollectiveImpulseFFGain;

    // Cyclic component
    float tailCyclicFF = fabsf(cyclicDeflection) * tailCyclicFFGain;

    // Calculate total precompensation
    float tailPrecomp = (tailCollectiveFF + tailCollectiveImpulseFF + tailCyclicFF + tailCenterOffset) * rotSign;

    // Add to YAW feedforward
    pidData[FD_YAW].F   += tailPrecomp;
    pidData[FD_YAW].Sum += tailPrecomp;

    DEBUG_SET(DEBUG_YAW_PRECOMP, 0, lrintf(tailCyclicFF));
    DEBUG_SET(DEBUG_YAW_PRECOMP, 1, lrintf(tailCollectiveFF));
    DEBUG_SET(DEBUG_YAW_PRECOMP, 2, lrintf(tailCollectiveImpulseFF));
    DEBUG_SET(DEBUG_YAW_PRECOMP, 3, lrintf(tailPrecomp));

    DEBUG32_SET(DEBUG_YAW_PRECOMP, 0, cyclicDeflection * 1000);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 1, tailCyclicFF * 10);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 2, collectiveDeflection * 1000);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 3, tailCollectiveFF * 10);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 4, collectiveDeflectionHPF * 1000);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 5, tailCollectiveImpulseFF * 10);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 6, tailPrecomp * 10);
}

static FAST_CODE void pidApplyCollective(void)
{
    if (FLIGHT_MODE(RESCUE_MODE))
        collectiveCommand = pidRescueCollective();
    else
        collectiveCommand = rcCommand[COLLECTIVE] * MIXER_RC_SCALING;
}



/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 1 - SAME AS RF-1.0
 **
 **   gyro ADC => errorFilter => Kp => P-term
 **   gyro dterm ADC => Kd => D-term
 **   gyro ADC => Relax => Ki => I-term
 **
 **   Using gyro-only D-term
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static FAST_CODE void pidApplyCyclicMode1(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = getSetpointRate(axis);

#ifdef USE_ACC
    // Apply leveling
    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)) {
        setpoint = pidLevelApply(axis, setpoint);
    }
#ifdef USE_ACRO_TRAINER
    else {
        // Apply trainer
        setpoint = acroTrainerApply(axis, setpoint);
    }
#endif
#endif

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;


  //// P-term

    // Calculate P-error rate
    float pterm = errorRate;
    if (pidProfile->error_cutoff[axis]) {
        pterm = pt1FilterApply(&errorFilter[axis], errorRate);
    }

    // Calculate P-component
    pidData[axis].P = pidCoefficient[axis].Kp * pterm;


  //// D-term

    // Calculate D-term with filtered ADCf
    float dError = -gyro.gyroDtermADCf[axis];
    float dterm = (dError - Derr[axis]) * pidFrequency;
    Derr[axis] = dError;

    // No D if axis saturated
    if (pidAxisSaturated(axis)) {
        dterm = 0;
    }

    // Calculate D-component
    pidData[axis].D = pidCoefficient[axis].Kd * dterm;


  //// I-term

    // Apply I-term relax
#ifdef USE_ITERM_RELAX
    if (itermRelax) {
        errorRate = applyItermRelax(axis, Ierr[axis], errorRate, gyroRate, setpoint);
    }
#endif
    float itermDelta = errorRate * dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        itermDelta = 0;
    }

    // Calculate I-component - Ki NOT included in Ierr
    Ierr[axis] = constrainf(Ierr[axis] + itermDelta, -errorLimit[axis], errorLimit[axis]);
    pidData[axis].I = pidCoefficient[axis].Ki * Ierr[axis];

    // Apply I-term error decay
#ifdef USE_ITERM_DECAY
    if (!isSpooledUp()) {
        Ierr[axis] -= Ierr[axis] * itermDecay;
    }
#endif


  //// F-term

    // Calculate feedforward component
    pidData[axis].F = pidCoefficient[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;

    // Save data
    pidSetPoint[axis] = setpoint;
}


static FAST_CODE void pidApplyYawMode1(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = getSetpointRate(axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;


  //// P-term

    // Calculate P-error rate
    float pterm = errorRate;
    if (pidProfile->error_cutoff[axis]) {
        pterm = pt1FilterApply(&errorFilter[axis], errorRate);
    }

    // Select stop gain
    float stopGain = (pterm > 0) ? tailCWStopGain : tailCCWStopGain;

    // Calculate P-component
    pidData[axis].P = pidCoefficient[axis].Kp * pterm * stopGain;


  //// D-term

    // Calculate D-term with filtered ADCf
    float dError = -gyro.gyroDtermADCf[axis];
    float dterm = (dError - Derr[axis]) * pidFrequency;
    Derr[axis] = dError;

    // No D if axis saturated
    if (pidAxisSaturated(axis)) {
        dterm = 0;
    }

    // Calculate D-component
    pidData[axis].D = pidCoefficient[axis].Kd * dterm * stopGain;


  //// I-term

    // Apply I-term relax
#ifdef USE_ITERM_RELAX
    if (itermRelax) {
        errorRate = applyItermRelax(axis, Ierr[axis], errorRate, gyroRate, setpoint);
    }
#endif
    float itermDelta = errorRate * dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        itermDelta = 0;
    }

    // Calculate I-component - Ki NOT included in Ierr
    Ierr[axis] = constrainf(Ierr[axis] + itermDelta, -errorLimit[axis], errorLimit[axis]);
    pidData[axis].I = pidCoefficient[axis].Ki * Ierr[axis];

    // Apply I-term decay
#ifdef USE_ITERM_DECAY
    if (!isSpooledUp()) {
        Ierr[axis] -= Ierr[axis] * itermDecay;
    }
#endif


  //// F-term

    // Calculate feedforward component
    pidData[axis].F = pidCoefficient[axis].Kf * setpoint;


  //// PID Sum

    // Calculate PID sum
    pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;

    // Save data
    pidSetPoint[axis] = setpoint;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 2
 **
 **   gyro ADC => errorFilter => Kp => P-term
 **   gyro ADC => errorFilter => dtermFilter => Kd => D-term
 **   gyro ADC => errorFilter => Relax => Ki => I-term
 **
 **   Stop gain on P/D
 **   NOT using gyroDtermADCf
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static FAST_CODE void pidApplyCyclicMode2(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = getSetpointRate(axis);

#ifdef USE_ACC
    // Apply leveling
    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)) {
        setpoint = pidLevelApply(axis, setpoint);
    }
#ifdef USE_ACRO_TRAINER
    else {
        // Apply trainer
        setpoint = acroTrainerApply(axis, setpoint);
    }
#endif
#endif

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&errorFilter[axis], errorRate);
    }


  //// P-term

    // Calculate P-component
    pidData[axis].P = pidCoefficient[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dError = pt1FilterApply(&dtermFilter[axis], errorRate);
    float dterm = (dError - Derr[axis]) * pidFrequency;
    Derr[axis] = dError;

    // Calculate D-component
    pidData[axis].D = pidCoefficient[axis].Kd * dterm;


  //// I-term

    // Apply I-term relax
#ifdef USE_ITERM_RELAX
    if (itermRelax) {
        errorRate = applyItermRelax(axis, Ierr[axis], errorRate, gyroRate, setpoint);
    }
#endif
    float itermDelta = errorRate * dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        if (Ierr[axis] * itermDelta > 0) // Same sign and not zero
            itermDelta = 0;
    }

    // Calculate I-component - Ki NOT included in Ierr
    Ierr[axis] = constrainf(Ierr[axis] + itermDelta, -errorLimit[axis], errorLimit[axis]);
    pidData[axis].I = pidCoefficient[axis].Ki * Ierr[axis];

    // Apply I-term error decay
#ifdef USE_ITERM_DECAY
    if (!isSpooledUp()) {
        Ierr[axis] -= Ierr[axis] * itermDecay;
    }
#endif


  //// F-term

    // Calculate feedforward component
    float fterm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fterm -= pt1FilterApply(&ftermFilter[axis], setpoint);
    }
    pidData[axis].F = pidCoefficient[axis].Kf * fterm;


  //// PID Sum

    // Calculate PID sum
    pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;

    // Save data
    pidSetPoint[axis] = setpoint;
}


static FAST_CODE void pidApplyYawMode2(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = getSetpointRate(axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate I-error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&errorFilter[axis], errorRate);
    }

    // Select stop gain
    float stopGain = (errorRate > 0) ? tailCWStopGain : tailCCWStopGain;


  //// P-term

    // Calculate P-component
    pidData[axis].P = pidCoefficient[axis].Kp * errorRate * stopGain;


  //// D-term

    // Calculate D-term
    float dterm = (errorRate - Derr[axis]) * pidFrequency;
    Derr[axis] = errorRate;

    // Filter D-term * stopGain
    dterm = pt1FilterApply(&dtermFilter[axis], dterm * stopGain);

    // Calculate D-component
    pidData[axis].D = pidCoefficient[axis].Kd * dterm;


  //// I-term

    // Apply I-term relax
#ifdef USE_ITERM_RELAX
    if (itermRelax) {
        errorRate = applyItermRelax(axis, Ierr[axis], errorRate, gyroRate, setpoint);
    }
#endif
    float itermDelta = errorRate * dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        if (Ierr[axis] * itermDelta > 0) // Same sign and not zero
            itermDelta = 0;
    }

    // Calculate I-component - Ki NOT included in Ierr
    Ierr[axis] = constrainf(Ierr[axis] + itermDelta, -errorLimit[axis], errorLimit[axis]);
    pidData[axis].I = pidCoefficient[axis].Ki * Ierr[axis];

    // Apply I-term decay
#ifdef USE_ITERM_DECAY
    if (!isSpooledUp()) {
        Ierr[axis] -= Ierr[axis] * itermDecay;
    }
#endif


  //// F-term

    // Calculate feedforward component
    float fterm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fterm -= pt1FilterApply(&ftermFilter[axis], setpoint);
    }

    // Calculate feedforward component
    pidData[axis].F = pidCoefficient[axis].Kf * fterm;


  //// PID Sum

    // Calculate PID sum
    pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;

    // Save data
    pidSetPoint[axis] = setpoint;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 3
 **
 **   gyro ADC => errorFilter => Kp => P-term
 **   gyro ADC => errorFilter => dtermFilter => Kd => D-term
 **   gyro ADC => errorFilter => Relax => Ki => I-term
 **
 **   Cyclic same as #2
 **   Yaw Stop gain on D only
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static FAST_CODE void pidApplyYawMode3(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = getSetpointRate(axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate I-error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&errorFilter[axis], errorRate);
    }

    // Select stop gain
    float stopGain = (errorRate > 0) ? tailCWStopGain : tailCCWStopGain;


  //// P-term

    // Calculate P-component - NO STOP GAIN
    pidData[axis].P = pidCoefficient[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term
    float dterm = (errorRate - Derr[axis]) * pidFrequency;
    Derr[axis] = errorRate;

    // Filter D-term * stopGain
    dterm = pt1FilterApply(&dtermFilter[axis], dterm * stopGain);

    // Calculate D-component
    pidData[axis].D = pidCoefficient[axis].Kd * dterm;


  //// I-term

    // Apply I-term relax
#ifdef USE_ITERM_RELAX
    if (itermRelax) {
        errorRate = applyItermRelax(axis, Ierr[axis], errorRate, gyroRate, setpoint);
    }
#endif
    float itermDelta = errorRate * dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        if (Ierr[axis] * itermDelta > 0) // Same sign and not zero
            itermDelta = 0;
    }

    // Calculate I-component - Ki NOT included in Ierr
    Ierr[axis] = constrainf(Ierr[axis] + itermDelta, -errorLimit[axis], errorLimit[axis]);
    pidData[axis].I = pidCoefficient[axis].Ki * Ierr[axis];

    // Apply I-term decay
#ifdef USE_ITERM_DECAY
    if (!isSpooledUp()) {
        Ierr[axis] -= Ierr[axis] * itermDecay;
    }
#endif


  //// F-term

    // Calculate feedforward component
    float fterm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fterm -= pt1FilterApply(&ftermFilter[axis], setpoint);
    }

    // Calculate feedforward component
    pidData[axis].F = pidCoefficient[axis].Kf * fterm;


  //// PID Sum

    // Calculate PID sum
    pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;

    // Save data
    pidSetPoint[axis] = setpoint;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 6 - TEST MODE
 **
 **   Yaw uses separate gains for CW & CCW
 **   Cyclic same as #2
 **
 **   gyro ADC => errorFilter => Kp => P-term
 **   gyro ADC => errorFilter => dtermFilter => Kd => D-term
 **   gyro ADC => errorFilter => Ki => I-term
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static FAST_CODE void pidApplyYawMode6(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = getSetpointRate(axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate I-error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&errorFilter[axis], errorRate);
    }

    // Select gains
    uint8_t index = (errorRate > 0) ? PID_YAW : PID_WAY;


  //// P-term

    // Calculate P-component
    pidData[axis].P = pidCoefficient[index].Kp * errorRate;


  //// D-term

    // Calculate D-term
    float dtermDelta = (errorRate - Derr[axis]) * pidFrequency;
    Derr[axis] = errorRate;

    // Filter D-term * Kd
    dtermDelta = pt1FilterApply(&dtermFilter[axis], pidCoefficient[index].Kd * dtermDelta);

    // Calculate D-component
    pidData[axis].D = dtermDelta;


  //// I-term

    // Calculate I-term delta
    float itermDelta = errorRate * dT * pidCoefficient[index].Ki;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        if (Ierr[axis] * itermDelta > 0) // Same sign and not zero
            itermDelta = 0;
    }

    // Calculate I-component
    float minKi = MIN(pidCoefficient[PID_YAW].Ki, pidCoefficient[PID_WAY].Ki);
    Ierr[axis] = constrainf(Ierr[axis] + itermDelta, -errorLimit[axis] * minKi, errorLimit[axis] * minKi);
    pidData[axis].I = Ierr[axis];

    // Apply I-term decay
#ifdef USE_ITERM_DECAY
    if (!isSpooledUp()) {
        Ierr[axis] -= Ierr[axis] * itermDecay;
    }
#endif


  //// F-term

    // Calculate feedforward component
    float fterm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fterm -= pt1FilterApply(&ftermFilter[axis], setpoint);
    }

    // Calculate feedforward component
    pidData[axis].F = pidCoefficient[index].Kf * fterm;


  //// PID Sum

    // Calculate PID sum
    pidData[axis].Sum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;

    // Save data
    pidSetPoint[axis] = setpoint;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/


FAST_CODE void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Rotate error around yaw axis
#ifdef USE_ITERM_ROTATION
    rotateIterm();
#endif
#ifdef USE_ABSOLUTE_CONTROL
    rotateAxisError();
#endif

    // Update rescue mode state
    pidRescueUpdate();

    // Apply PID for each axis
    switch (pidMode) {
        case 6:
            pidApplyCyclicMode2(pidProfile, FD_ROLL);     // Same as Mode#2
            pidApplyCyclicMode2(pidProfile, FD_PITCH);
            pidApplyYawMode6(pidProfile);
            break;

        case 3:
            pidApplyCyclicMode2(pidProfile, FD_ROLL);     // Same as Mode#2
            pidApplyCyclicMode2(pidProfile, FD_PITCH);
            pidApplyYawMode3(pidProfile);
            break;

        case 2:
            pidApplyCyclicMode2(pidProfile, FD_ROLL);
            pidApplyCyclicMode2(pidProfile, FD_PITCH);
            pidApplyYawMode2(pidProfile);
            break;

        default:
            pidApplyCyclicMode1(pidProfile, FD_ROLL);
            pidApplyCyclicMode1(pidProfile, FD_PITCH);
            pidApplyYawMode1(pidProfile);
            break;
    }

    // Calculate cyclic/collective precompensation
    pidApplyPrecomp();

    // Calculate stabilized collective
    pidApplyCollective();

    // Reset error if in passthrough mode
    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        pidResetError(FD_ROLL);
        pidResetError(FD_PITCH);
    }

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}

