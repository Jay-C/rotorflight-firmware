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

#pragma once

#include <stdbool.h>

#include "common/time.h"
#include "common/filter.h"
#include "common/axis.h"

#include "pg/pg.h"


#if defined(STM32F1)
#define PID_PROCESS_DENOM_DEFAULT   8
#elif defined(STM32F3)
#define PID_PROCESS_DENOM_DEFAULT   4
#elif defined(STM32F411xE)
#define PID_PROCESS_DENOM_DEFAULT   2
#else
#define PID_PROCESS_DENOM_DEFAULT   1
#endif

#define MAX_PID_PROCESS_DENOM       16

#define PIDSUM_LIMIT                500
#define PIDSUM_SCALING              500

#define ROLL_P_TERM_SCALE           0.00333333f
#define ROLL_I_TERM_SCALE           0.05000000f
#define ROLL_D_TERM_SCALE           0.00005000f
#define ROLL_F_TERM_SCALE           0.01250000f

#define PITCH_P_TERM_SCALE          0.00333333f
#define PITCH_I_TERM_SCALE          0.05000000f
#define PITCH_D_TERM_SCALE          0.00005000f
#define PITCH_F_TERM_SCALE          0.01250000f

#define YAW_P_TERM_SCALE            0.03333333f
#define YAW_I_TERM_SCALE            0.25000000f
#define YAW_D_TERM_SCALE            0.00050000f
#define YAW_F_TERM_SCALE            0.01250000f

#define ITERM_RELAX_SETPOINT_THRESHOLD    40.0f


enum {
    FD_COLL = 3,
};

typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_YAW_CW = PID_YAW,
    PID_YAW_CCW,
    PID_ITEM_COUNT
} pidIndex_e;

typedef enum {
    ITERM_RELAX_OFF,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY,
    ITERM_RELAX_RP_INC,
    ITERM_RELAX_RPY_INC,
    ITERM_RELAX_COUNT,
} itermRelax_e;

typedef enum {
    ITERM_RELAX_GYRO,
    ITERM_RELAX_SETPOINT,
    ITERM_RELAX_TYPE_COUNT,
} itermRelaxType_e;

typedef enum {
    NORM_ABSOLUTE,
    NORM_LINEAR,
    NORM_NATURAL,
    NORM_TYPE_COUNT,
} normalization_e;

typedef struct pidf_s {
    uint16_t P;
    uint16_t I;
    uint16_t D;
    uint16_t F;
} pidf_t;

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

typedef struct pidAxisData_s {
    float Setpoint;
    float GyroRate;
    float Perror;
    float Ierror;
    float Derror;
    float P;
    float I;
    float D;
    float F;
    float Sum;
} pidAxisData_t;

typedef struct pidConfig_s {
    uint8_t pid_process_denom;     // PID controller vs gyro sampling rate
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);


#define MAX_PROFILE_NAME_LENGTH 8

typedef struct pidProfile_s
{
    char      profileName[MAX_PROFILE_NAME_LENGTH + 1]; // Descriptive name for profile

    pidf_t    pid[PID_ITEM_COUNT];

    uint8_t   debug_axis;                     // The axis for which debugging values are captured

    uint8_t   error_filter_hz[XYZ_AXIS_COUNT];  // Additional filtering on PID error

    uint8_t   angle_level_strength;
    uint8_t   angle_level_limit;              // Max angle in degrees in level mode

    uint8_t   horizon_level_strength;
    uint8_t   horizon_transition;
    uint8_t   horizon_tilt_effect;            // inclination factor for Horizon mode
    uint8_t   horizon_tilt_expert_mode;       // OFF or ON

    uint16_t  iterm_limit[XYZ_AXIS_COUNT];    // Maximum I-term for each axis
    uint8_t   iterm_decay;
    uint8_t   iterm_rotation;                 // rotates iterm to translate world errors to local coordinate system

    uint8_t   iterm_relax;                    // Enable iterm suppression during stick input
    uint8_t   iterm_relax_type;               // Specifies type of relax algorithm
    uint8_t   iterm_relax_cutoff[XYZ_AXIS_COUNT]; // Low pass filter which predicts average response to setpoint

    uint8_t   acro_trainer_gain;              // The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
    uint8_t   acro_trainer_angle_limit;       // Acro trainer roll/pitch angle limit in degrees
    uint16_t  acro_trainer_lookahead_ms;      // The lookahead window in milliseconds used to reduce overshoot

    uint8_t   abs_control;                    // Enable absolute control
    uint8_t   abs_control_gain;               // How strongly should the absolute accumulated error be corrected for
    uint8_t   abs_control_limit;              // Limit to the correction
    uint8_t   abs_control_error_limit;        // Limit to the accumulated error
    uint8_t   abs_control_cutoff;             // Cutoff frequency for path estimation in abs control

    uint8_t   ff_interpolate_sp;              // Calculate FF from interpolated setpoint
    uint8_t   ff_spike_limit;                 // FF stick extrapolation lookahead period in ms
    uint8_t   ff_max_rate_limit;              // Maximum setpoint rate percentage for FF
    uint8_t   ff_smooth_factor;               // Amount of smoothing for interpolated FF steps
    uint8_t   ff_boost;                       // amount of high-pass filtered FF to add to FF, 100 means 100% added

    uint8_t   yaw_pid_mode;                   // CW/CCW yaw mode
    int16_t   yaw_center_offset;              // Yaw zero offset
    uint8_t   yaw_cw_stop_gain;               // Yaw clockwise stop gain
    uint8_t   yaw_ccw_stop_gain;              // Yaw counter-clockwise stop gain
    uint16_t  yaw_cyclic_ff_gain;             // Feedforward for cyclic into Yaw
    uint16_t  yaw_collective_ff_gain;         // Feedforward for collective into Yaw
    uint16_t  yaw_collective_ff_impulse_gain; // Feedforward for collective impulse into Yaw
    uint8_t   yaw_collective_ff_impulse_freq; // Collective input impulse high-pass filter cutoff frequency

    uint16_t  pitch_collective_ff_gain;       // Collective to pitch feedforward gain
    uint16_t  pitch_collective_ff_impulse_gain; // Collective to pitch feedforward impulse gain

    uint8_t   cyclic_normalization;           // Type of pitch/roll rate normalization
    uint8_t   collective_normalization;       // Type of collective normalization
    uint8_t   normalization_min_ratio;        // Minimum headspeed ratio for normalization

    uint16_t  rescue_collective;              // Collective value for rescue
    uint16_t  rescue_boost;                   // Add  boost to rescue_collective until delay has expired
    uint8_t   rescue_delay;                   // Timer for non-inverted rescue in 0.1s steps

    uint16_t  gov_headspeed;                  // Governor full headspeed
    uint8_t   gov_gain;                       // Master gain
    uint8_t   gov_p_gain;                     // P-gain
    uint8_t   gov_i_gain;                     // I-gain
    uint8_t   gov_d_gain;                     // D-gain
    uint8_t   gov_f_gain;                     // Feedforward gain
    uint8_t   gov_tta_gain;                   // Tail Torque Assist gain
    uint8_t   gov_tta_limit;                  // Tail Torque Assist limit
    uint8_t   gov_cyclic_ff_weight;           // Cyclic feedforward weight
    uint8_t   gov_collective_ff_weight;       // Collective feedforward weight

} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);


void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

void pidInit(const pidProfile_t *pidProfile);
void pidInitFilters(const pidProfile_t *pidProfile);
void pidInitProfile(const pidProfile_t *pidProfile);
void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex);

void pidAcroTrainerInit(void);
void pidSetAcroTrainerState(bool newState);
void pidResetError(int axis);
void resetPidProfile(pidProfile_t *profile);

void pidInitSetpointDerivativeLpf(uint16_t filterCutoff, uint8_t debugAxis, uint8_t filterType);
void pidUpdateSetpointDerivativeLpf(uint16_t filterCutoff);

float pidGetDT();
float pidGetPidFrequency();
uint32_t pidGetLooptime();

bool pidAxisDebug(int axis);

float getPidSum(int axis);
const pidAxisData_t * getPidData(int axis);

float pidGetStabilizedCollective(void);

