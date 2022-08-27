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

#pragma once

#include <stdbool.h>

#include "fc/rc_modes.h"
#include "flight/pid.h"
#include "pg/pg.h"

typedef enum {
    ADJUSTMENT_NONE = 0,
    ADJUSTMENT_PITCH_RATE,
    ADJUSTMENT_ROLL_RATE,
    ADJUSTMENT_YAW_RATE,
    ADJUSTMENT_PITCH_RC_RATE,       // 4
    ADJUSTMENT_ROLL_RC_RATE,
    ADJUSTMENT_YAW_RC_RATE,
    ADJUSTMENT_PITCH_RC_EXPO,       // 7
    ADJUSTMENT_ROLL_RC_EXPO,
    ADJUSTMENT_YAW_RC_EXPO,
    ADJUSTMENT_PITCH_P_GAIN,        // 10
    ADJUSTMENT_PITCH_I_GAIN,
    ADJUSTMENT_PITCH_D_GAIN,
    ADJUSTMENT_PITCH_F_GAIN,
    ADJUSTMENT_ROLL_P_GAIN,         // 14
    ADJUSTMENT_ROLL_I_GAIN,
    ADJUSTMENT_ROLL_D_GAIN,
    ADJUSTMENT_ROLL_F_GAIN,
    ADJUSTMENT_YAW_P_GAIN,          // 18
    ADJUSTMENT_YAW_I_GAIN,
    ADJUSTMENT_YAW_D_GAIN,
    ADJUSTMENT_YAW_F_GAIN,
    ADJUSTMENT_YAW_CENTER,          // 22
    ADJUSTMENT_YAW_CW_GAIN,
    ADJUSTMENT_YAW_CCW_GAIN,
    ADJUSTMENT_YAW_CYCLIC_FF,       // 25
    ADJUSTMENT_YAW_COLLECTIVE_FF,
    ADJUSTMENT_YAW_IMPULSE_FF,
    ADJUSTMENT_RESCUE_COLLECTIVE,   // 28
    ADJUSTMENT_RESCUE_COLL_BOOST,
    ADJUSTMENT_ANGLE_LEVEL_GAIN,    // 30
    ADJUSTMENT_HORIZON_LEVEL_GAIN,
    ADJUSTMENT_ACRO_TRAINER_GAIN,
    ADJUSTMENT_GOV_GAIN,            // 33
    ADJUSTMENT_GOV_P_GAIN,
    ADJUSTMENT_GOV_I_GAIN,
    ADJUSTMENT_GOV_D_GAIN,
    ADJUSTMENT_GOV_F_GAIN,
    ADJUSTMENT_GOV_TTA_GAIN,        // 38
    ADJUSTMENT_GOV_CYCLIC_FF,
    ADJUSTMENT_GOV_COLLECTIVE_FF,
    ADJUSTMENT_RATE_PROFILE,        // 41
    ADJUSTMENT_PID_PROFILE,
    ADJUSTMENT_OSD_PROFILE,
    ADJUSTMENT_LED_PROFILE,
    ADJUSTMENT_PITCH_COLL_FF,       // 45
    ADJUSTMENT_PITCH_IMPULSE_FF,
    ADJUSTMENT_SWASH_PHASE,
    ADJUSTMENT_PITCH_PID_GAIN,      // 48
    ADJUSTMENT_PITCH_PI_GAIN,
    ADJUSTMENT_PITCH_PD_GAIN,
    ADJUSTMENT_ROLL_PID_GAIN,       // 51
    ADJUSTMENT_ROLL_PI_GAIN,
    ADJUSTMENT_ROLL_PD_GAIN,
    ADJUSTMENT_YAW_PID_GAIN,        // 54
    ADJUSTMENT_YAW_PI_GAIN,
    ADJUSTMENT_YAW_PD_GAIN,
    ADJUSTMENT_PITCH_ERROR_CUTOFF,  // 57
    ADJUSTMENT_PITCH_DTERM_CUTOFF,
    ADJUSTMENT_PITCH_FTERM_CUTOFF,
    ADJUSTMENT_ROLL_ERROR_CUTOFF,   // 60
    ADJUSTMENT_ROLL_DTERM_CUTOFF,
    ADJUSTMENT_ROLL_FTERM_CUTOFF,
    ADJUSTMENT_YAW_ERROR_CUTOFF,    // 63
    ADJUSTMENT_YAW_DTERM_CUTOFF,
    ADJUSTMENT_YAW_FTERM_CUTOFF,
    ADJUSTMENT_WAY_P_GAIN,          // 66
    ADJUSTMENT_WAY_I_GAIN,
    ADJUSTMENT_WAY_D_GAIN,
    ADJUSTMENT_WAY_F_GAIN,
    ADJUSTMENT_FUNCTION_COUNT       // 70
} adjustmentFunction_e;

typedef enum {
    ADJUSTMENT_TYPE_NONE  = 0,
    ADJUSTMENT_TYPE_RATE  = BIT(0),
    ADJUSTMENT_TYPE_PID   = BIT(1),
    ADJUSTMENT_TYPE_GOV   = BIT(2),
    ADJUSTMENT_TYPE_MIX   = BIT(3),
} adjustmentType_e;

typedef struct {
    timeMs_t timer;
    int16_t  value;
} adjustmentState_t;

typedef struct {
    char *  cfgName;
    uint8_t cfgType;
    int16_t cfgMin;
    int16_t cfgMax;
} adjustmentConfig_t;

typedef struct {
    uint8_t enaChannel;
    channelRange_t enaRange;
    uint8_t adjFunction;
    uint8_t adjChannel;
    uint8_t adjStep;
    int16_t adjMin;
    int16_t adjMax;
} adjustmentRange_t;


#define MAX_ADJUSTMENT_RANGE_COUNT 32

PG_DECLARE_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges);


void adjustmentRangeInit(void);
void adjustmentRangeReset(int index);

void processRcAdjustments(void);

const char *getAdjustmentsRangeName(void);
int getAdjustmentsRangeValue(void);

