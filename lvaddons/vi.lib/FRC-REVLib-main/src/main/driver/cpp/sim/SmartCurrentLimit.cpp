/*
 * Copyright (c) 2024-2026 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//
// Created by Rylan on 8/7/2024.
//

#include "rev/sim/SmartCurrentLimit.h"

#define MIN_DC_VAL 0.00f
#define REBOUND_FACTOR 0.0175f
#define REBOUND_RAMP 1.75f

float constrain_value(float input, float min, float max) {
    input = input > max ? max : input;
    input = input < min ? min : input;
    return input;
}

float c_SIM_Spark_RunSmartCurrentLimit(float dutyCycleCommand, float limit,
                                       float phaseCurrent, float movingAvg,
                                       float* reboundFactor) {
    const float dc = fabsf(dutyCycleCommand);
    const float dcLimitedPrev = fabsf(movingAvg);
    const float currLim = limit;
    const float phaseCurr = fabsf(phaseCurrent);

    float dcAdjusted = dc;
    if (phaseCurr > currLim && ((dutyCycleCommand > 0) == (phaseCurrent > 0))) {
        // Multiply the ratio of the Duty cycle that created the outlying phase
        // current
        dcAdjusted = dcLimitedPrev * (currLim / phaseCurr);
        // Reset the rebounding multiplier that returns the limited duty cycle
        // to the duty cycle command
        *reboundFactor = REBOUND_FACTOR;
    } else {
        if (dc > dcLimitedPrev) {
            // exponentially restore the duty cycle requested
            dcAdjusted =
                dcLimitedPrev + ((*reboundFactor) * (dc - dcLimitedPrev));
        }
        *reboundFactor *= REBOUND_RAMP;
        if (*reboundFactor > 1) *reboundFactor = 1;
    }
    // Overwrite limited duty cycle within the minimum and requested DC
    dcAdjusted = constrain_value(dcAdjusted, MIN_DC_VAL, dc);

    return copysign(dcAdjusted, dutyCycleCommand);
}
