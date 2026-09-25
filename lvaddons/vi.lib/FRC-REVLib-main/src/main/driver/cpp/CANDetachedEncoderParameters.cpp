/*
 * Copyright (c) 2025 REV Robotics
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

/*
 * This file is auto-generated. Do NOT modify it directly.
 * See https://github.com/REVrobotics/SparkParameters
 */

#include "rev/CANDetachedEncoderParameters.h"

#include <array>
#include <string>

namespace {

struct private_parameter_table_entry_t {
    c_Detached_ConfigParameter id;
    c_REVLib_ParameterType type;
    uint32_t defaultValue;
    std::string name;
};

const std::array<private_parameter_table_entry_t, c_Detached_NumParameters>
    s_Detached_ParameterTable = {{
        {c_Detached_kEncoderAverageDepth, c_REVLib_kUint32, 64, "Encoder Average Depth: Depth of encoder averaging filter"},
        {c_Detached_kEncoderInverted, c_REVLib_kBool, false, "Encoder Inverted: Invert encoder direction"},
        {c_Detached_kPositionConversionFactor, c_REVLib_kFloat32, 0x3f800000, "Position Conversion Factor: Factor to convert encoder counts or other position units to the desired position units."},
        {c_Detached_kVelocityConversionFactor, c_REVLib_kFloat32, 0x3f800000, "Velocity Conversion Factor: Factor to convert encoder counts or other velocity units to the desired velocity units."},
        {c_Detached_kDutyCycleZeroCentered, c_REVLib_kBool, false, "Duty Cycle Zero Centered: Center Duty Cycle readings at 0"},
        {c_Detached_kDutyCycleAverageDepth, c_REVLib_kUint32, 7, "Duty Cycle Average Depth: Log base 2 of the number of samples to average for velocity data based on duty cycle encoder input"},
        {c_Detached_kDutyCycleOffset, c_REVLib_kFloat32, 0x00000000, "Duty Cycle Offset: Offset Duty Cycle encoder readings by a fixed value (0 to 1)"},
        {c_Detached_kStatus0Period, c_REVLib_kUint32, 10, "Status 0 Period: Status frame 0 period, in us"},
        {c_Detached_kStatus1Period, c_REVLib_kUint32, 250, "Status 1 Period: Status frame 1 period, in us"},
        {c_Detached_kStatus2Period, c_REVLib_kUint32, 20, "Status 2 Period: Status frame 2 period, in us"},
        {c_Detached_kStatus3Period, c_REVLib_kUint32, 20, "Status 3 Period: Status frame 3 period, in us"},
        {c_Detached_kStatus4Period, c_REVLib_kUint32, 20, "Status 4 Period: Status frame 4 period, in us"},
        {c_Detached_kAngleConversionFactor, c_REVLib_kFloat32, 0x3f800000, "Angle Conversion Factor: Factor to convert encoder counts or other angle units to the desired angle units."},
        {c_Detached_kDutyCycleStartPulseUs, c_REVLib_kFloat32, 0x00000000, "Duty Cycle Start Pulse Us: Sets the expected start pulse of the input duty cycle signal in us."},
        {c_Detached_kDutyCycleEndPulseUs, c_REVLib_kFloat32, 0x00000000, "Duty Cycle End Pulse Us: Sets the expected end pulse of the input duty cycle signal in us."},
        {c_Detached_kDutyCyclePeriodUs, c_REVLib_kFloat32, 0x00000000, "Duty Cycle Period Us: Sets the expected absolute position signal period in us."},
    }};

}  // namespace

c_REVLib_ParameterType c_Detached_GetParameterType(
    c_Detached_ConfigParameter parameterId) {
    return s_Detached_ParameterTable[parameterId].type;
}

uint32_t c_Detached_GetParameterDefaultValue(c_Detached_ConfigParameter parameterId) {
    return s_Detached_ParameterTable[parameterId].defaultValue;
}

const char* c_Detached_GetParameterName(c_Detached_ConfigParameter parameterId) {
    return s_Detached_ParameterTable[parameterId].name.c_str();
}

c_Detached_ConfigParameter c_Detached_GetConfigParameter(
    c_Detached_ConfigParameter parameterId) {
    return s_Detached_ParameterTable[parameterId].id;
}
