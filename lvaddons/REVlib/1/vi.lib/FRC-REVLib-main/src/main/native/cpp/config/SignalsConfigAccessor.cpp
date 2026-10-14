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

#include "rev/config/SignalsConfigAccessor.h"

#include <stdint.h>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SignalsConfigAccessor::SignalsConfigAccessor(void* sparkHandle)
    : m_sparkHandle(sparkHandle) {}

int SignalsConfigAccessor::GetAppliedOutputPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus0Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetAppliedOutputAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_0, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetBusVoltagePeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus0Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetBusVoltageAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_0, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetOutputCurrentPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus0Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetOutputCurrentAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_0, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetMotorTemperaturePeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus0Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetMotorTemperatureAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_0, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetLimitsPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus0Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetLimitsAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_0, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetFaultsPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus1Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetFaultsAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_1, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetWarningsPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus1Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetWarningsAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_1, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetPrimaryEncoderVelocityPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus2Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetPrimaryEncoderVelocityAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_2, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetPrimaryEncoderPositionPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus2Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetPrimaryEncoderPositionAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_2, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetAnalogVoltagePeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus3Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetAnalogVoltageAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_3, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetAnalogVelocityPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus3Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetAnalogVelocityAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_3, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetAnalogPositionPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus3Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetAnalogPositionAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_3, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetExternalOrAltEncoderVelocityPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus4Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetExternalOrAltEncoderVelocityAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_4, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetExternalOrAltEncoderPositionPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus4Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetExternalOrAltEncoderPositionAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_4, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetAbsoluteEncoderVelocityPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus5Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetAbsoluteEncoderVelocityAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_5, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetAbsoluteEncoderPositionPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus5Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetAbsoluteEncoderPositionAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_5, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetUnadjustedDutyCyclePeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus6Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetUnadjustedDutyCyclePeriodAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_6, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetDutyCyclePeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus6Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetDutyCyclePeriodAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_6, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetIAccumulationPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus7Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetIAccumulationAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_7, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetSetpointPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus8Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetSetpointAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_8, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetIsAtSetpointPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus8Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetIsAtSetpointAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_8, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetSelectedSlotPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus8Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetSelectedSlotAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_8, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetMAXMotionSetpointPositionPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus9Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetMAXMotionSetpointPositionAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_9, &value);
    return value != 0;
}

int SignalsConfigAccessor::GetMAXMotionSetpointVelocityPeriodMs() {
    uint32_t value;
    c_Spark_GetParameterUint32(static_cast<c_Spark_handle>(m_sparkHandle),
                               c_Spark_kStatus9Period, &value);
    return value;
}

bool SignalsConfigAccessor::GetMAXMotionSetpointVelocityAlwaysOn() {
    uint8_t value;
    c_Spark_GetParameterBool(static_cast<c_Spark_handle>(m_sparkHandle),
                             c_Spark_kForceEnableStatus_9, &value);
    return value != 0;
}
