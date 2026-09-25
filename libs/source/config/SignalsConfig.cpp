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

#include "rev/config/SignalsConfig.h"

#include <algorithm>

#include "rev/config/SparkParameters.h"

using namespace rev::spark;

SignalsConfig& SignalsConfig::Apply(SignalsConfig& config) {
    BaseConfig::Apply(config);
    return *this;
}

SignalsConfig& SignalsConfig::AppliedOutputPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus0Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::BusVoltagePeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus0Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::OutputCurrentPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus0Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::MotorTemperaturePeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus0Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::LimitsPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus0Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::FaultsPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus1Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::FaultsAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_1, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::WarningsPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus1Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::WarningsAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_1, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::PrimaryEncoderVelocityPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus2Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::PrimaryEncoderVelocityAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_2, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::PrimaryEncoderPositionPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus2Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::PrimaryEncoderPositionAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_2, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::AnalogVoltagePeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus3Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::AnalogVoltageAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_3, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::AnalogVelocityPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus3Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::AnalogVelocityAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_3, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::AnalogPositionPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus3Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::AnalogPositionAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_3, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::ExternalOrAltEncoderVelocity(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus4Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::ExternalOrAltEncoderVelocityAlwaysOn(
    bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_4, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::ExternalOrAltEncoderPosition(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus4Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::ExternalOrAltEncoderPositionAlwaysOn(
    bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_4, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::AbsoluteEncoderVelocityPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus5Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::AbsoluteEncoderVelocityAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_5, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::AbsoluteEncoderPositionPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus5Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::AbsoluteEncoderPositionAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_5, enabled);
    return *this;
}

rev::spark::SignalsConfig& SignalsConfig::UnadjustedDutyCyclePeriodMs(
    int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus6Period, periodMs);
    return *this;
}

rev::spark::SignalsConfig& SignalsConfig::UnadjustedDutyCyclePeriodAlwaysOn(
    bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_6, enabled);
    return *this;
}

rev::spark::SignalsConfig& SignalsConfig::DutyCyclePeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus6Period, periodMs);
    return *this;
}

rev::spark::SignalsConfig& SignalsConfig::DutyCyclePeriodAlwaysOn(
    bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_6, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::IAccumulationPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus7Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::IAccumulationAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_7, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::SetpointPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus8Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::SetpointAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_8, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::IsAtSetpointPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus8Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::IsAtSetpointAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_8, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::SelectedSlotPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus8Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::SelectedSlotAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_8, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::MaxMotionSetpointPositionPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus9Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::MaxMotionSetpointPositionAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_9, enabled);
    return *this;
}

SignalsConfig& SignalsConfig::MaxMotionSetpointVelocityPeriodMs(int periodMs) {
    SetPeriodMsCore(SparkParameter::kStatus9Period, periodMs);
    return *this;
}

SignalsConfig& SignalsConfig::MaxMotionSetpointVelocityAlwaysOn(bool enabled) {
    SetAlwaysOnCore(SparkParameter::kForceEnableStatus_9, enabled);
    return *this;
}

void SignalsConfig::SetPeriodMsCore(int parameterId, int periodMs) {
    auto optValue = GetParameter(parameterId);

    if (!optValue) {
        PutParameter(parameterId, static_cast<uint32_t>(periodMs));
    } else {
        const int currentPeriodMs = std::get<uint32_t>(*optValue);
        PutParameter(parameterId, static_cast<uint32_t>(
                                      std::min(currentPeriodMs, periodMs)));
    }
}

void SignalsConfig::SetAlwaysOnCore(int parameterId, bool enabled) {
    // Ignore status 0
    if (parameterId == SparkParameter::kForceEnableStatus_0) {
        return;
    }

    auto optValue = GetParameter(parameterId);

    if (!optValue) {
        PutParameter(parameterId, enabled ? 1u : 0u);
    } else {
        const int currentEnabled = std::get<uint32_t>(*optValue);
        PutParameter(parameterId, (enabled || currentEnabled == 1u) ? 1u : 0u);
    }
}
