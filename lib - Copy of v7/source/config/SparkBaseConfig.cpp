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

#include "rev/config/SparkBaseConfig.h"

#include <string>

#include "rev/config/SparkParameters.h"

using namespace rev::spark;

SparkBaseConfig& SparkBaseConfig::Apply(SparkBaseConfig& config) {
    BaseConfig::Apply(config);
    absoluteEncoder.Apply(config.absoluteEncoder);
    analogSensor.Apply(config.analogSensor);
    closedLoop.Apply(config.closedLoop);
    encoder.Apply(config.encoder);
    limitSwitch.Apply(config.limitSwitch);
    signals.Apply(config.signals);
    softLimit.Apply(config.softLimit);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Apply(AbsoluteEncoderConfig& config) {
    absoluteEncoder.Apply(config);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Apply(AnalogSensorConfig& config) {
    analogSensor.Apply(config);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Apply(ClosedLoopConfig& config) {
    closedLoop.Apply(config);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Apply(EncoderConfig& config) {
    encoder.Apply(config);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Apply(LimitSwitchConfig& config) {
    limitSwitch.Apply(config);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Apply(SignalsConfig& config) {
    signals.Apply(config);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Apply(SoftLimitConfig& config) {
    softLimit.Apply(config);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::SetIdleMode(IdleMode idleMode) {
    PutParameter(SparkParameter::kIdleMode, idleMode);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Inverted(bool inverted) {
    PutParameter(SparkParameter::kInverted, inverted);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::SmartCurrentLimit(int stallLimit,
                                                    int freeLimit,
                                                    int limitRpm) {
    PutParameter(SparkParameter::kSmartCurrentStallLimit,
                 static_cast<uint32_t>(stallLimit));
    PutParameter(SparkParameter::kSmartCurrentFreeLimit,
                 static_cast<uint32_t>(freeLimit));
    PutParameter(SparkParameter::kSmartCurrentConfig,
                 static_cast<uint32_t>(limitRpm));
    return *this;
}

SparkBaseConfig& SparkBaseConfig::SecondaryCurrentLimit(double limit,
                                                        int chopCycles) {
    PutParameter(SparkParameter::kCurrentChop, static_cast<float>(limit));
    PutParameter(SparkParameter::kCurrentChopCycles,
                 static_cast<uint32_t>(chopCycles));
    return *this;
}

SparkBaseConfig& SparkBaseConfig::AdvanceCommutation(double byDegrees) {
    PutParameter(SparkParameter::kCommutationAdvance,
                 static_cast<float>(byDegrees));
    return *this;
}

SparkBaseConfig& SparkBaseConfig::OpenLoopRampRate(double rate) {
    if (rate != 0.0) {
        rate = 1.0 / rate;
    }

    PutParameter(SparkParameter::kOpenLoopRampRate, static_cast<float>(rate));
    return *this;
}

SparkBaseConfig& SparkBaseConfig::ClosedLoopRampRate(double rate) {
    if (rate != 0.0) {
        rate = 1.0 / rate;
    }

    PutParameter(SparkParameter::kClosedLoopRampRate, static_cast<float>(rate));
    return *this;
}

SparkBaseConfig& SparkBaseConfig::VoltageCompensation(double nominalVoltage) {
    PutParameter(SparkParameter::kCompensatedNominalVoltage,
                 static_cast<float>(nominalVoltage));
    PutParameter(SparkParameter::kVoltageCompensationMode, 2u);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::DisableVoltageCompensation() {
    PutParameter(SparkParameter::kVoltageCompensationMode, 0u);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Follow(int leaderCanId, bool invert) {
    PutParameter(SparkParameter::kFollowerModeLeaderId,
                 static_cast<uint32_t>(leaderCanId));
    PutParameter(SparkParameter::kFollowerModeIsInverted, invert);
    return *this;
}

SparkBaseConfig& SparkBaseConfig::Follow(const SparkBase& leader, bool invert) {
    return Follow(leader.GetDeviceId(), invert);
}

SparkBaseConfig& SparkBaseConfig::DisableFollowerMode() {
    PutParameter(SparkParameter::kFollowerModeLeaderId, 0u);
    PutParameter(SparkParameter::kFollowerModeIsInverted, false);
    return *this;
}

std::string SparkBaseConfig::Flatten() {
    std::string FlattenedString{
        BaseConfig::Flatten() + absoluteEncoder.Flatten() +
        analogSensor.Flatten() + encoder.Flatten() + limitSwitch.Flatten() +
        softLimit.Flatten() + closedLoop.Flatten() + signals.Flatten()};

    return FlattenedString;
}
