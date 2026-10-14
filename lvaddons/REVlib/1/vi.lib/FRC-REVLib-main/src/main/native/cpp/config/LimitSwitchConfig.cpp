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

#include "rev/config/LimitSwitchConfig.h"

#include "rev/config/SparkMaxConfig.h"
#include "rev/config/SparkParameters.h"

using namespace rev::spark;

LimitSwitchConfig& LimitSwitchConfig::Apply(LimitSwitchConfig& config) {
    BaseConfig::Apply(config);
    return *this;
}

LimitSwitchConfig& LimitSwitchConfig::SetSparkMaxDataPortConfig() {
    PutParameter(
        SparkParameter::kCompatibilityPortConfig,
        SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
    return *this;
}

LimitSwitchConfig& LimitSwitchConfig::ForwardLimitSwitchTriggerBehavior(
    Behavior behavior) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kHardLimitFwdEn, behavior);
    return *this;
}

LimitSwitchConfig& LimitSwitchConfig::ForwardLimitSwitchType(Type type) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kLimitSwitchFwdPolarity,
                 type == Type::kNormallyClosed);
    return *this;
}

LimitSwitchConfig& LimitSwitchConfig::ForwardLimitSwitchPosition(
    double position) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kLimitSwitchFwdPosition,
                 static_cast<float>(position));
    return *this;
}

LimitSwitchConfig& LimitSwitchConfig::ReverseLimitSwitchTriggerBehavior(
    Behavior behavior) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kHardLimitRevEn, behavior);
    return *this;
}

LimitSwitchConfig& LimitSwitchConfig::ReverseLimitSwitchType(Type type) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kLimitSwitchRevPolarity,
                 type == Type::kNormallyClosed);
    return *this;
}

LimitSwitchConfig& LimitSwitchConfig::ReverseLimitSwitchPosition(
    double position) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kLimitSwitchRevPosition,
                 static_cast<float>(position));
    return *this;
}

LimitSwitchConfig& LimitSwitchConfig::LimitSwitchPositionSensor(
    FeedbackSensor sensor) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kLimitSwitchPositionSensor,
                 static_cast<uint32_t>(sensor));
    return *this;
}
