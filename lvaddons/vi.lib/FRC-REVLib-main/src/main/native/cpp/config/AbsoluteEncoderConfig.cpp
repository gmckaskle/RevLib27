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

#include "rev/config/AbsoluteEncoderConfig.h"

#include "rev/config/SparkMaxConfig.h"
#include "rev/config/SparkParameters.h"

using namespace rev::spark;

AbsoluteEncoderConfig& AbsoluteEncoderConfig::Apply(
    AbsoluteEncoderConfig& config) {
    BaseConfig::Apply(config);
    return *this;
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::SetSparkMaxDataPortConfig() {
    PutParameter(
        SparkParameter::kCompatibilityPortConfig,
        SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
    return *this;
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::Inverted(bool inverted) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kDutyCycleInverted, inverted);
    return *this;
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::PositionConversionFactor(
    double factor) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kDutyCyclePositionFactor,
                 static_cast<float>(factor));
    return *this;
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::VelocityConversionFactor(
    double factor) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kDutyCycleVelocityFactor,
                 static_cast<float>(factor));
    return *this;
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::ZeroOffset(double offset) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kDutyCycleOffset, static_cast<float>(offset));
    return *this;
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::AverageDepth(int depth) {
    SetSparkMaxDataPortConfig();

    uint32_t depthIndex;
    switch (depth) {
        case 1:
            depthIndex = 0;
            break;
        case 2:
            depthIndex = 1;
            break;
        case 4:
            depthIndex = 2;
            break;
        case 8:
            depthIndex = 3;
            break;
        case 16:
            depthIndex = 4;
            break;
        case 32:
            depthIndex = 5;
            break;
        case 64:
            depthIndex = 6;
            break;
        default:
            depthIndex = 7;
    }

    PutParameter(SparkParameter::kDutyCycleAverageDepth, depthIndex);
    return *this;
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::StartPulseUs(
    double startPulseUs) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kDutyCycleEncoderStartPulseUs,
                 static_cast<float>(startPulseUs));
    return *this;
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::EndPulseUs(double endPulseUs) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kDutyCycleEncoderEndPulseUs,
                 static_cast<float>(endPulseUs));
    return *this;
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::ZeroCentered(bool zeroCentered) {
    return RangeOffset(0.0);
}

AbsoluteEncoderConfig& AbsoluteEncoderConfig::RangeOffset(double offset) {
    SetSparkMaxDataPortConfig();

    PutParameter(SparkParameter::kDutyCycleRangeOffset,
                 static_cast<float>(offset));
    return *this;
}
