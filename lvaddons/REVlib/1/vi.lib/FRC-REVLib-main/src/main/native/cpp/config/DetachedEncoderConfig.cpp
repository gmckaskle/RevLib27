/*
 * Copyright (c) 2025-2026 REV Robotics
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

#include "rev/config/DetachedEncoderConfig.h"

#include <string>

#include "rev/config/DetachedEncoderParameters.h"

using namespace rev::detached;

DetachedEncoderConfig& DetachedEncoderConfig::Apply(
    DetachedEncoderConfig& config) {
    BaseConfig::Apply(config);
    signals.Apply(config.signals);
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::Apply(SignalsConfig& config) {
    signals.Apply(config);
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::Inverted(bool inverted) {
    PutParameter(DetachedEncoderParameter::kEncoderInverted, inverted);
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::PositionConversionFactor(
    double factor) {
    PutParameter(DetachedEncoderParameter::kPositionConversionFactor,
                 static_cast<float>(factor));
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::VelocityConversionFactor(
    double factor) {
    PutParameter(DetachedEncoderParameter::kVelocityConversionFactor,
                 static_cast<float>(factor));
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::VelocityAverageDepth(int depth) {
    PutParameter(DetachedEncoderParameter::kEncoderAverageDepth,
                 static_cast<uint32_t>(depth));
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::AngleConversionFactor(
    double factor) {
    PutParameter(DetachedEncoderParameter::kAngleConversionFactor,
                 static_cast<float>(factor));
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::ZeroOffset(double offset) {
    PutParameter(DetachedEncoderParameter::kDutyCycleOffset,
                 static_cast<float>(offset));
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::ZeroCentered(bool zeroCentered) {
    PutParameter(DetachedEncoderParameter::kDutyCycleZeroCentered,
                 zeroCentered);
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::StartPulseUs(
    double startPulseUs) {
    PutParameter(DetachedEncoderParameter::kDutyCycleStartPulseUs,
                 static_cast<float>(startPulseUs));
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::EndPulseUs(double endPulseUs) {
    PutParameter(DetachedEncoderParameter::kDutyCycleEndPulseUs,
                 static_cast<float>(endPulseUs));
    return *this;
}

DetachedEncoderConfig& DetachedEncoderConfig::AbsolutePeriodUs(
    double periodUs) {
    PutParameter(DetachedEncoderParameter::kDutyCyclePeriodUs,
                 static_cast<float>(periodUs));
    return *this;
}

std::string DetachedEncoderConfig::Flatten() {
    std::string FlattenedString{BaseConfig::Flatten() + signals.Flatten()};

    return FlattenedString;
}
