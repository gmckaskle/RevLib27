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

#include "rev/config/ClosedLoopConfig.h"

#include <string>

#include "rev/DetachedEncoder.h"
#include "rev/config/SparkParameters.h"

using namespace rev::spark;

namespace {

static constexpr int kSlotOffset{8};
static constexpr int kAdditionalSlotOffset{4};

}  // namespace

ClosedLoopConfig& ClosedLoopConfig::Apply(ClosedLoopConfig& config) {
    BaseConfig::Apply(config);
    maxMotion.Apply(config.maxMotion);
    feedForward.Apply(config.feedForward);
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::Apply(MAXMotionConfig& config) {
    maxMotion.Apply(config);
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::Apply(FeedForwardConfig& config) {
    feedForward.Apply(config);
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::Pid(double p, double i, double d,
                                        ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kP_0 + (slot * kSlotOffset),
                 static_cast<float>(p));
    PutParameter(SparkParameter::kI_0 + (slot * kSlotOffset),
                 static_cast<float>(i));
    PutParameter(SparkParameter::kD_0 + (slot * kSlotOffset),
                 static_cast<float>(d));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::P(double p, ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kP_0 + (slot * kSlotOffset),
                 static_cast<float>(p));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::I(double i, ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kI_0 + (slot * kSlotOffset),
                 static_cast<float>(i));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::D(double d, ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kD_0 + (slot * kSlotOffset),
                 static_cast<float>(d));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::DFilter(double dFilter,
                                            ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kDFilter_0 + (slot * kSlotOffset),
                 static_cast<float>(dFilter));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::IZone(double iZone, ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kIZone_0 + (slot * kSlotOffset),
                 static_cast<float>(iZone));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::MinOutput(double minOutput,
                                              ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kOutputMin_0 + (slot * kSlotOffset),
                 static_cast<float>(minOutput));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::MaxOutput(double maxOutput,
                                              ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kOutputMax_0 + (slot * kSlotOffset),
                 static_cast<float>(maxOutput));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::OutputRange(double minOutput,
                                                double maxOutput,
                                                ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kOutputMin_0 + (slot * kSlotOffset),
                 static_cast<float>(minOutput));
    PutParameter(SparkParameter::kOutputMax_0 + (slot * kSlotOffset),
                 static_cast<float>(maxOutput));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::IMaxAccum(double iMaxAccum,
                                              ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kIMaxAccum_0 + (slot * kAdditionalSlotOffset),
                 static_cast<float>(iMaxAccum));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::AllowedClosedLoopError(
    double allowedError, ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kAllowedClosedLoopError_0 +
                     (slot * kAdditionalSlotOffset),
                 static_cast<float>(allowedError));
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::PositionWrappingEnabled(bool enabled) {
    PutParameter(SparkParameter::kPositionPIDWrapEnable, enabled);
    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::SetFeedbackSensor(FeedbackSensor sensor) {
    switch (sensor) {
        case FeedbackSensor::kNoSensor:
            [[fallthrough]];
        case FeedbackSensor::kPrimaryEncoder:
            [[fallthrough]];
        case FeedbackSensor::kAnalogSensor:
            [[fallthrough]];
        case FeedbackSensor::kAlternateOrExternalEncoder:
            [[fallthrough]];
        case FeedbackSensor::kAbsoluteEncoder:
            PutParameter(SparkParameter::kClosedLoopControlSensor,
                         static_cast<uint32_t>(sensor));
            break;
        default:
            break;
    }

    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::SetFeedbackSensor(
    FeedbackSensor sensor, int detachedEncoderDeviceId) {
    switch (sensor) {
        case FeedbackSensor::kDetachedAbsoluteEncoder:
            [[fallthrough]];
        case FeedbackSensor::kDetachedRelativeEncoder:
            PutParameter(SparkParameter::kClosedLoopControlSensor,
                         static_cast<uint32_t>(sensor));
            PutParameter(SparkParameter::kDetachedEncoderDeviceID,
                         static_cast<uint32_t>(detachedEncoderDeviceId));
            break;
        default:
            break;
    }

    return *this;
}

ClosedLoopConfig& ClosedLoopConfig::SetFeedbackSensor(
    FeedbackSensor sensor,
    const rev::detached::DetachedEncoder& detachedEncoder) {
    return SetFeedbackSensor(sensor, detachedEncoder.GetDeviceId());
}

std::string ClosedLoopConfig::Flatten() {
    std::string FlattenedString{BaseConfig::Flatten() + maxMotion.Flatten() +
                                feedForward.Flatten()};

    return FlattenedString;
}
