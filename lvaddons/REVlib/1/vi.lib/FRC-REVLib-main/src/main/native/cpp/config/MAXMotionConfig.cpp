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

#include "rev/config/MAXMotionConfig.h"

#include "rev/config/SparkParameters.h"

using namespace rev::spark;

namespace {

constexpr int kSlotOffset{5};

}  // namespace

MAXMotionConfig& MAXMotionConfig::Apply(MAXMotionConfig& config) {
    BaseConfig::Apply(config);
    return *this;
}

MAXMotionConfig& MAXMotionConfig::CruiseVelocity(double cruiseVelocity,
                                                 ClosedLoopSlot slot) {
    PutParameter(
        SparkParameter::kMAXMotionCruiseVelocity_0 + (slot * kSlotOffset),
        static_cast<float>(cruiseVelocity));
    return *this;
}

MAXMotionConfig& MAXMotionConfig::MaxAcceleration(double maxAcceleration,
                                                  ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kMAXMotionMaxAccel_0 + (slot * kSlotOffset),
                 static_cast<float>(maxAcceleration));
    return *this;
}

MAXMotionConfig& MAXMotionConfig::AllowedProfileError(double allowedError,
                                                      ClosedLoopSlot slot) {
    PutParameter(
        SparkParameter::kMAXMotionAllowedProfileError_0 + (slot * kSlotOffset),
        static_cast<float>(allowedError));
    return *this;
}

MAXMotionConfig& MAXMotionConfig::PositionMode(MAXMotionPositionMode mode,
                                               ClosedLoopSlot slot) {
    PutParameter(
        SparkParameter::kMAXMotionPositionMode_0 + (slot * kSlotOffset), mode);
    return *this;
}
