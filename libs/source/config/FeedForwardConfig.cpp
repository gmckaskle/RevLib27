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

#include "rev/config/FeedForwardConfig.h"

#include <wpi/system/Errors.hpp>

#include "rev/config/SparkParameters.h"

using namespace rev::spark;

namespace {

static constexpr int kPidSlotOffset{kP_1 - kP_0};
static constexpr int kFFSlotOffset{kS_1 - kS_0};

}  // namespace

FeedForwardConfig& FeedForwardConfig::Apply(FeedForwardConfig& config) {
    BaseConfig::Apply(config);
    return *this;
}

FeedForwardConfig& FeedForwardConfig::kS(double kS, ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kS_0 + (slot * kFFSlotOffset),
                 static_cast<float>(kS));
    return *this;
}

FeedForwardConfig& FeedForwardConfig::kV(double kV, ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kV_0 + (slot * kPidSlotOffset),
                 static_cast<float>(kV));
    return *this;
}

FeedForwardConfig& FeedForwardConfig::kA(double kA, ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kA_0 + (slot * kFFSlotOffset),
                 static_cast<float>(kA));
    return *this;
}

FeedForwardConfig& FeedForwardConfig::kG(double kG, ClosedLoopSlot slot) {
    if (GetParameter(SparkParameter::kCos_0 + (slot * kFFSlotOffset)) !=
        std::nullopt) {
        WPILIB_ReportError(
            wpi::warn::Warning,
            "kG is overriden by kCos, only kCos will be used on Closed Loop "
            "Slot " +
                std::to_string(slot));
        return *this;
    }
    PutParameter(SparkParameter::kG_0 + (slot * kFFSlotOffset),
                 static_cast<float>(kG));
    return *this;
}

FeedForwardConfig& FeedForwardConfig::kCos(double kCos, ClosedLoopSlot slot) {
    if (GetParameter(SparkParameter::kG_0 + (slot * kFFSlotOffset)) !=
        std::nullopt) {
        WPILIB_ReportError(
            wpi::warn::Warning,
            "kG is overriden by kCos, only kCos will be used on Closed Loop "
            "Slot " +
                std::to_string(slot));
        RemoveParameter(SparkParameter::kG_0 + (slot * kFFSlotOffset));
    }
    PutParameter(SparkParameter::kCos_0 + (slot * kFFSlotOffset),
                 static_cast<float>(kCos));
    return *this;
}

FeedForwardConfig& FeedForwardConfig::kCosRatio(double kCosRatio,
                                                ClosedLoopSlot slot) {
    PutParameter(SparkParameter::kCosRatio_0 + (slot * kFFSlotOffset),
                 static_cast<float>(kCosRatio));
    return *this;
}

FeedForwardConfig& FeedForwardConfig::sv(double kS, double kV,
                                         ClosedLoopSlot slot) {
    FeedForwardConfig::kS(kS, slot);
    FeedForwardConfig::kV(kV, slot);
    return *this;
}

FeedForwardConfig& FeedForwardConfig::sva(double kS, double kV, double kA,
                                          ClosedLoopSlot slot) {
    FeedForwardConfig::kS(kS, slot);
    FeedForwardConfig::kV(kV, slot);
    FeedForwardConfig::kA(kA, slot);
    return *this;
}

FeedForwardConfig& FeedForwardConfig::svag(double kS, double kV, double kA,
                                           double kG, ClosedLoopSlot slot) {
    FeedForwardConfig::kS(kS, slot);
    FeedForwardConfig::kV(kV, slot);
    FeedForwardConfig::kA(kA, slot);
    FeedForwardConfig::kG(kG, slot);
    return *this;
}

FeedForwardConfig& FeedForwardConfig::svacr(double kS, double kV, double kA,
                                            double kCos, double kCosRatio,
                                            ClosedLoopSlot slot) {
    FeedForwardConfig::kS(kS, slot);
    FeedForwardConfig::kV(kV, slot);
    FeedForwardConfig::kA(kA, slot);
    FeedForwardConfig::kCos(kCos, slot);
    FeedForwardConfig::kCosRatio(kCosRatio, slot);
    return *this;
}

FeedForwardConfig& FeedForwardConfig::sg(double kS, double kG,
                                         ClosedLoopSlot slot) {
    FeedForwardConfig::kS(kS, slot);
    FeedForwardConfig::kG(kG, slot);
    return *this;
}

FeedForwardConfig& FeedForwardConfig::scr(double kS, double kCos,
                                          double kCosRatio,
                                          ClosedLoopSlot slot) {
    FeedForwardConfig::kS(kS, slot);
    FeedForwardConfig::kCos(kCos, slot);
    FeedForwardConfig::kCosRatio(kCosRatio, slot);
    return *this;
}
